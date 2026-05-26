# -*- coding: utf-8 -*-
"""
Separuje bryly w ramach jednego detalu i zapisuje kazda jako osobny plik STEP
(Fusion 360). Dla dokumentow, ktore nie sa zlozeniem komponentow, lecz zawieraja
wiele osobnych bryl (np. Bryla1..Bryla18) w komponencie glownym.

Dziala na aktywnym projekcie (Design). Skrypt:
  1. Loguje dzialania do pliku .log w folderze docelowym oraz do palety
     "Text Commands" Fusion.
  2. Zbiera wszystkie bryly typu solid z komponentu glownego.
  3. (opcjonalnie) Grupuje identyczne bryly po geometrii (objetosc, pole, liczba
     scian i krawedzi - niezmienniki obrotu/przesuniecia) i zlicza je.
  4. Eksportuje kazda unikalna bryle do osobnego pliku STEP wg maski:
        NazwaBryly_ZZZszt.step      (ilosc uzupelniona do 3 cyfr, np. 005szt).

UWAGA: grupowanie po objetosci/polu/liczbie scian nie odroznia czesci LUSTRZANYCH
(np. lewa/prawa) - maja te same niezmienniki. Jesli to problem, ustaw
GROUP_IDENTICAL_BODIES = False, aby zapisac kazda bryle osobno (po 1 szt.).
"""

import adsk.core
import adsk.fusion
import traceback
import os
import re
import datetime


# Grupowanie identycznych bryl i zliczanie ich do nazwy (_ZZZszt). False => kazda
# bryla zapisywana osobno (zawsze 001szt).
GROUP_IDENTICAL_BODIES = True


class Logger:
    def __init__(self, path, app):
        self.path = path
        self.app = app
        self._file = open(path, 'a', encoding='utf-8')

    def log(self, msg):
        stamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        line = '[{}] {}'.format(stamp, msg)
        self._file.write(line + '\n')
        self._file.flush()
        try:
            self.app.log(line)
        except Exception:
            pass

    def close(self):
        try:
            self._file.close()
        except Exception:
            pass


def sanitize_filename(name):
    return re.sub(r'[<>:"/\\|?*\r\n\t]', '_', name).strip()


def format_thickness_mm(mm):
    return '{:.1f}'.format(mm).replace('.', ',')


def build_filename(detail_name, body_name, thickness_mm, count):
    """Maska: NazwaDetalu_NazwaBryly_#X,Xmm_ZZZszt.step."""
    return '{}_{}_#{}mm_{:03d}szt.step'.format(
        sanitize_filename(detail_name), sanitize_filename(body_name),
        format_thickness_mm(thickness_mm), count)


def largest_planar_face(body):
    best = None
    best_area = -1.0
    for f in body.faces:
        if isinstance(f.geometry, adsk.core.Plane) and f.area > best_area:
            best = f
            best_area = f.area
    return best


def body_thickness_mm(body):
    """Grubosc materialu w mm: najmniejsza odleglosc miedzy najwieksza plaska
    sciana a rownolegla scianka po drugiej stronie (niezmiennik orientacji).
    Fallback: najmniejszy wymiar bryly z bounding box."""
    f1 = largest_planar_face(body)
    gap_cm = None
    if f1 is not None:
        n1 = f1.geometry.normal
        o1 = f1.geometry.origin
        for f in body.faces:
            if f is f1 or not isinstance(f.geometry, adsk.core.Plane):
                continue
            if abs(n1.dotProduct(f.geometry.normal)) < 0.999:
                continue
            o2 = f.geometry.origin
            v = adsk.core.Vector3D.create(o2.x - o1.x, o2.y - o1.y, o2.z - o1.z)
            gap = abs(v.dotProduct(n1))
            if gap > 1e-4 and (gap_cm is None or gap < gap_cm):
                gap_cm = gap
    if gap_cm is None:
        bb = body.boundingBox
        gap_cm = min(bb.maxPoint.x - bb.minPoint.x,
                     bb.maxPoint.y - bb.minPoint.y,
                     bb.maxPoint.z - bb.minPoint.z)
    return round(gap_cm * 10.0, 1)


def body_signature(body):
    """Niezmiennik ksztaltu odporny na obrot i przesuniecie."""
    return (round(body.volume, 4), round(body.area, 4),
            body.faces.count, body.edges.count)


def gather_unique_bodies(root_comp, group, logger):
    """
    Zwraca liste grup: {'rep': BRepBody, 'count': int, 'members': [nazwy]}.
    Przy group=False kazda bryla to osobna grupa o liczbie 1.
    """
    bodies = [b for b in root_comp.bRepBodies if b.isSolid]
    groups = []
    if group:
        index = {}
        for b in bodies:
            sig = body_signature(b)
            g = index.get(sig)
            if g is None:
                g = {'rep': b, 'count': 1, 'members': [b.name]}
                index[sig] = g
                groups.append(g)
            else:
                g['count'] += 1
                g['members'].append(b.name)
    else:
        for b in bodies:
            groups.append({'rep': b, 'count': 1, 'members': [b.name]})

    logger.log('Bryl solid: {}, unikalnych po grupowaniu: {}'.format(len(bodies), len(groups)))
    return groups


def unique_path(folder, filename):
    """Gwarantuje unikalna sciezke - dokleja _2, _3, ... gdy plik juz istnieje."""
    path = os.path.join(folder, filename)
    if not os.path.exists(path):
        return path
    base, ext = os.path.splitext(filename)
    i = 2
    while True:
        candidate = os.path.join(folder, '{}_{}{}'.format(base, i, ext))
        if not os.path.exists(candidate):
            return candidate
        i += 1


def export_body_step(export_mgr, root_comp, detail_name, group, out_folder, logger):
    """Eksportuje reprezentanta grupy do STEP. Zwraca 'ok'|'fail'.
    STEP eksportuje KOMPONENT, nie pojedyncza bryle - dlatego kopiujemy bryle do
    tymczasowego komponentu, eksportujemy go i usuwamy."""
    body = group['rep']
    count = group['count']
    thickness_mm = body_thickness_mm(body)
    filename = build_filename(detail_name, body.name, thickness_mm, count)
    filepath = unique_path(out_folder, filename)

    temp_occ = None
    ok = False
    try:
        temp_occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        try:
            temp_occ.component.name = sanitize_filename(body.name)
        except Exception:
            pass
        body.copyToComponent(temp_occ)
        opts = export_mgr.createSTEPExportOptions(filepath, temp_occ.component)
        ok = export_mgr.execute(opts)
    except Exception as e:
        logger.log('[BLAD] {}: eksport STEP nieudany ({}).'.format(body.name, e))
    finally:
        if temp_occ is not None:
            try:
                temp_occ.deleteMe()
            except Exception:
                pass

    if ok:
        extra = ''
        if count > 1:
            extra = '  [identyczne: {}]'.format(', '.join(group['members']))
        logger.log('[STEP] {}: zapisano "{}" (grubosc {} mm, {} szt.){}'.format(
            body.name, os.path.basename(filepath), format_thickness_mm(thickness_mm), count, extra))
        return 'ok'
    logger.log('[BLAD] {}: eksport nieudany.'.format(body.name))
    return 'fail'


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    logger = None
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Otworz projekt (Design) przed uruchomieniem skryptu.')
            return

        root_comp = design.rootComponent
        if root_comp.bRepBodies.count == 0:
            ui.messageBox('Komponent glowny nie zawiera bryl. Ten skrypt dziala dla '
                          'detalu z wieloma brylami w komponencie glownym.')
            return

        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Wybierz folder docelowy dla plikow STEP i logu'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        out_folder = folder_dlg.folder

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(out_folder, 'separuj_bryly_{}.log'.format(stamp))
        logger = Logger(log_path, app)

        logger.log('=== START: separacja bryl do STEP ===')
        try:
            doc_name = design.parentDocument.name
        except Exception:
            doc_name = '(nieznany)'
        logger.log('Projekt: {}'.format(doc_name))
        logger.log('Folder docelowy: {}'.format(out_folder))
        logger.log('Grupowanie identycznych bryl: {}'.format(
            'TAK' if GROUP_IDENTICAL_BODIES else 'NIE'))

        groups = gather_unique_bodies(root_comp, GROUP_IDENTICAL_BODIES, logger)
        export_mgr = design.exportManager
        detail_name = root_comp.name

        n_ok = n_fail = 0
        for g in groups:
            res = export_body_step(export_mgr, root_comp, detail_name, g, out_folder, logger)
            if res == 'ok':
                n_ok += 1
            else:
                n_fail += 1

        logger.log('=== KONIEC: zapisano {}, bledy {} ==='.format(n_ok, n_fail))

        ui.messageBox(
            'Zakonczono.\n\n'
            'Zapisane pliki STEP: {}\n'
            'Bledy: {}\n\n'
            'Log: {}'.format(n_ok, n_fail, log_path))

    except Exception:
        msg = 'Blad wykonania skryptu:\n{}'.format(traceback.format_exc())
        if logger:
            logger.log(msg)
        if ui:
            ui.messageBox(msg)
    finally:
        if logger:
            logger.close()
