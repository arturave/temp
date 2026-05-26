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


def export_body_step(export_mgr, group, out_folder, logger):
    """Eksportuje reprezentanta grupy do STEP. Zwraca 'ok'|'fail'."""
    body = group['rep']
    count = group['count']
    filename = '{}_{:03d}szt.step'.format(sanitize_filename(body.name), count)
    filepath = unique_path(out_folder, filename)
    try:
        opts = export_mgr.createSTEPExportOptions(filepath, body)
        ok = export_mgr.execute(opts)
    except Exception as e:
        logger.log('[BLAD] {}: eksport STEP nieudany ({}).'.format(body.name, e))
        return 'fail'
    if ok:
        extra = ''
        if count > 1:
            extra = '  [identyczne: {}]'.format(', '.join(group['members']))
        logger.log('[STEP] {}: zapisano "{}" ({} szt.){}'.format(
            body.name, os.path.basename(filepath), count, extra))
        return 'ok'
    logger.log('[BLAD] {}: execute() zwrocilo False.'.format(body.name))
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

        n_ok = n_fail = 0
        for g in groups:
            res = export_body_step(export_mgr, g, out_folder, logger)
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
