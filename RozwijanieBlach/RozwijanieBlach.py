# -*- coding: utf-8 -*-
"""
Automatyczne rozwijanie elementow konstrukcji blachowej ze zlozenia (Fusion 360).

Dziala na aktywnym projekcie (Design). Skrypt:
  1. Loguje wszystkie dzialania do pliku .log w folderze docelowym oraz do palety
     "Text Commands" Fusion (Widok > Show Text Commands).
  2. Analizuje drzewo zlozenia i identyfikuje niepowtarzalne komponenty oraz
     liczbe ich wystapien (laczna ilosc sztuk w calym zlozeniu).
  3. Dla kazdego komponentu probuje utworzyc wzor plaski (flat pattern). Elementy,
     ktorych nie da sie rozwinac (tuleje, profile, odlewy, bryly o niejednolitej
     grubosci) sa pomijane i odnotowane w logu.
  4. Eksportuje rozwiniety ksztalt do DXF do wskazanego folderu, nadajac nazwe wg
     maski:  NazwaKomponentu_#X,Xmm_ZZZszt.dxf
     (grubosc z przecinkiem dziesietnym, ilosc uzupelniona do 3 cyfr, np. 005szt).

Grubosc jest odczytywana z geometrii rozwinietej blachy (najmniejszy wymiar bryly
plaskiej). Fusion rozpoznaje grubosc i stosuje wlasny zestaw regul rozwijania
(Sheet Metal Rules) zdefiniowany w projekcie.

UWAGA: Tworzenie wzorow plaskich wymaga trybu parametrycznego (Design History
wlaczona). W trybie bezposrednim (Direct) funkcja nie jest dostepna.
"""

import adsk.core
import adsk.fusion
import traceback
import os
import re
import datetime


# ----------------------------------------------------------------------------
# Logowanie
# ----------------------------------------------------------------------------
class Logger:
    """Zapisuje komunikaty rownoczesnie do pliku i do palety Text Commands."""

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


# ----------------------------------------------------------------------------
# Funkcje pomocnicze
# ----------------------------------------------------------------------------
def sanitize_filename(name):
    """Usuwa znaki niedozwolone w nazwach plikow."""
    return re.sub(r'[<>:"/\\|?*\r\n\t]', '_', name).strip()


def format_thickness_mm(thickness_mm):
    """Formatuje grubosc w mm z przecinkiem dziesietnym, jedno miejsce po przecinku."""
    return '{:.1f}'.format(thickness_mm).replace('.', ',')


def build_filename(comp_name, thickness_mm, count):
    """Buduje nazwe pliku wg maski NazwaKomponentu_#X,Xmm_ZZZszt.dxf."""
    safe = sanitize_filename(comp_name)
    thick = format_thickness_mm(thickness_mm)
    return '{}_#{}mm_{:03d}szt.dxf'.format(safe, thick, count)


def largest_planar_face(body):
    """Zwraca plaska sciane o najwiekszej powierzchni (lub None)."""
    best = None
    best_area = -1.0
    for face in body.faces:
        if isinstance(face.geometry, adsk.core.Plane) and face.area > best_area:
            best = face
            best_area = face.area
    return best


def largest_solid_body(component):
    """Zwraca bryle solid o najwiekszej objetosci (lub None)."""
    best = None
    best_vol = -1.0
    for body in component.bRepBodies:
        if body.isSolid and body.volume > best_vol:
            best = body
            best_vol = body.volume
    return best


def min_bbox_dimension_mm(body):
    """Najmniejszy wymiar bryly wg bounding box, przeliczony z cm na mm."""
    bb = body.boundingBox
    dims = [
        bb.maxPoint.x - bb.minPoint.x,
        bb.maxPoint.y - bb.minPoint.y,
        bb.maxPoint.z - bb.minPoint.z,
    ]
    return min(dims) * 10.0  # jednostki wewnetrzne Fusion to centymetry


# ----------------------------------------------------------------------------
# Analiza drzewa zlozenia
# ----------------------------------------------------------------------------
def gather_unique_components(root_comp, logger):
    """
    Przechodzi wszystkie wystapienia w calym zlozeniu i grupuje je po komponencie.
    Zwraca slownik: nazwa_komponentu -> {'comp': Component, 'count': int}.
    """
    result = {}
    occurrences = root_comp.allOccurrences
    for i in range(occurrences.count):
        comp = occurrences.item(i).component
        key = comp.name
        if key not in result:
            result[key] = {'comp': comp, 'count': 0}
        result[key]['count'] += 1

    # Projekt jednoczesciowy (brak wystapien) - rozpatrz komponent glowny.
    if not result and root_comp.bRepBodies.count > 0:
        result[root_comp.name] = {'comp': root_comp, 'count': 1}

    logger.log('Wystapien lacznie: {}, niepowtarzalnych komponentow: {}'.format(
        occurrences.count, len(result)))
    return result


# ----------------------------------------------------------------------------
# Tworzenie wzoru plaskiego i eksport DXF
# ----------------------------------------------------------------------------
def export_flat_pattern_dxf(flat_pattern, comp, count, out_folder, root_comp, logger):
    """
    Eksportuje rozwiniety ksztalt do DXF.
    Bryla wzoru plaskiego jest kopiowana do komponentu glownego, aby utworzyc na
    jej najwiekszej scianie szkic i zapisac go jako DXF (Sketch.saveAsDXF).
    Zwraca (grubosc_mm, sukces_bool).
    """
    flat_body = None
    best_vol = -1.0
    for body in flat_pattern.bodies:
        if body.volume > best_vol:
            flat_body = body
            best_vol = body.volume
    if flat_body is None:
        raise RuntimeError('brak bryly we wzorze plaskim')

    copied = flat_body.copyToComponent(root_comp)
    sketch = None
    try:
        thickness_mm = round(min_bbox_dimension_mm(copied), 1)

        face = largest_planar_face(copied)
        if face is None:
            raise RuntimeError('brak plaskiej sciany w rozwinietej bryle')

        sketch = root_comp.sketches.add(face)
        sketch.project(face)  # rzutuje kontur zewnetrzny i otwory na szkic

        filename = build_filename(comp.name, thickness_mm, count)
        filepath = os.path.join(out_folder, filename)
        ok = sketch.saveAsDXF(filepath)

        if ok:
            logger.log('[DXF] {}: zapisano "{}" (grubosc {} mm, {} szt.)'.format(
                comp.name, filename, format_thickness_mm(thickness_mm), count))
        else:
            logger.log('[BLAD] {}: saveAsDXF zwrocilo False'.format(comp.name))
        return thickness_mm, ok
    finally:
        # Sprzatanie artefaktow uzytych tylko do eksportu.
        if sketch is not None:
            try:
                sketch.deleteMe()
            except Exception:
                pass
        try:
            copied.deleteMe()
        except Exception:
            pass


def process_component(comp, count, out_folder, root_comp, logger):
    """
    Przetwarza pojedynczy komponent: tworzy wzor plaski (jesli to mozliwe) i
    eksportuje DXF. Zwraca 'ok' | 'skip' | 'fail'.
    """
    name = comp.name

    if comp.bRepBodies.count == 0:
        logger.log('[POMIN] {}: brak bryl (zlozenie posrednie / szkic).'.format(name))
        return 'skip'

    body = largest_solid_body(comp)
    if body is None:
        logger.log('[POMIN] {}: brak bryl typu solid.'.format(name))
        return 'skip'

    # Utworzenie lub pobranie wzoru plaskiego.
    flat_pattern = None
    try:
        if comp.hasFlatPattern:
            flat_pattern = comp.flatPattern
            logger.log('[INFO] {}: wzor plaski juz istnieje - uzywam istniejacego.'.format(name))
        else:
            stationary = largest_planar_face(body)
            if stationary is None:
                logger.log('[POMIN] {}: brak plaskiej sciany bazowej.'.format(name))
                return 'skip'
            flat_pattern = comp.createFlatPattern(stationary)
            logger.log('[OK] {}: utworzono wzor plaski.'.format(name))
    except Exception as e:
        # Najczestszy powod: element nie jest blacha (tuleja, profil, odlew,
        # bryla o niejednolitej grubosci) - Fusion nie potrafi go rozwinac.
        logger.log('[POMIN] {}: nie da sie rozwinac jako blachy ({}).'.format(name, e))
        return 'skip'

    if flat_pattern is None or flat_pattern.bodies.count == 0:
        logger.log('[POMIN] {}: pusty wzor plaski.'.format(name))
        return 'skip'

    try:
        export_flat_pattern_dxf(flat_pattern, comp, count, out_folder, root_comp, logger)
        return 'ok'
    except Exception as e:
        logger.log('[BLAD] {}: eksport DXF nieudany ({}).'.format(name, e))
        return 'fail'


# ----------------------------------------------------------------------------
# Punkt wejscia
# ----------------------------------------------------------------------------
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

        # Wybor folderu docelowego.
        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Wybierz folder docelowy dla plikow DXF i logu'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        out_folder = folder_dlg.folder

        # Inicjalizacja logu.
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(out_folder, 'rozwijanie_blach_{}.log'.format(stamp))
        logger = Logger(log_path, app)

        logger.log('=== START: automatyczne rozwijanie blach ===')
        try:
            doc_name = design.parentDocument.name
        except Exception:
            doc_name = '(nieznany)'
        logger.log('Projekt: {}'.format(doc_name))
        logger.log('Folder docelowy: {}'.format(out_folder))

        if design.designType == adsk.fusion.DesignTypes.DirectDesignType:
            logger.log('[UWAGA] Projekt w trybie bezposrednim (Direct). Wzory plaskie '
                       'wymagaja trybu parametrycznego. Wlacz historie projektu i '
                       'uruchom skrypt ponownie.')

        # Analiza drzewa i przetwarzanie.
        components = gather_unique_components(root_comp, logger)

        n_ok = n_skip = n_fail = 0
        for data in components.values():
            comp = data['comp']
            count = data['count']
            logger.log('--- Komponent: {} (wystapien: {}) ---'.format(comp.name, count))
            result = process_component(comp, count, out_folder, root_comp, logger)
            if result == 'ok':
                n_ok += 1
            elif result == 'skip':
                n_skip += 1
            else:
                n_fail += 1

        logger.log('=== KONIEC: zapisano {}, pominieto {}, bledy {} ==='.format(
            n_ok, n_skip, n_fail))

        ui.messageBox(
            'Zakonczono.\n\n'
            'Zapisane DXF: {}\n'
            'Pominiete: {}\n'
            'Bledy: {}\n\n'
            'Log: {}'.format(n_ok, n_skip, n_fail, log_path))

    except Exception:
        msg = 'Blad wykonania skryptu:\n{}'.format(traceback.format_exc())
        if logger:
            logger.log(msg)
        if ui:
            ui.messageBox(msg)
    finally:
        if logger:
            logger.close()
