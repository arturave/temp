# -*- coding: utf-8 -*-
"""
Automatyczne rozwijanie elementow konstrukcji blachowej ze zlozenia (Fusion 360).

Dziala na aktywnym projekcie (Design). Skrypt:
  1. Loguje wszystkie dzialania do pliku .log w folderze docelowym oraz do palety
     "Text Commands" Fusion (Widok > Show Text Commands).
  2. Analizuje drzewo zlozenia i identyfikuje niepowtarzalne komponenty oraz
     liczbe ich wystapien (laczna ilosc sztuk w calym zlozeniu).
  3. Dla kazdego komponentu wyznacza ksztalt do wyciecia, na jeden z dwoch sposobow:
       a) BLACHA GIETA - gdy bryla jest juz konstrukcja blachowa
          (BRepBody.isSheetMetal == True). Tworzony jest wzor plaski
          (createFlatPattern) wg regul rozwijania (Sheet Metal Rules) projektu.
       b) PLASKA PLYTA - gdy bryla jest plaska plyta o jednolitej grubosci
          (np. element ciety laserem/plazma, niezdefiniowany jako blacha).
          Eksportowany jest bezposrednio obrys najwiekszej sciany - rozwijanie
          nie jest potrzebne, bo element jest juz plaski.
     Pozostale elementy (tuleje, profile, walki, odlewy, elementy giete
     niezdefiniowane jako blacha) sa pomijane i odnotowane w logu.
  4. Eksportuje ksztalt do DXF do wskazanego folderu, nadajac nazwe wg maski:
        NazwaKomponentu_#X,Xmm_ZZZszt.dxf
     (grubosc z przecinkiem dziesietnym, ilosc uzupelniona do 3 cyfr, np. 005szt).

Grubosc jest odczytywana z geometrii (objetosc / pole najwiekszej sciany).

OGRANICZENIE API FUSION: nie istnieje programowa komenda "Convert to Sheet Metal".
Bryly giete, ktore nie sa zdefiniowane jako blacha, trzeba przekonwertowac recznie
(Sheet Metal > Convert to Sheet Metal) i uruchomic skrypt ponownie. Skrypt
wypisuje liste takich elementow w logu.

UWAGA: Tworzenie wzorow plaskich wymaga trybu parametrycznego (Design History
wlaczona). W trybie bezposrednim (Direct) funkcja nie jest dostepna.
"""

import adsk.core
import adsk.fusion
import traceback
import os
import re
import datetime


# Maksymalny stosunek grubosci do "rozpietosci" sciany (sqrt z pola), powyzej
# ktorego bryla nie jest juz traktowana jako plaska plyta (odrzuca walki,
# kostki, profile). 0.2 => grubosc < 20% rozmiaru obrysu.
PLATE_THICKNESS_RATIO_MAX = 0.2
# Tolerancja zgodnosci pol gornej i dolnej sciany plyty (10%).
PLATE_AREA_MATCH_TOL = 0.10


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
    """Grubosc w mm z przecinkiem dziesietnym, jedno miejsce po przecinku."""
    return '{:.1f}'.format(thickness_mm).replace('.', ',')


def build_filename(comp_name, thickness_mm, count):
    """Nazwa pliku wg maski NazwaKomponentu_#X,Xmm_ZZZszt.dxf."""
    safe = sanitize_filename(comp_name)
    thick = format_thickness_mm(thickness_mm)
    return '{}_#{}mm_{:03d}szt.dxf'.format(safe, thick, count)


def largest_planar_face(body):
    """Plaska sciana o najwiekszym polu (lub None)."""
    best = None
    best_area = -1.0
    for face in body.faces:
        if isinstance(face.geometry, adsk.core.Plane) and face.area > best_area:
            best = face
            best_area = face.area
    return best


def largest_solid_body(component):
    """Bryla solid o najwiekszej objetosci (lub None)."""
    best = None
    best_vol = -1.0
    for body in component.bRepBodies:
        if body.isSolid and body.volume > best_vol:
            best = body
            best_vol = body.volume
    return best


def body_is_sheet_metal(body):
    """True, jesli bryla jest konstrukcja blachowa mozliwa do rozwiniecia."""
    try:
        return bool(body.isSheetMetal)
    except Exception:
        # Starsze wersje API moga nie miec tej wlasciwosci.
        return False


def detect_flat_plate(body):
    """
    Sprawdza, czy bryla jest plaska plyta o jednolitej grubosci.
    Zwraca (sciana_bazowa, grubosc_cm) albo None.

    Kryteria: istnieja dwie rownolegle plaskie sciany o zblizonym polu (gora/dol),
    odlegle o grubosc = objetosc / pole sciany, przy czym grubosc jest mala
    wzgledem rozmiaru obrysu (odrzuca walki, kostki, profile).
    """
    f1 = largest_planar_face(body)
    if f1 is None or f1.area <= 0:
        return None

    thickness_cm = body.volume / f1.area
    extent_cm = f1.area ** 0.5
    if extent_cm <= 0:
        return None
    if thickness_cm / extent_cm > PLATE_THICKNESS_RATIO_MAX:
        return None  # zbyt "gruba" wzgledem obrysu - to nie plyta

    n1 = f1.geometry.normal
    o1 = f1.geometry.origin

    for face in body.faces:
        if face is f1:
            continue
        if not isinstance(face.geometry, adsk.core.Plane):
            continue
        if abs(face.area - f1.area) > PLATE_AREA_MATCH_TOL * f1.area:
            continue
        n2 = face.geometry.normal
        dot = n1.dotProduct(n2)
        # Plaszczyzny gora/dol musza byc rownolegle. Plane.normal zwraca normalna
        # geometryczna plaszczyzny (nie zorientowana na zewnatrz), wiec moga byc
        # rownolegle (dot ~ +1) albo antyrownolegle (dot ~ -1) - oba przypadki OK.
        if abs(dot) < 0.999:
            continue
        o2 = face.geometry.origin
        gap = adsk.core.Vector3D.create(o2.x - o1.x, o2.y - o1.y, o2.z - o1.z)
        dist = abs(gap.dotProduct(n1))
        if abs(dist - thickness_cm) <= max(0.005, 0.15 * thickness_cm):
            return f1, thickness_cm
    return None


# ----------------------------------------------------------------------------
# Analiza drzewa zlozenia
# ----------------------------------------------------------------------------
def gather_unique_components(root_comp, logger):
    """
    Grupuje wszystkie wystapienia w calym zlozeniu po komponencie.
    Zwraca slownik: nazwa -> {'comp': Component, 'count': int}.
    """
    result = {}
    occurrences = root_comp.allOccurrences
    for i in range(occurrences.count):
        comp = occurrences.item(i).component
        key = comp.name
        if key not in result:
            result[key] = {'comp': comp, 'count': 0}
        result[key]['count'] += 1

    if not result and root_comp.bRepBodies.count > 0:
        result[root_comp.name] = {'comp': root_comp, 'count': 1}

    logger.log('Wystapien lacznie: {}, niepowtarzalnych komponentow: {}'.format(
        occurrences.count, len(result)))
    return result


# ----------------------------------------------------------------------------
# Eksport DXF
# ----------------------------------------------------------------------------
def export_face_as_dxf(comp, face, filepath, logger):
    """Rzutuje obrys sciany (z otworami) na szkic i zapisuje go jako DXF."""
    sketch = comp.sketches.add(face)
    try:
        for edge in face.edges:
            try:
                sketch.project(edge)
            except Exception:
                pass
        return sketch.saveAsDXF(filepath)
    finally:
        try:
            sketch.deleteMe()
        except Exception:
            pass


def export_flat_pattern_dxf(design, comp, body, count, out_folder, logger):
    """Tworzy/uzywa wzoru plaskiego blachy gietej i eksportuje DXF."""
    try:
        flat = comp.flatPattern
    except Exception:
        flat = None

    if flat is None:
        stationary = largest_planar_face(body)
        if stationary is None:
            logger.log('[POMIN] {}: brak plaskiej sciany bazowej dla wzoru plaskiego.'.format(comp.name))
            return 'skip'
        flat = comp.createFlatPattern(stationary)
        logger.log('[OK] {}: utworzono wzor plaski (blacha gieta).'.format(comp.name))
    else:
        logger.log('[INFO] {}: wzor plaski juz istnieje - uzywam istniejacego.'.format(comp.name))

    # Grubosc z bryly plaskiej: objetosc / pole gornej sciany.
    try:
        thickness_cm = flat.flatBody.volume / flat.topFace.area
    except Exception:
        bb = flat.flatBody.boundingBox
        thickness_cm = min(
            bb.maxPoint.x - bb.minPoint.x,
            bb.maxPoint.y - bb.minPoint.y,
            bb.maxPoint.z - bb.minPoint.z,
        )
    thickness_mm = round(thickness_cm * 10.0, 1)

    filename = build_filename(comp.name, thickness_mm, count)
    filepath = os.path.join(out_folder, filename)

    opts = design.exportManager.createDXFFlatPatternExportOptions(filepath, flat)
    ok = design.exportManager.execute(opts)
    if ok:
        logger.log('[DXF] {}: zapisano "{}" (blacha gieta, grubosc {} mm, {} szt.)'.format(
            comp.name, filename, format_thickness_mm(thickness_mm), count))
        return 'ok'
    logger.log('[BLAD] {}: eksport DXF wzoru plaskiego nie powiodl sie.'.format(comp.name))
    return 'fail'


# ----------------------------------------------------------------------------
# Przetwarzanie pojedynczego komponentu
# ----------------------------------------------------------------------------
def process_component(comp, count, out_folder, design, logger):
    """Zwraca 'ok' | 'skip' | 'fail'."""
    name = comp.name

    if comp.bRepBodies.count == 0:
        logger.log('[POMIN] {}: brak bryl (zlozenie posrednie / szkic).'.format(name))
        return 'skip'

    body = largest_solid_body(comp)
    if body is None:
        logger.log('[POMIN] {}: brak bryl typu solid.'.format(name))
        return 'skip'

    # a) Blacha gieta zdefiniowana jako sheet metal.
    if body_is_sheet_metal(body):
        try:
            return export_flat_pattern_dxf(design, comp, body, count, out_folder, logger)
        except Exception as e:
            logger.log('[BLAD] {}: rozwiniecie blachy gietej nieudane ({}).'.format(name, e))
            return 'fail'

    # b) Plaska plyta o jednolitej grubosci - eksport obrysu bezposrednio.
    plate = detect_flat_plate(body)
    if plate is not None:
        face, thickness_cm = plate
        thickness_mm = round(thickness_cm * 10.0, 1)
        filename = build_filename(name, thickness_mm, count)
        filepath = os.path.join(out_folder, filename)
        try:
            ok = export_face_as_dxf(comp, face, filepath, logger)
        except Exception as e:
            logger.log('[BLAD] {}: eksport obrysu plyty nieudany ({}).'.format(name, e))
            return 'fail'
        if ok:
            logger.log('[DXF] {}: zapisano "{}" (plaska plyta, grubosc {} mm, {} szt.)'.format(
                name, filename, format_thickness_mm(thickness_mm), count))
            return 'ok'
        logger.log('[BLAD] {}: saveAsDXF zwrocilo False.'.format(name))
        return 'fail'

    # c) Element nie jest ani blacha, ani plaska plyta.
    logger.log('[POMIN] {}: nie jest blacha ani plaska plyta. Pominieto '
               '(np. profil, tuleja, walek, odlew lub element giety wymagajacy '
               'recznej konwersji "Convert to Sheet Metal").'.format(name))
    return 'skip'


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

        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Wybierz folder docelowy dla plikow DXF i logu'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        out_folder = folder_dlg.folder

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
                       'blach gietych wymagaja trybu parametrycznego. Wlacz historie '
                       'projektu, jesli element jest blacha gieta.')

        components = gather_unique_components(root_comp, logger)

        n_ok = n_skip = n_fail = 0
        for data in components.values():
            comp = data['comp']
            count = data['count']
            logger.log('--- Komponent: {} (wystapien: {}) ---'.format(comp.name, count))
            result = process_component(comp, count, out_folder, design, logger)
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
