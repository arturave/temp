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
# Czy probowac automatycznej konwersji bryl gietych na blache poleceniem Fusion
# "Convert to Sheet Metal" (sterowanym przez executeTextCommand). Mechanizm jest
# nieudokumentowany - przy problemach ustaw na False (skrypt wypisze wtedy liste
# elementow do recznej konwersji).
ATTEMPT_AUTO_CONVERT = True
# Wewnetrzne ID polecenia "Convert to Sheet Metal". To najlepsza znana nazwa, ale
# moze sie roznic miedzy wersjami Fusion. Aby sprawdzic faktyczne ID na swoim
# komputerze: w palecie Text Commands wpisz  TextCommands.List /hidden  i poszukaj
# pozycji z "SheetMetal"/"Convert", albo zaloguj args.commandId w zdarzeniu
# ui.commandStarting podczas recznego klikniecia przycisku.
CONVERT_CMD_ID = 'ConvertToSheetMetalCmd'


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


def _parallel_face_gap(f1, face):
    """Odleglosc miedzy plaszczyznami dwoch rownoleglych scian (cm) albo None."""
    if not isinstance(face.geometry, adsk.core.Plane):
        return None
    n1 = f1.geometry.normal
    if abs(n1.dotProduct(face.geometry.normal)) < 0.999:
        return None  # nie sa rownolegle
    o1 = f1.geometry.origin
    o2 = face.geometry.origin
    v = adsk.core.Vector3D.create(o2.x - o1.x, o2.y - o1.y, o2.z - o1.z)
    gap = abs(v.dotProduct(n1))
    return gap if gap > 1e-4 else None


def probe_wall_thickness_cm(body):
    """
    Szacowana grubosc materialu (cm): najmniejsza odleglosc miedzy najwieksza
    plaska sciana a rownolegla do niej scianą po drugiej stronie sciany.
    Dziala takze dla elementow gietych (mierzy grubosc scianki). None gdy brak.
    """
    f1 = largest_planar_face(body)
    if f1 is None:
        return None
    best = None
    for face in body.faces:
        if face is f1:
            continue
        gap = _parallel_face_gap(f1, face)
        if gap is not None and (best is None or gap < best):
            best = gap
    return best


def detect_flat_plate(body):
    """
    Sprawdza, czy bryla jest CZYSTA plaska plyta (prosty graniastoslup), ktora
    mozna wyeksportowac bez rozwijania. Zwraca (sciana_bazowa, grubosc_cm) albo None.

    Kryteria:
      - istnieja dwie rownolegle plaskie sciany o zblizonym polu (gora/dol),
      - grubosc = odleglosc miedzy nimi (gap), mala wzgledem rozmiaru obrysu,
      - objetosc ~= pole_sciany * gap  => bryla jest prosta plyta (bez flansz,
        giec ani lokalnych pogrubien). To odrzuca elementy giete, ktore inaczej
        daly by zawyzona grubosc i niepelny obrys.
    """
    f1 = largest_planar_face(body)
    if f1 is None or f1.area <= 0:
        return None
    a1 = f1.area
    extent_cm = a1 ** 0.5
    if extent_cm <= 0:
        return None

    best_gap = None
    for face in body.faces:
        if face is f1:
            continue
        if abs(face.area - a1) > PLATE_AREA_MATCH_TOL * a1:
            continue
        gap = _parallel_face_gap(f1, face)
        if gap is not None and (best_gap is None or gap < best_gap):
            best_gap = gap
    if best_gap is None:
        return None

    if best_gap / extent_cm > PLATE_THICKNESS_RATIO_MAX:
        return None  # zbyt gruba wzgledem obrysu - to nie plyta

    # Weryfikacja "czystej plyty": objetosc zgodna z prostym wyciagnieciem.
    ratio = body.volume / (a1 * best_gap)
    if ratio < 0.90 or ratio > 1.03:
        return None  # flansze/giecia/pogrubienia - wymaga prawdziwego rozwiniecia

    return f1, best_gap


def looks_like_sheet_candidate(body):
    """
    Czy bryla wyglada na element blaszany o jednolitej grubosci, ktory dalo by
    sie rozwinac PO konwersji 'Convert to Sheet Metal'. Zwraca grubosc_mm albo None.
    """
    t_cm = probe_wall_thickness_cm(body)
    if t_cm is None:
        return None
    t_mm = t_cm * 10.0
    if t_mm < 0.3 or t_mm > 8.0:
        return None  # poza typowym zakresem blachy
    f1 = largest_planar_face(body)
    if f1 is None or f1.area <= 0:
        return None
    if t_cm / (f1.area ** 0.5) > PLATE_THICKNESS_RATIO_MAX:
        return None  # zbyt masywna wzgledem obrysu (walek, kostka, profil lity)
    return round(t_mm, 1)


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
    try:
        opts.isSplineConvertedToPolyline = True  # lepsze dla wycinarek laserowych
    except Exception:
        pass
    ok = design.exportManager.execute(opts)
    if ok:
        logger.log('[DXF] {}: zapisano "{}" (blacha gieta, grubosc {} mm, {} szt.)'.format(
            comp.name, filename, format_thickness_mm(thickness_mm), count))
        return 'ok'
    logger.log('[BLAD] {}: eksport DXF wzoru plaskiego nie powiodl sie.'.format(comp.name))
    return 'fail'


# ----------------------------------------------------------------------------
# Automatyczna konwersja na blache (polecenie "Convert to Sheet Metal")
# ----------------------------------------------------------------------------
def try_convert_to_sheet_metal(app, ui, comp, body, logger):
    """
    Proba automatycznej konwersji bryly na konstrukcje blachowa przez wbudowane
    polecenie Fusion 'ConvertToSheetMetalCmd' sterowane text commands. Polecenie
    samo wykrywa grubosc i stosuje aktywna regule blachowa (jak w oknie dialogowym).
    Zwraca True, jesli bryla stala sie blacha.
    """
    face = largest_planar_face(body)
    if face is None:
        return False
    try:
        ui.activeSelections.clear()
        ui.activeSelections.add(face)
        app.executeTextCommand(u'Commands.Start ' + CONVERT_CMD_ID)
        # Poczekaj, az wejscia polecenia beda poprawne do zatwierdzenia.
        try:
            app.executeTextCommand(u'FusionDoc.WaitInputsValidForCommit')
        except Exception:
            pass
        app.executeTextCommand(u'NuCommands.CommitCmd')
    except Exception as e:
        logger.log('[INFO] {}: wyjatek przy auto-konwersji ({}).'.format(comp.name, e))
    finally:
        # Zamknij ewentualnie otwarte okno polecenia, aby nie kolidowalo z kolejna
        # iteracja petli (execute jest nieblokujace - niezatwierdzony dialog
        # zostalby otwarty). Wyczysc tez zaznaczenie.
        try:
            if ui.activeCommand and ui.activeCommand != 'SelectCommand':
                app.executeTextCommand(u'NuCommands.CancelCmd')
        except Exception:
            pass
        try:
            ui.activeSelections.clear()
        except Exception:
            pass

    # Po konwersji bryla moze byc nowym obiektem - pobierz ja na swiezo.
    new_body = largest_solid_body(comp)
    return new_body is not None and body_is_sheet_metal(new_body)


# ----------------------------------------------------------------------------
# Przetwarzanie pojedynczego komponentu
# ----------------------------------------------------------------------------
def process_component(app, ui, comp, count, out_folder, design, logger):
    """Zwraca 'ok' | 'convert' | 'skip' | 'fail'."""
    name = comp.name

    if comp.bRepBodies.count == 0:
        logger.log('[POMIN] {}: brak bryl (zlozenie posrednie / szkic).'.format(name))
        return 'skip'

    body = largest_solid_body(comp)
    if body is None:
        logger.log('[POMIN] {}: brak bryl typu solid.'.format(name))
        return 'skip'

    # a) Blacha gieta zdefiniowana jako sheet metal -> rozwin i eksportuj.
    if body_is_sheet_metal(body):
        try:
            return export_flat_pattern_dxf(design, comp, body, count, out_folder, logger)
        except Exception as e:
            logger.log('[BLAD] {}: rozwiniecie blachy gietej nieudane ({}).'.format(name, e))
            return 'fail'

    # b) Czysta plaska plyta o jednolitej grubosci -> eksport obrysu bezposrednio.
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

    # c) Element gięty o jednolitej grubosci -> kandydat do konwersji na blache.
    cand_mm = looks_like_sheet_candidate(body)
    if cand_mm is not None:
        if ATTEMPT_AUTO_CONVERT:
            logger.log('[KONWERSJA] {}: proba auto-konwersji na blache '
                       '(wstepnie wykryta grubosc ~{} mm)...'.format(
                           name, format_thickness_mm(cand_mm)))
            if try_convert_to_sheet_metal(app, ui, comp, body, logger):
                logger.log('[OK] {}: skonwertowano na blache.'.format(name))
                try:
                    return export_flat_pattern_dxf(
                        design, comp, largest_solid_body(comp), count, out_folder, logger)
                except Exception as e:
                    logger.log('[BLAD] {}: rozwiniecie po konwersji nieudane ({}).'.format(name, e))
                    return 'fail'
            logger.log('[KONWERSJA] {}: auto-konwersja nieudana - wykonaj recznie '
                       '"Convert to Sheet Metal" i uruchom skrypt ponownie.'.format(name))
            return 'convert'
        logger.log('[KONWERSJA] {}: kandydat na blache, wykryta grubosc ~{} mm. '
                   'Wykonaj recznie "Convert to Sheet Metal" i uruchom skrypt ponownie.'.format(
                       name, format_thickness_mm(cand_mm)))
        return 'convert'

    logger.log('[POMIN] {}: nie jest blacha ani plaska plyta '
               '(np. walek, tuleja, odlew, zlaczka, element lity).'.format(name))
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

        n_ok = n_skip = n_fail = n_conv = 0
        convert_names = []
        for data in components.values():
            comp = data['comp']
            count = data['count']
            logger.log('--- Komponent: {} (wystapien: {}) ---'.format(comp.name, count))
            result = process_component(app, ui, comp, count, out_folder, design, logger)
            if result == 'ok':
                n_ok += 1
            elif result == 'convert':
                n_conv += 1
                convert_names.append(comp.name)
            elif result == 'skip':
                n_skip += 1
            else:
                n_fail += 1

        if convert_names:
            logger.log('--- Kandydaci do recznej konwersji na blache ({}): ---'.format(n_conv))
            for nm in convert_names:
                logger.log('    * {}'.format(nm))

        logger.log('=== KONIEC: zapisano {}, do konwersji {}, pominieto {}, '
                   'bledy {} ==='.format(n_ok, n_conv, n_skip, n_fail))

        summary = ('Zakonczono.\n\n'
                   'Zapisane DXF: {}\n'
                   'Do recznej konwersji na blache: {}\n'
                   'Pominiete: {}\n'
                   'Bledy: {}\n\n'
                   'Log: {}').format(n_ok, n_conv, n_skip, n_fail, log_path)
        if convert_names:
            preview = '\n'.join('  - ' + nm for nm in convert_names[:12])
            if n_conv > 12:
                preview += '\n  - ... (+{} wiecej, szczegoly w logu)'.format(n_conv - 12)
            summary += ('\n\nElementy do konwersji "Convert to Sheet Metal" '
                        '(potem uruchom skrypt ponownie):\n' + preview)
        ui.messageBox(summary)

    except Exception:
        msg = 'Blad wykonania skryptu:\n{}'.format(traceback.format_exc())
        if logger:
            logger.log(msg)
        if ui:
            ui.messageBox(msg)
    finally:
        if logger:
            logger.close()
