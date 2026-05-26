# -*- coding: utf-8 -*-
"""
Automatyczne rozwijanie elementow konstrukcji blachowej ze zlozenia (Fusion 360).

Dziala na aktywnym projekcie (Design). Skrypt:
  1. Loguje wszystkie dzialania do pliku .log w folderze docelowym oraz do palety
     "Text Commands" Fusion (Widok > Show Text Commands).
  2. Analizuje drzewo zlozenia i identyfikuje niepowtarzalne komponenty oraz
     liczbe ich wystapien (laczna ilosc sztuk w calym zlozeniu).
  3. Przetwarza detale sekwencyjnie (kreator pol-automatyczny), na jeden z trzech
     sposobow:
       a) BLACHA GIETA - gdy bryla jest juz konstrukcja blachowa
          (BRepBody.isSheetMetal == True). Tworzony jest wzor plaski
          (createFlatPattern) wg regul rozwijania (Sheet Metal Rules) i od razu
          zapisywany DXF - bez pytania uzytkownika.
       b) PLASKA PLYTA - czysta plyta o jednolitej grubosci (graniastoslup).
          Eksportowany jest obrys najwiekszej sciany - bez konwersji.
       c) ELEMENT GIETY (kandydat na blache) - skrypt zaznacza sciane i OTWIERA
          okno "Convert to Sheet Metal" (Fusion sam wykrywa grubosc). Uzytkownik
          klika OK lub Anuluj:
            * OK     -> tworzony wzor plaski i zapisywany DXF,
            * Anuluj -> detal pomijany, przechodzimy do nastepnego.
     Pozostale elementy (tuleje, walki, odlewy, zlaczki) sa pomijane.
  4. Eksportuje ksztalt do DXF do wskazanego folderu, nadajac nazwe wg maski:
        NazwaKomponentu_#X,Xmm_ZZZszt.dxf
     (grubosc z przecinkiem dziesietnym, ilosc uzupelniona do 3 cyfr, np. 005szt).
     DXF wzoru plaskiego zawiera tylko linie srodkowe giecia (bez linii zakresu),
     a splajny sa zamieniane na polilinie (pod wycinarki laserowe).

Grubosc blachy gietej i plyty jest odczytywana z geometrii, a dla elementow
konwertowanych - wykrywana przez samo polecenie Fusion "Convert to Sheet Metal".

UWAGA: Tworzenie wzorow plaskich i konwersja wymagaja trybu parametrycznego
(Design History wlaczona) oraz EDYTOWALNEGO dokumentu (nie "tylko do odczytu").
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
    Zwraca slownik: nazwa -> {'comp': Component, 'count': int, 'occ': Occurrence}.
    'occ' to reprezentatywne wystapienie - potrzebne do zaznaczania scian w
    kontekscie zlozenia (proxy).
    """
    result = {}
    occurrences = root_comp.allOccurrences
    for i in range(occurrences.count):
        occ = occurrences.item(i)
        comp = occ.component
        key = comp.name
        if key not in result:
            result[key] = {'comp': comp, 'count': 0, 'occ': occ}
        result[key]['count'] += 1

    if not result and root_comp.bRepBodies.count > 0:
        result[root_comp.name] = {'comp': root_comp, 'count': 1, 'occ': None}

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
    # Tylko linie srodkowe giecia; bez linii zakresu giecia. Splajny -> polilinie.
    try:
        opts.isCenterLinesExported = True
    except Exception:
        pass
    try:
        opts.isExtentLinesExported = False
    except Exception:
        pass
    try:
        opts.isSplineConvertedToPolyline = True
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
# Pol-automatyczny kreator (wizard) - sekwencyjne przetwarzanie detali
# ----------------------------------------------------------------------------
# Fusion nie pozwala czekac synchronicznie na zamkniecie okna dialogowego, a
# CommandDefinition.execute() jest nieblokujace. Dlatego detale przetwarzamy jako
# maszyne stanow sterowana zdarzeniami:
#   - dla detalu wymagajacego konwersji otwieramy okno "Convert to Sheet Metal"
#     (uzytkownik klika OK lub Anuluj),
#   - zdarzenie commandTerminated mowi, czy zatwierdzono (OK) czy anulowano,
#   - po OK tworzymy wzor plaski i zapisujemy DXF, po Anuluj pomijamy detal,
#   - kolejny detal uruchamiamy przez wlasne zdarzenie (custom event), aby nie
#     wywolywac polecenia z wnetrza obslugi zdarzenia (re-entrancy).
NEXT_EVENT_ID = 'rozwijanie_blach_next_part'

# Referencje globalne, aby kreator i handlery nie zostaly usuniete po run().
_wizard = None
_handlers = []
_candidates_logged = False


def log_command_id_candidates(ui, logger):
    """Wypisuje do logu ID polecen, ktore moga byc komenda konwersji na blache
    (zawieraja w ID 'sheet'/'convert'/'flat'). Pomaga ustawic CONVERT_CMD_ID."""
    global _candidates_logged
    if _candidates_logged:
        return
    _candidates_logged = True
    try:
        defs = ui.commandDefinitions
        found = []
        for i in range(defs.count):
            try:
                cid = defs.item(i).id
            except Exception:
                continue
            low = cid.lower()
            if 'sheetmetal' in low or 'convert' in low or 'flatpattern' in low:
                found.append(cid)
        if found:
            logger.log('Mozliwe ID polecen do CONVERT_CMD_ID (zawieraja sheetmetal/convert/flatpattern):')
            for cid in sorted(set(found)):
                logger.log('    ? ' + cid)
        else:
            logger.log('Nie znaleziono kandydatow ID w commandDefinitions. '
                       'Uzyj skryptu pomocniczego WykryjIDPolecenia.')
    except Exception as e:
        logger.log('Skan commandDefinitions nieudany: {}'.format(e))


class Wizard(object):
    def __init__(self, app, ui, design, out_folder, logger, queue, log_path):
        self.app = app
        self.ui = ui
        self.design = design
        self.out_folder = out_folder
        self.logger = logger
        self.queue = queue
        self.log_path = log_path
        self.idx = 0
        self.pending = None          # wynik ostatniego okna konwersji do rozliczenia
        self.waiting = False         # czy czekamy na zamkniecie okna konwersji
        self.waiting_comp = None
        self.waiting_count = 0
        self.n_ok = 0
        self.n_skip = 0
        self.n_fail = 0
        self.n_cancel = 0
        self.cancelled = []
        self.next_event = None
        self._finished = False

    # -- start / koniec ------------------------------------------------------
    def start(self):
        try:
            self.app.unregisterCustomEvent(NEXT_EVENT_ID)
        except Exception:
            pass
        self.next_event = self.app.registerCustomEvent(NEXT_EVENT_ID)
        next_handler = _NextPartHandler(self)
        self.next_event.add(next_handler)
        _handlers.append(next_handler)

        term_handler = _CommandTerminatedHandler(self)
        self.ui.commandTerminated.add(term_handler)
        _handlers.append(term_handler)
        self.term_handler = term_handler

        adsk.autoTerminate(False)
        self.pump()

    def finish(self):
        if self._finished:
            return
        self._finished = True
        self.logger.log('=== KONIEC: zapisano {}, anulowano {}, pominieto {}, '
                        'bledy {} ==='.format(self.n_ok, self.n_cancel, self.n_skip, self.n_fail))

        try:
            self.ui.commandTerminated.remove(self.term_handler)
        except Exception:
            pass
        try:
            self.app.unregisterCustomEvent(NEXT_EVENT_ID)
        except Exception:
            pass

        summary = ('Zakonczono.\n\n'
                   'Zapisane DXF: {}\n'
                   'Anulowane (Anuluj w oknie): {}\n'
                   'Pominiete (nie-blacha): {}\n'
                   'Bledy: {}\n\n'
                   'Log: {}').format(self.n_ok, self.n_cancel, self.n_skip, self.n_fail, self.log_path)
        if self.cancelled:
            preview = '\n'.join('  - ' + nm for nm in self.cancelled[:15])
            summary += '\n\nAnulowane detale:\n' + preview
        try:
            self.ui.messageBox(summary)
        except Exception:
            pass
        self.logger.close()
        adsk.terminate()

    # -- glowna petla --------------------------------------------------------
    def pump(self):
        """Rozlicza poprzednie okno konwersji i przetwarza kolejne detale az do
        napotkania detalu wymagajacego okna dialogowego (wtedy czeka) lub konca."""
        if self.pending is not None:
            self._resolve_pending()
            self.pending = None
            self.idx += 1

        while self.idx < len(self.queue):
            item = self.queue[self.idx]
            comp = item['comp']
            count = item['count']
            occ = item.get('occ')
            self.logger.log('--- Komponent: {} (wystapien: {}) ---'.format(comp.name, count))

            body = largest_solid_body(comp)
            if body is None:
                self.logger.log('[POMIN] {}: brak bryl typu solid.'.format(comp.name))
                self.n_skip += 1
                self.idx += 1
                continue

            if body_is_sheet_metal(body):
                self._unfold_and_export(comp, body, count)
                self.idx += 1
                continue

            plate = detect_flat_plate(body)
            if plate is not None:
                self._export_plate(comp, plate, count)
                self.idx += 1
                continue

            cand_mm = looks_like_sheet_candidate(body)
            if cand_mm is not None:
                if self._open_convert_dialog(comp, occ, body, cand_mm, count):
                    return  # czekamy na zamkniecie okna (commandTerminated)
                self.idx += 1
                continue

            self.logger.log('[POMIN] {}: nie jest blacha ani plaska plyta '
                            '(np. walek, tuleja, odlew, zlaczka, element lity).'.format(comp.name))
            self.n_skip += 1
            self.idx += 1

        self.finish()

    def _resolve_pending(self):
        # Wywolywane z kolejki zdarzen (po tiku), gdy model po konwersji jest juz
        # przeliczony - dlatego dopiero tu weryfikujemy isSheetMetal.
        kind = self.pending[0]
        comp = self.pending[1]
        if kind == 'check':
            count = self.pending[2]
            body = largest_solid_body(comp)
            if body is not None and body_is_sheet_metal(body):
                self.logger.log('[OK] {}: skonwertowano na blache (grubosc wykryta przez Fusion).'.format(comp.name))
                self._unfold_and_export(comp, body, count)
            else:
                self.logger.log('[POMIN] {}: po zamknieciu okna bryla nie jest blacha '
                                '(konwersja nieudana lub niemozliwa).'.format(comp.name))
                self.n_skip += 1
        elif kind == 'cancel':
            self.logger.log('[ANULOWANO] {}: wcisnieto Anuluj - pomijam kolejne kroki.'.format(comp.name))
            self.n_cancel += 1
            self.cancelled.append(comp.name)

    # -- akcje na detalu -----------------------------------------------------
    def _open_convert_dialog(self, comp, occ, body, cand_mm, count):
        """Zaznacza sciane i otwiera okno 'Convert to Sheet Metal'. Zwraca True,
        jesli okno otwarto (czekamy na jego zamkniecie)."""
        face = largest_planar_face(body)
        if face is None:
            self.logger.log('[POMIN] {}: brak plaskiej sciany bazowej.'.format(comp.name))
            self.n_skip += 1
            return False

        cmd_def = self.ui.commandDefinitions.itemById(CONVERT_CMD_ID)
        if cmd_def is None:
            self.logger.log('[BLAD] Nie znaleziono polecenia "{}". Ustaw poprawne '
                            'CONVERT_CMD_ID (patrz log ponizej / skrypt WykryjIDPolecenia). '
                            'Pomijam {}.'.format(CONVERT_CMD_ID, comp.name))
            log_command_id_candidates(self.ui, self.logger)
            self.n_fail += 1
            return False

        # Sciana z definicji komponentu nie jest zaznaczalna w aktywnym zlozeniu -
        # potrzebny jest jej odpowiednik (proxy) w kontekscie wystapienia.
        sel_face = face
        if occ is not None:
            try:
                sel_face = face.createForAssemblyContext(occ)
            except Exception:
                sel_face = face

        try:
            self.ui.activeSelections.clear()
            self.ui.activeSelections.add(sel_face)
        except Exception as e:
            self.logger.log('[BLAD] {}: nie udalo sie zaznaczyc sciany ({}).'.format(comp.name, e))
            self.n_fail += 1
            return False

        self.waiting = True
        self.waiting_comp = comp
        self.waiting_count = count
        self.logger.log('[KONWERSJA] {}: otwarto okno "Convert to Sheet Metal" '
                        '(wstepnie ~{} mm). Zatwierdz (OK) albo Anuluj.'.format(
                            comp.name, format_thickness_mm(cand_mm)))
        try:
            cmd_def.execute()
        except Exception as e:
            self.logger.log('[BLAD] {}: nie udalo sie otworzyc okna konwersji ({}).'.format(comp.name, e))
            self.waiting = False
            self.waiting_comp = None
            self.n_fail += 1
            return False
        return True

    def _unfold_and_export(self, comp, body, count):
        try:
            res = export_flat_pattern_dxf(self.design, comp, body, count, self.out_folder, self.logger)
        except Exception as e:
            self.logger.log('[BLAD] {}: rozwiniecie/eksport nieudane ({}).'.format(comp.name, e))
            self.n_fail += 1
            return
        if res == 'ok':
            self.n_ok += 1
        elif res == 'fail':
            self.n_fail += 1
        else:
            self.n_skip += 1

    def _export_plate(self, comp, plate, count):
        face, thickness_cm = plate
        thickness_mm = round(thickness_cm * 10.0, 1)
        filename = build_filename(comp.name, thickness_mm, count)
        filepath = os.path.join(self.out_folder, filename)
        try:
            ok = export_face_as_dxf(comp, face, filepath, self.logger)
        except Exception as e:
            self.logger.log('[BLAD] {}: eksport obrysu plyty nieudany ({}).'.format(comp.name, e))
            self.n_fail += 1
            return
        if ok:
            self.logger.log('[DXF] {}: zapisano "{}" (plaska plyta, grubosc {} mm, {} szt.)'.format(
                comp.name, filename, format_thickness_mm(thickness_mm), count))
            self.n_ok += 1
        else:
            self.logger.log('[BLAD] {}: saveAsDXF zwrocilo False.'.format(comp.name))
            self.n_fail += 1

    # -- zdarzenia -----------------------------------------------------------
    def on_command_terminated(self, args):
        if not self.waiting:
            return
        try:
            if args.commandId != CONVERT_CMD_ID:
                return
        except Exception:
            return

        comp = self.waiting_comp
        count = self.waiting_count
        self.waiting = False
        self.waiting_comp = None
        try:
            self.ui.activeSelections.clear()
        except Exception:
            pass

        try:
            completed = (args.terminationReason ==
                         adsk.core.CommandTerminationReason.CompletedTerminationReason)
        except Exception:
            completed = False

        # Tylko klasyfikujemy OK vs Anuluj. Weryfikacja isSheetMetal i rozwiniecie
        # nastepuja w _resolve_pending (po tiku), gdy model jest juz przeliczony.
        if completed:
            self.pending = ('check', comp, count)
        else:
            self.pending = ('cancel', comp)

        # Przejscie do kolejnego detalu poza kontekstem obslugi polecenia.
        try:
            self.app.fireCustomEvent(NEXT_EVENT_ID)
        except Exception:
            self.pump()


class _CommandTerminatedHandler(adsk.core.ApplicationCommandEventHandler):
    def __init__(self, wizard):
        super(_CommandTerminatedHandler, self).__init__()
        self.wizard = wizard

    def notify(self, args):
        try:
            self.wizard.on_command_terminated(args)
        except Exception:
            try:
                self.wizard.logger.log('Blad w commandTerminated:\n' + traceback.format_exc())
            except Exception:
                pass


class _NextPartHandler(adsk.core.CustomEventHandler):
    def __init__(self, wizard):
        super(_NextPartHandler, self).__init__()
        self.wizard = wizard

    def notify(self, args):
        try:
            self.wizard.pump()
        except Exception:
            try:
                self.wizard.logger.log('Blad w pump:\n' + traceback.format_exc())
            except Exception:
                pass


# ----------------------------------------------------------------------------
# Punkt wejscia
# ----------------------------------------------------------------------------
def run(context):
    global _wizard
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

        logger.log('=== START: rozwijanie blach (kreator pol-automatyczny) ===')
        try:
            doc_name = design.parentDocument.name
        except Exception:
            doc_name = '(nieznany)'
        logger.log('Projekt: {}'.format(doc_name))
        logger.log('Folder docelowy: {}'.format(out_folder))

        if design.designType == adsk.fusion.DesignTypes.DirectDesignType:
            logger.log('[UWAGA] Projekt w trybie bezposrednim (Direct). Wzory plaskie '
                       'wymagaja trybu parametrycznego (wlacz historie projektu).')

        components = gather_unique_components(root_comp, logger)
        queue = [{'comp': d['comp'], 'count': d['count']} for d in components.values()]
        if not queue:
            logger.log('Brak komponentow do przetworzenia.')
            ui.messageBox('Brak komponentow do przetworzenia.')
            logger.close()
            return

        del _handlers[:]
        _wizard = Wizard(app, ui, design, out_folder, logger, queue, log_path)
        _wizard.start()
        # run() konczy sie tutaj; kreator dziala dalej dzieki autoTerminate(False)
        # i zostanie zamkniety przez adsk.terminate() w Wizard.finish().

    except Exception:
        msg = 'Blad wykonania skryptu:\n{}'.format(traceback.format_exc())
        if logger:
            logger.log(msg)
        if ui:
            ui.messageBox(msg)
