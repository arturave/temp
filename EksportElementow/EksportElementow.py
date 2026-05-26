# -*- coding: utf-8 -*-
"""
Rozklada zlozenie na poszczegolne elementy i zapisuje kazdy jako osobny plik STEP
(Fusion 360). Bez rozwijania blach.

Dziala na aktywnym projekcie (Design). Skrypt:
  1. Loguje dzialania do pliku .log w folderze docelowym oraz do palety
     "Text Commands" Fusion.
  2. Analizuje drzewo zlozenia i identyfikuje niepowtarzalne komponenty oraz
     liczbe ich wystapien (laczna ilosc sztuk w calym zlozeniu).
  3. Eksportuje kazdy unikalny komponent posiadajacy geometrie (bryly) do osobnego
     pliku STEP w geometrii wlasnej komponentu.
  4. Nazwa pliku wg maski:  NazwaKomponentu_ZZZszt.step
     (ilosc uzupelniona do 3 cyfr, np. 005szt).

Komponenty bez wlasnych bryl (czyste podzespoly) sa pomijane - reprezentuja je
ich elementy skladowe, ktore i tak sa eksportowane osobno.
"""

import adsk.core
import adsk.fusion
import traceback
import os
import re
import datetime


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


def sanitize_filename(name):
    """Usuwa znaki niedozwolone w nazwach plikow."""
    return re.sub(r'[<>:"/\\|?*\r\n\t]', '_', name).strip()


def build_filename(comp_name, count):
    """Nazwa pliku wg maski NazwaKomponentu_ZZZszt.step."""
    return '{}_{:03d}szt.step'.format(sanitize_filename(comp_name), count)


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

    # Projekt jednoczesciowy (brak wystapien) - rozpatrz komponent glowny.
    if not result and root_comp.bRepBodies.count > 0:
        result[root_comp.name] = {'comp': root_comp, 'count': 1}

    logger.log('Wystapien lacznie: {}, niepowtarzalnych komponentow: {}'.format(
        occurrences.count, len(result)))
    return result


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


def export_component_step(export_mgr, comp, count, out_folder, logger):
    """Eksportuje pojedynczy komponent do pliku STEP. Zwraca 'ok'|'skip'|'fail'."""
    name = comp.name

    if comp.bRepBodies.count == 0:
        logger.log('[POMIN] {}: brak wlasnych bryl (podzespol) - pomijam.'.format(name))
        return 'skip'

    filename = build_filename(name, count)
    filepath = unique_path(out_folder, filename)

    try:
        step_opts = export_mgr.createSTEPExportOptions(filepath, comp)
        ok = export_mgr.execute(step_opts)
    except Exception as e:
        logger.log('[BLAD] {}: eksport STEP nieudany ({}).'.format(name, e))
        return 'fail'

    if ok:
        logger.log('[STEP] {}: zapisano "{}" ({} szt.)'.format(
            name, os.path.basename(filepath), count))
        return 'ok'
    logger.log('[BLAD] {}: execute() zwrocilo False.'.format(name))
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

        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = 'Wybierz folder docelowy dla plikow STEP i logu'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        out_folder = folder_dlg.folder

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(out_folder, 'eksport_elementow_{}.log'.format(stamp))
        logger = Logger(log_path, app)

        logger.log('=== START: eksport elementow zlozenia do STEP ===')
        try:
            doc_name = design.parentDocument.name
        except Exception:
            doc_name = '(nieznany)'
        logger.log('Projekt: {}'.format(doc_name))
        logger.log('Folder docelowy: {}'.format(out_folder))

        components = gather_unique_components(root_comp, logger)
        export_mgr = design.exportManager

        n_ok = n_skip = n_fail = 0
        for data in components.values():
            comp = data['comp']
            count = data['count']
            logger.log('--- Komponent: {} (wystapien: {}) ---'.format(comp.name, count))
            res = export_component_step(export_mgr, comp, count, out_folder, logger)
            if res == 'ok':
                n_ok += 1
            elif res == 'skip':
                n_skip += 1
            else:
                n_fail += 1

        logger.log('=== KONIEC: zapisano {}, pominieto {}, bledy {} ==='.format(
            n_ok, n_skip, n_fail))

        ui.messageBox(
            'Zakonczono.\n\n'
            'Zapisane pliki STEP: {}\n'
            'Pominiete (podzespoly bez bryl): {}\n'
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
