# -*- coding: utf-8 -*-
"""
Pomocniczy skrypt Fusion 360: wykrywa wewnetrzne ID polecenia, ktore uruchomisz.

Sluzy do ustalenia ID polecenia "Konwertuj na konstrukcje blachowa", ktore nalezy
wpisac do stalej CONVERT_CMD_ID w glownym skrypcie RozwijanieBlach.

Sposob uzycia:
  1. Uruchom ten skrypt.
  2. W oknie instrukcji kliknij OK.
  3. W Fusion kliknij przycisk "Konwertuj na konstrukcje blachowa"
     (zakladka Konstrukcja blachowa > Utworz > Konwertuj na konstrukcje blachowa).
  4. Skrypt wyswietli ID polecenia - skopiuj je do CONVERT_CMD_ID.
  5. Okno konwersji mozesz zamknac przyciskiem Anuluj.

Polecenia pomocnicze (np. SelectCommand) sa pomijane; lapane jest pierwsze
"prawdziwe" polecenie. Pelna sekwencja jest tez wypisywana w palecie Text Commands.
"""

import adsk.core
import traceback

_app = None
_ui = None
_handler = None
_handlers = []

# Polecenia ignorowane jako "szum" przy wykrywaniu.
_IGNORE = {'', 'SelectCommand'}


class _CommandStartingHandler(adsk.core.ApplicationCommandEventHandler):
    def notify(self, args):
        try:
            cid = args.commandId
            _app.log('commandStarting: ' + cid)
            if cid in _IGNORE:
                return
            try:
                _ui.commandStarting.remove(_handler)
            except Exception:
                pass
            _ui.messageBox(
                'ID uruchomionego polecenia:\n\n    {}\n\n'
                'Wpisz je do CONVERT_CMD_ID w glownym skrypcie RozwijanieBlach.\n'
                'Okno konwersji mozesz teraz zamknac przyciskiem Anuluj.'.format(cid))
            adsk.terminate()
        except Exception:
            if _ui:
                _ui.messageBox('Blad:\n' + traceback.format_exc())


def run(context):
    global _app, _ui, _handler
    _app = adsk.core.Application.get()
    _ui = _app.userInterface
    try:
        _handler = _CommandStartingHandler()
        _ui.commandStarting.add(_handler)
        _handlers.append(_handler)
        adsk.autoTerminate(False)
        _ui.messageBox(
            'Za chwile kliknij w Fusion przycisk '
            '"Konwertuj na konstrukcje blachowa".\n\n'
            'Skrypt zlapie ID tego polecenia i je wyswietli.\n'
            '(Polecenia pomocnicze typu zaznaczanie sa pomijane.)')
    except Exception:
        if _ui:
            _ui.messageBox('Blad:\n' + traceback.format_exc())
