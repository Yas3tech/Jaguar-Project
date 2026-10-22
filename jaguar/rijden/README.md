# rijden/ — de rijscripts

| Script | Wat het doet |
|---|---|
| `rechtdoor.py` | Rechtdoor rijden: `python -m jaguar.rijden.rechtdoor --power 250 --distance 2` |
| `heen_en_terug.py` | Heen en terug rijden: `python -m jaguar.rijden.heen_en_terug --distance 1.5` |
| `parcours.py` | Een reeks stappen (vooruit, achteruit, links, rechts): `python -m jaguar.rijden.parcours v:2 l:90 v:2` |
| `bediening.py` | Hetzelfde vanuit de browser, met een knop Rijden en STOP: `python -m jaguar.rijden.bediening` |

Elke rit slaat een CSV op in `ritten/`. Alle scripts hebben `--no-csv`, `--no-live`, `--ip` en `--mqtt`. Met `--ip 127.0.0.1` rijd je tegen de nep-robot.
