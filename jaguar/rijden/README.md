# rijden/ — de rijscripts

| Script | Wat het doet |
|---|---|
| `rechtdoor.py` | Rechtdoor rijden: `python -m jaguar.rijden.rechtdoor --power 250 --distance 2` |
| `heen_en_terug.py` | Heen en terug rijden: `python -m jaguar.rijden.heen_en_terug --distance 1.5` |
| `parcours.py` | Een reeks stappen (vooruit, achteruit, links, rechts): `python -m jaguar.rijden.parcours v:2 l:90 v:2` |
| `bediening.py` | Hetzelfde vanuit de browser, met een knop Rijden (een ontwerper met raster of kaart, bogen en simulatie), Pauze, Terug naar start, Handmatig en STOP: `python -m jaguar.rijden.bediening` |

Elke rit slaat een CSV op in `ritten/`. Alle scripts hebben `--no-csv`, `--no-live`, `--ip` en `--mqtt`. Met `--ip 127.0.0.1` rijd je tegen de nep-robot of de gesimuleerde robot (`python -m jaguar.testen.sim_robot`).

## Bediening vanuit de browser

- **Pauze** (knop of P): het robot stopt waar het is. **Hervatten** rijdt dezelfde stap verder naar hetzelfde geplande hoekpunt of dezelfde koers (een draai of boog: de rest ervan). Pauzeren mag ook vanaf een ander apparaat dat meekijkt, zoals STOP; hervatten alleen op de laptop. In de analyse telt een stap met een pauze als één stap.
- **Terug naar start**: tijdens een rit breekt dit het parcours af; het robot draait naar de start, rijdt er in een rechte lijn heen (ligt de start achter hem en dichter dan 3 m: achteruit) en draait naar de beginrichting (`naar_start` in `parcours.py`). Na de rit kan het ook nog, met de laatste positie volgens de odometrie: dan mag niemand het robot intussen verplaatst hebben. De fasen heten `terug-1-draai`, `terug-2-vooruit`, …; een aparte terugrit krijgt een CSV `_terug`. Het robot kijkt niet of de rechte lijn vrij is (alleen de camera vooruit, als Obstakels aan staat).
- **Batterij**: de spanning springt ±0,5 V tussen twee metingen, dus `robot.py` kijkt naar de mediaan over 10 s. Beginnen kan vanaf 22,2 V. Zakt ze tijdens een parcours onder 21,6 V (3,6 V per cel), dan breekt het parcours af en rijdt het robot vanzelf terug naar de start; onder 21,0 V (3,5 V per cel) stopt het helemaal, ook op de terugweg. Handmatig rijden stopt onder 21,6 V; Terug naar start kan dan nog (tot 21,0 V). Dat geldt voor alle rijscripts: zonder bediening.py stopt de rit onder 21,6 V.
- **Handmatig**: rijden met de pijltjes of WASD, de knoppen op het scherm (ingedrukt houden) of een gamepad (linkerstick, B = STOP), met een schuif voor het vermogen. De pagina stuurt 10 keer per seconde de stand; komt er 0,4 s niets (tabblad dicht, wifi weg), dan staat het robot stil. Rechtdoor houdt de gyro de koers vast. De veiligheidsstops blijven gelden (kantelen, vastgelopen, obstakel vooruit); daarna eerst alles loslaten. Alleen op de laptop. De rit krijgt een CSV `_handmatig` en de analyse werkt zoals altijd; daarna brengt Terug naar start het robot naar waar het handmatig rijden begon.
- `gps_route.py`: een route rijden die uit GPS-coördinaten bestaat (`--richting` of `--kalibreer-noord`, `--droog` om alleen het plan te tonen). Zie [navigatie/](../navigatie/README.md).
