# testen/ — zonder robot oefenen

- `nep_robot.py`: speelt een opgenomen rit (`testdata/demo_rit.csv`) af alsof het de robot is.

```
python -m jaguar.testen.nep_robot
python -m jaguar.rijden.rechtdoor --ip 127.0.0.1 --duration 5 --no-csv
```
De nep-robot speelt altijd dezelfde opname af, dus rijden op afstand reageert niet echt.

- `sim_robot.py`: een gesimuleerde robot die wel op de motoren reageert (vertraging, wrijving, slip bij draaien, watchdog en E-Stop zoals de echte). Hij stuurt ook de GPS: `$GPRMC` met 5 Hz, zoals de Garmin van het robot. Zo kun je rijden, pauzeren, terug naar de start, handmatig rijden en GPS-routes proberen. Leerwaarden (`config/leerwaarden.json`) worden met 127.0.0.1 niet gelezen of bewaard.

```
python -m jaguar.testen.sim_robot
python -m jaguar.rijden.bediening --ip 127.0.0.1
python -m jaguar.testen.sim_robot --batterij 22.4 --verbruik 3                 # batterij die snel leegloopt
python -m jaguar.testen.sim_robot --richting 90 --gps-ruis 1.5 --gps-dwaal 0.3 --gps-vertraging 0.6   # GPS zoals buiten
python -m jaguar.testen.sim_robot --slip 0.05 --geen-gps                        # 5 % minder ver dan de wielen tellen, zonder GPS
```

| Optie | Wat het doet |
|---|---|
| `--lat`, `--lon` | waar het robot begint (standaard 50,8419 en 4,3229) |
| `--richting` | kompaskoers bij de start (0 = noord, 90 = oost) |
| `--geen-gps` | geen `$GPRMC` sturen (zoals binnen zonder antenne) |
| `--gps-ruis` | ruis van de GPS-positie in m (standaard 0,02; echte GPS: 1 tot 3) |
| `--gps-dwaal` | langzaam dwalen van de GPS in m per wortel-seconde (echte GPS buiten: ca. 0,2) |
| `--gps-vertraging` | s: de GPS meldt waar het robot zoveel seconden geleden was (echte GPS: tot 1 s) |
| `--slip` | deel van de wielweg dat de grond niet volgt (0,05 = 5 % minder ver). De odometrie ziet dat niet; de GPS wel |
| `--snelheid` | echte snelheid t.o.v. wat het vermogen voorspelt (standaard 0,9) |
| `--vertraging` | s: hoe traag de motoren het vermogen volgen (standaard 0,25) |
| `--watchdog` | s zonder PING voor de motoren stoppen (standaard 0,3) |
| `--batterij`, `--verbruik`, `--port` | spanning bij de start, V per minuut rijden, poort (standaard 10001) |

De GPS staat standaard bijna foutloos (ruis 2 cm), zoals in een simulator. Met `--gps-ruis` en `--gps-dwaal` lijkt hij op de echte.

- `test_rijden.py`: controles voor het rijden zelf, tegen `sim_robot` (`python -m jaguar.testen.test_rijden`, ongeveer 4 minuten; `... test_rijden vijf` draait alleen de tests met "vijf" in de naam). Bewezen wordt, in de wereld van de simulatie:
  - 5 m rijden gebeurt in één beweging: geen centimeter achteruit, één stop, op het doel;
  - dat blijft zo met tragere motoren, een andere snelheid en meer vermogen (de uitloop wordt bijgeleerd);
  - heen en terug, en een vierkant met draaien: het robot eindigt op de start;
  - alleen MM0 meldt encoders: het robot rijdt toch;
  - met `--precies` werkt de oude manier (kruipstoten) nog.
- `test_gps_route.py`: controles voor de GPS-routes, tegen `sim_robot` met GPS (`python -m jaguar.testen.test_gps_route`, ongeveer 6 minuten; `... test_gps_route slip` alleen de tests met "slip"). Bewezen wordt:
  - een route met `--richting` komt aan op het laatste punt;
  - het noorden wordt gevonden door een stukje te rijden (ook als het robot scheef staat), en `--vanaf-hier` werkt;
  - met slip ziet de GPS dat: de schaal klopt, en met GPS-correctie komt het robot veel dichter bij het doel;
  - een onrustige GPS (meters ruis, dwalen) maakt de rit niet slechter dan rijden op de wielen;
  - een GPS die achterloopt: de vertraging wordt gevonden;
  - zonder GPS rijdt een route met `--richting` gewoon, en vraagt het zoeken van het noorden om `--richting`;
  - de afstandsmeting met de GPS (`gps_afstand`) klopt bij 5 % slip.
- `test_navigatie.py`: controles voor de GPS-omrekening, het noorden en het anker, zonder robot (`python -m jaguar.testen.test_navigatie`).
- `test_bediening.py`: bediening.py tegen de gesimuleerde robot: pauze, terug naar start (tijdens en na de rit), lege batterij, handmatig rijden met de dodemansknop en STOP tijdens een pauze (`python -m jaguar.testen.test_bediening`, ongeveer twee minuten, geen CSV's). Gebruikt een vrije poort, dus niet 10001.
- `sim_boog.py`: simulatie van een skid-steer robot om de boogregeling (`Jaguar.arc`) te controleren (`python -m jaguar.testen.sim_boog`).

## De websimulator (robotics.dinno.be)

Op robotics.dinno.be staat ook een simulator van de Jaguar 4x4 Wheel. Die is een andere dan `sim_robot.py` en heeft een eigen brug:

1. Open de Jaguar 4x4 Wheel simulator op robotics.dinno.be.
2. Start in de map van de repository robotSimulatorAPI de brug (eerst eenmalig `pip install websockets`):
   ```
   python jaguar_bridge.py
   ```
3. Klik in de kaart **Real Jaguar** op **Connect**. De gesimuleerde Jaguar staat dan op 127.0.0.1:10001.
4. Alle scripts hier werken dan met `--ip 127.0.0.1`:
   ```
   python -m jaguar.rijden.rechtdoor --ip 127.0.0.1 --distance 5
   python -m jaguar.rijden.parcours --ip 127.0.0.1 vooruit:5 rechts:90 vooruit:2
   python -m jaguar.rijden.bediening --ip 127.0.0.1
   python -m jaguar.rijden.gps_route --ip 127.0.0.1 --vanaf-hier 50.84010,4.32000
   ```

Start `sim_robot.py` niet tegelijk: beide gebruiken poort 10001. De IMU-parser accepteert de extra COMP-velden en andere veldnamen. Meldt alleen MM0 encoders, dan tellen die waarden voor beide drivers.

De websimulator rekent met een wielstraal van 0,135 m; dit project met de straal die op het echte robot gemeten is (`config/kalibratie.json`, 0,1336 m). Zet daarom voor de websimulator eerst de straal goed, anders rijdt het robot daar 1 % te ver (PowerShell: `$env:JAGUAR_WIELSTRAAL = "0.135"`; bash: `export JAGUAR_WIELSTRAAL=0.135`). Start de brug met `--no-robot` als het echte robot niet aan staat. De eerste keer vraagt Chrome of de pagina het lokale netwerk mag gebruiken: sta dat toe, anders blijft Connect op "Connecting…" staan.

Wat de websimulator stuurt, is vergeleken met wat de code leest (documentatie en broncode van de pagina): dezelfde IMU-regel (17 velden, met COMP), encoders van beide drivers met 5 Hz, `V=` en `FF=`, `$GPRMC` (en `$GPGGA`, die wordt overgeslagen), watchdog 0,3 s. De GPS heeft daar standaard een fout van ongeveer 1 m die langzaam dwaalt (vakje "GPS error") en een resolutie van 18 cm: `gps_route` behandelt hem als een gewone GPS.

**Eerlijk over de status:** er is **nog geen rit gereden tegen de websimulator zelf** (de testbrowser mocht van Chrome niet met de brug verbinden) en **niet op de echte robot**. Alles is alleen gecontroleerd tegen `sim_robot.py`, en dat is een model.
