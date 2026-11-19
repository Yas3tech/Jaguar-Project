# Jaguar-Project

Software voor de Dr Robot Jaguar 4x4: rijden op afstand, op centimeters, een route volgen naar GPS-coördinaten, obstakels zien met de camera, live meekijken en elke rit achteraf analyseren.

**Demo:** [video van de simulatie](docs/demo/robot_video_simulatie.mp4)

## Wat je nodig hebt

| Wat | Detail |
|---|---|
| Robot | Jaguar 4x4 op het wifi `DriJaguar`, besturing op `192.168.0.60:10001` (TCP) |
| Camera | Axis op het robot, `192.168.0.65:8081` (alleen nodig voor camerabeeld en obstakels) |
| Laptop | Python 3.11, `pip install matplotlib` (analyse), `pip install opencv-python numpy` (obstakels) |
| Optioneel | Docker voor Grafana (`grafana/`) |

Alle commando's draai je vanuit de projectmap: `python -m jaguar.<map>.<bestand>`.

## Hoe het geheel werkt

```
robot ──TCP-tekstregels──> kern/robot.py ──> odometrie.py (waar ben ik?)
                               │                   │
                               │                   └──> rijscripts sturen de motoren (rijden/)
                               ├──> live pagina http://127.0.0.1:8765  (+ MQTT → Grafana)
                               └──> ritten/rit_<datum>.csv ──> analyse/rit.py ──> dashboard/dashboard.html
```

De robot zelf rekent niets uit: hij stuurt ruwe metingen en voert motoropdrachten uit. Alles (positie, sturen, veiligheid) gebeurt op de laptop.

## Wat de robot stuurt

| Regel | Hoe vaak | Inhoud |
|---|---|---|
| `#..,GYRO,..,ACC,..` | 50 Hz | IMU: draaisnelheid (gyro) en versnelling |
| `MM0 C=..` / `MM1 C=..` | 5 Hz | tellers van de 4 wielen (1 puls = 2,8 mm) |
| `MM0 V=..` | 5 Hz | batterijspanning |
| `$GPRMC,..` | 5 Hz | GPS: breedte, lengte, snelheid en koers over de grond |
| `MM0 FF=..` | 1 Hz | E-Stop actief of niet |

De laptop stuurt elke 100 ms een PING en het vermogen per kant (`MMW !M <links> <-rechts>`, -1000..1000). Komt er geen PING meer, dan stopt het robot vanzelf (watchdog).

## Waar is het robot? (odometrie)

[kern/odometrie.py](jaguar/kern/odometrie.py), live tijdens het rijden én achteraf in de analyse:

- **Afstand:** gemiddelde van de 4 wielen. Een wiel dat afwijkt van de rest, of van wat de accelerometer voorspelt, slipt en telt niet.
- **Koers:** de gyro. Het kompas is onbruikbaar door de motoren.
- **Alle wielen slippen:** de accelerometer overbrugt hoogstens 0,6 s; daarna is de afstand "onzeker" en stopt het robot na 1 s.
- **Grens:** slippen alle vier de wielen evenveel (gras, zand), dan ziet niemand het.

## Rijden

| Script | Wat het doet |
|---|---|
| `rijden.rechtdoor --distance 2` | rechtdoor in één beweging: optrekken, remmen, vermogen uit zodat het uitrollen precies op het doel eindigt (fout ≤ 10 mm) |
| `rijden.heen_en_terug --distance 1.5` | heen en terug |
| `rijden.parcours v:2 l:90 v:2` | stappen vooruit/achteruit en draaien ter plaatse op de gyro (tot op 1°) |
| `rijden.gps_route <lat,lon> ...` | een route van GPS-punten (zie hieronder) |
| `rijden.bediening` | alles vanuit de browser (zie hieronder) |

- **Rechtdoor blijven:** de afwijking van de lijn geeft een gewenste koers, een PID op de gyro zet die om in een verschil links/rechts.
- **Een bocht of route in één beweging** (`Jaguar.volg`): elke 0,05 s mikt het robot op de richting van het pad 0,15 s verderop, plus 100° per meter naast de lijn (hoogstens 25°), en remt voor bochten.
- **Bijleren:** uitrollen, vermogen→snelheid en draaistoten worden na elke rit bewaard in `config/leerwaarden.json`.

## Naar GPS-coördinaten

```powershell
python -m jaguar.rijden.gps_route --punten-bestand testdata/websim_epsilon_baan.txt --richting 0 --vloeiend
python -m jaguar.rijden.gps_route 50.8400000,4.3200000 50.8400449,4.3200000 --richting 0 --droog   # alleen het plan
```

1. **Punt 1 = nulpunt.** Elke coördinaat wordt (oost, noord) in meter rond punt 1 ([navigatie/gps.py](jaguar/navigatie/gps.py), `Lokaal`).
2. **Het noorden:** `--richting` (0 = noord, 90 = oost), of het robot rijdt eerst 6 m rechtdoor en haalt het noorden uit het GPS-spoor.
3. **Draaien naar het robotstelsel:** (oost, noord) wordt (vooruit, links), de hoeken worden afgerond, een punt om de 2 cm.
4. **Rijden op wielen en gyro.** De GPS (3 tot 15 m fout) controleert het noorden en de schaal (slip), en verbetert de positie pas als hij beter is dan de wielen.

### Hoe weten we waar het robot is aangekomen?

Aan de robot of zijn API moest niets veranderen: de GPS-ontvanger stuurt zelf al `$GPRMC`-zinnen, en `navigatie/gps.py` leest ze. Na de rit geeft `gps_route` twee posities:

- **Volgens wielen en gyro:** de positie van de odometrie, met het noorden en punt 1 terug omgerekend naar breedte/lengte (`robot_naar_enu` + `Lokaal.naar_ll`). Dit is de nauwkeurigste.
- **Volgens de GPS:** het gemiddelde van de laatste 2 s GPRMC, met de afstand tot het laatste punt.

In de websimulator meldt de brug (`testen/api_brug.py`) bij het sluiten ook waar het robot **echt** staat in de simulatie.

## Bediening vanuit de browser

`python -m jaguar.rijden.bediening` opent http://127.0.0.1:8765:

- **Rijden:** ontwerper met een raster (Binnen) of een kaart (Buiten). Je tekent de route, bekijkt de simulatie en start.
- **Pauze** (of P), **Terug naar start**, **STOP** (of spatie).
- **Handmatig:** pijltjes/WASD of een gamepad. Ontvangt het robot 0,4 s niets, dan stopt het.
- **Analyse:** het dashboard van de rit die net gereden is.

Andere apparaten op het wifi kunnen meekijken en STOP drukken; starten kan alleen op de laptop.

## Camera en obstakels

[camera/obstakel.py](jaguar/camera/obstakel.py), standaard uit (aan via Opties → Camera / Obstakels):

- **Zien:** in de middelste strook van het beeld zoekt OpenCV van onder naar boven de eerste plotse kleursprong (de voet van een obstakel).
- **Afstand:** de camera staat vast, dus elke beeldlijn is een afstand op de vloer (`d = k / (y − y_horizon)`), eenmalig gekalibreerd tegen een muur op 1, 2 en 3 m.
- **Reageren:** vooruit afremmen en stoppen op 0,4 m. Een parcours wacht tot 4 s en rijdt er dan omheen. Geen beeld gedurende 1,5 s = stoppen.
- **Grens:** alleen vooruit; glas, een muur in de kleur van de vloer of duisternis ziet hij niet.

## Veiligheid

Het robot stopt vanzelf bij:

- meer dan 35° kantelen;
- 1,5 s vol vermogen zonder vooruit te komen (vastgelopen);
- wielen die 1 s draaien zonder dat het robot rijdt;
- meer dan 1 m naast de route;
- batterij: onder 21,6 V terug naar de start, onder 21,0 V stoppen.

De fysieke noodstop blijft de echte noodstop.

## Na de rit: analyse en dashboard

- Elke rit wordt een CSV in `ritten/` (tijd, fase, vermogen, ruwe regel).
- `python -m jaguar.analyse.rit` rekent alles opnieuw uit en werkt `dashboard/dashboard.html` bij: afstand, slip, gebeurtenissen, de route op ware schaal en een 3D-weergave.
- Live in Grafana: voeg `--mqtt` toe (opzetten: [grafana/](grafana/README.md)).

## Kalibreren

| Wat | Commando |
|---|---|
| Wielstraal (rolmeter) | `python -m jaguar.kalibratie.kalibreer 2.968` na 3 m rijden |
| Afstand met de GPS | `python -m jaguar.kalibratie.gps_afstand --distance 20` |
| Gyroschaal | `python -m jaguar.kalibratie.test_sondewiel draai --turns 2` |
| Camera | Opties → Obstakels → Kalibreren… |

## Zonder robot

| Wat | Commando |
|---|---|
| Gesimuleerde robot (met GPS, slip, vertraging) | `python -m jaguar.testen.sim_robot`, dan rijden met `--ip 127.0.0.1` |
| Websimulator robotics.dinno.be | `.\start_websim.ps1` (zie [testen/](jaguar/testen/README.md)) |
| Tests | `python -m jaguar.testen.test_rijden` (~4 min), `test_gps_route` (~6 min), `test_bediening` (~2 min) |

## Resultaten en wat nog niet getest is

| Waar | Gemiddeld naast de lijn | Hoogstens |
|---|---|---|
| sim_robot, epsilon-baan (echte positie) | 3,9 cm | 11,6 cm |
| websimulator, epsilon-baan (volgens het robot) | 8 cm | 27 cm |

- **Op het echte robot gemeten:** alleen `--precies` (1 en 5 m binnen 1 à 3 mm volgens de encoders).
- **Niet op het echte robot getest:** routes in één beweging, bogen, GPS-routes, de camera. Eerst kort, traag en met de noodstop bij de hand.

## Mappen

```
jaguar/kern/        robot.py (verbinding, motoren, veiligheid) en odometrie.py
jaguar/rijden/      rechtdoor, heen_en_terug, parcours, gps_route, bediening
jaguar/navigatie/   GPS lezen, coördinaten ↔ meters, het noorden, routes
jaguar/camera/      obstakels zien
jaguar/live/        live pagina en ontwerper
jaguar/analyse/     rit.py: analyse en dashboard
jaguar/kalibratie/  wielstraal, gyroschaal, driver
jaguar/testen/      nep- en simulatierobot, brug naar de websimulator, tests
ritten/  dashboard/  config/  grafana/  testdata/  docs/
```

Elke map heeft een eigen README. De vroegere uitgebreide uitleg staat in [docs/uitgebreid.md](docs/uitgebreid.md).
