# Jaguar-Project

## Rijden en opslaan

Beide scripts slaan standaard een CSV op in `ritten`. Gebruik `--no-csv` om dit uit te schakelen; `--csv` om het expliciet aan te zetten.

```powershell
python fase1_rechtdoor_imu.py --power 250 --distance 2.0
python fase1_heen_en_terug.py --power 250 --distance 1.5 --pause 1.5
python fase1_heen_en_terug.py --power 150 --duration 2 --ramp 2      # oude manier: op tijd
python fase1_rechtdoor_imu.py --power 150 --distance 1 --no-csv
python analyse_rit.py
python analyse_rit.py ritten/rit_20261002_100122.csv
```

`--distance` (standaard) is de afstand in meter volgens de encoders, voor heen/terug **per richting**. De robot bouwt het vermogen op over `--ramp` seconden (standaard 1,5), remt af naarmate het doel nadert, stopt 2 cm ervoor en legt de laatste millimeters af met korte kruipstoten (`--creep`, standaard 60). Na elke stoot meet hij in stilstand opnieuw, tot de fout binnen `--tolerance` mm ligt (standaard 3). Na elk segment toont het script het doel, de meting en de fout.

Grenzen van die precisie: één encoderpuls is 2,8 mm wielomtrek, en de encoders komen maar met ongeveer 5 Hz binnen. De stopfout is dus gemeten **aan de wielen**: wielslip, gras of een helling verschuiven de echte positie zonder dat de encoders het zien. Als de robot bij vol vermogen 1,5 s nauwelijks vooruitgaat (vastgelopen, obstakel), breekt het script het segment af.

Richting: `--distance -3` of `--power -250` rijdt achteruit. Bij heen/terug betekent een negatief vermogen of een negatieve afstand: eerst achteruit, dan terug vooruit.

Rechtdoor houden met een PID-regelaar (cascade): tijdens de rustfase meet het script de gyrobias. Tijdens het rijden geeft de zijdelingse afwijking een gewenste koers (20° per meter, max. 5°, terug naar de startlijn). Een PID op de gyrokoers zet die om in een vermogensverschil tussen links en rechts:
- P (`KP_HEADING` = 6 per graad) reageert op de koersfout;
- I (`KI_HEADING` = 4 per graad·s) compenseert een kant die blijvend zwakker trekt; bij achteruit rijden wordt die geleerde waarde gespiegeld;
- D (`KD_HEADING` = 0,4 per graad/s, op de gemeten draaisnelheid) dempt overslingeren.

Het verschil is begrensd op 30 % van het rijvermogen (60 % bij de kruipstoten). De integrator stopt met groeien zolang de uitgang begrensd is (anti-windup). Zonder regeling draaide de robot bij het optrekken 3,6° weg en eindigde hij 17 cm naast de lijn na 3 m. Na elk segment toont het script de koers en de geschatte afstand tot de startlijn. Slingert de robot heen en weer: verlaag `KP_HEADING`. Blijft hij scheef: verhoog `KI_HEADING`.

`--duration` rijdt op tijd zoals vroeger. `--power` is een open-loop motoropdracht en het maximum tijdens het rijden, geen snelheid in m/s. Het bereik is 0..1000 voor heen/terug en -1000..1000 voor rechtdoor (negatief = achteruit).

## Afstand kalibreren

De encoders rekenen met een nominale wielstraal van 0,135 m. De echte rolstraal van de noppenbanden hangt af van de ondergrond en de bandenspanning. Kalibreren gaat zo:

```powershell
python fase1_rechtdoor_imu.py --power 250 --distance 3
# meet met de rolmeter hoe ver één vast punt van de robot verplaatst is, bv. 2,968 m
python kalibreer.py 2.968
```

Dit schrijft `kalibratie.json`. De rijscripts en de analyse gebruiken daarna die straal. Verwijder het bestand om terug te gaan naar 0,135 m.

## Live meekijken tijdens het rijden

- **Browser (standaard aan):** elk rijscript opent `http://127.0.0.1:8765/` met live tegels en grafieken (`live_view.py`, `live.html`). Uitschakelen met `--no-live`.
- **Grafana (`--mqtt`):** dezelfde live waarden gaan via MQTT naar InfluxDB, en het dashboard **Jaguar live** op http://localhost:3000 toont ze. De indeling volgt dit analysedashboard: kerncijfers, Ritverkenner (met plaats voor de camera), Pad in 3D (hoogte en gebeurtenissen), Tracking, Niveau en Trillingen. Opzetten: zie [grafana/README.md](grafana/README.md).
- **Zonder robot testen:** `python nep_robot.py` speelt `testdata/demo_rit.csv` af. Rijd ertegen met `--ip 127.0.0.1 --duration 5 --no-csv`. Rijden op afstand werkt niet met de nep-robot, omdat die een vaste opname afspeelt.

```powershell
cd grafana; docker compose up -d; cd ..
python fase1_rechtdoor_imu.py --power 150 --distance 1 --mqtt --no-live
```

De live waarden zijn een benadering. Dit dashboard (`analyse_rit.py` op de CSV) blijft de definitieve meting. De CSV verandert niet door de live weergave.

## Dashboard

Open `dashboard.html` in de projectmap en kies een rit met **Selecteer een rit**. Elke uitvoering van `python analyse_rit.py` werkt ditzelfde dashboard bij met alle `rit_*_analyse.json`-bestanden uit de map van de geanalyseerde CSV. De zojuist geanalyseerde rit is standaard geselecteerd. Ververs de browser na een nieuwe analyse. Er wordt geen apart HTML-bestand per rit meer gemaakt; bestaande HTML-rapporten blijven beschikbaar.

Het dashboard opent rechtstreeks in de browser, zonder server. Het bevat een tijdschuif, een geschatte baan met gelijke schaal op beide assen, pitch/roll-aanzichten, motoropdracht en snelheid, en rusthoeken voor/na het rijden. Bij het wisselen van rit worden alle grafieken en cijfers bijgewerkt en begint de tijdschuif opnieuw. Alle teksten blijven Nederlands.

De sectie **Pad in 3D** toont de geschatte baan als lint met de hoogte (overdrijving instelbaar), een hoogteprofiel tegen de afgelegde weg en een lijst gebeurtenissen: vastgelopen, schuin (pitch/roll > 7° gedurende ≥ 1 s), vrije val (< 0,35 g), vrij draaiende wielen en één kant die blokkeert. De hoogte is sin(pitch) × encoderafstand; tijdens het rijden komt de pitch uit de gyro, verankerd op de accelerometer in stilstand, omdat trillingen de accelerometer een schijnbare helling geven. Bij heen en terug wordt de kanteling van het chassis onder het motorkoppel geschat en afgetrokken; bij rechtdoor kan dat niet. Het dashboard laadt Three.js van internet; offline valt de robotstand terug op het 2D-schema.

Totale afstand telt heen en terug op. Getekende verplaatsing trekt terugrijden af. De baan combineert encoderafstand met gyrokoers; geen GPS-positie. Pitch en roll zijn accelerometerschattingen die tijdens rijden ook reageren op acceleratie en trillingen. Eindrust gebruikt het laatste halve seconde van uitbollen; als deze fase ontbreekt wordt geen eindrusthoek getoond.

CSV bevat tijd, fase, motoropdracht en ruwe sensorregel. Zonder CSV kan achteraf geen nieuw dashboard worden berekend. Analyse vereist IMU, rustfase en encoders van beide motordrivers.
