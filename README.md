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

`--distance` (standaard) is de afstand in meter volgens de slipbewuste odometrie (zie hieronder), voor heen/terug **per richting**. De robot bouwt het vermogen op over `--ramp` seconden (standaard 1,5), remt af naarmate het doel nadert, stopt 2 cm ervoor en legt de laatste millimeters af met korte kruipstoten (`--creep`, standaard 60). Na elke stoot meet hij in stilstand opnieuw, tot de fout binnen `--tolerance` mm ligt (standaard 3). Na elk segment toont het script het doel, de meting en de fout.

Grenzen van die precisie: één encoderpuls is 2,8 mm wielomtrek, en de encoders komen maar met ongeveer 5 Hz binnen. Als de robot bij vol vermogen 1,5 s nauwelijks vooruitgaat (vastgelopen, obstakel), breekt het script het segment af. Het stopt ook als de robot meer dan 35° kantelt t.o.v. de start, of als de wielen langer dan 1 s draaien zonder dat de robot rijdt (opgetild, op zijn neus, tegen een obstakel). Beweegt de robot tijdens de rustmeting, dan meet het script de rust opnieuw.

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

Dit schrijft `kalibratie.json`. De rijscripts en de analyse gebruiken daarna die straal. Verwijder het bestand om terug te gaan naar 0,135 m. Kalibreer op een harde, vlakke ondergrond: op gras zou de straal de slip mee opnemen. `python kalibreer.py 6.41 --vergelijk` slaat niets op en toont per methode (rijscript, gemiddelde 4 wielen, slipbewust, voor- en achterwielen) de fout t.o.v. de rolmeter.

## Afstand op elke ondergrond

`odometrie.py` berekent de afgelegde weg uit de vier wielencoders, getoetst aan de gyro en de accelerometer. Dezelfde code draait live in de rijscripts en achteraf in `analyse_rit.py`. Op een schone ondergrond is het resultaat gelijk aan het gemiddelde van de vier wielen; het verschil verschijnt pas als er iets slipt.

Wat de ritten van 2 en 8 oktober lieten zien (en waarop de drempels gebaseerd zijn):

- De motion controller stuurt de encoders op een vaste klok van 0,2 s; de wifi verschuift de ontvangsttijd ±20 ms. Snelheid = pulsen ÷ (n × 0,2 s) is daardoor 3,5× minder ruizig (0,019 i.p.v. 0,067 m/s).
- Soms houdt de wifi pakketten 0,5–0,7 s vast en levert ze dan samen (volgorde blijft juist). Het rijscript telt de tijd daarom met de IMU-volgnummers (50 Hz), niet met de ontvangsttijd. De eerste versie deed dat niet: in de ritten van 8 oktober vanaf 15:03 verwierp ze daardoor soms goede wielwaarden en reed de robot 7 tot 30 cm te ver.
- Voor- en achterwiel aan dezelfde kant hebben in rechte, schone stukken dezelfde snelheid (verhouding 1,000; verschil < 0,04 m/s bij p90). Wat in een rit toch verschilt, ontstaat bij optrekken: slip.
- De IMU alleen kan een slippende periode maar kort overbruggen: fout (p90) 1,5 cm na 0,2 s, 7 cm na 0,6 s, 17 cm na 1 s, 70 cm na 2 s. Dubbel integreren als afstandsmeting werkt dus niet (mediane fout 17 % per segment).
- Wielen die de grond niet raken trillen ≈ 12× minder (0,035 g) dan wielen die echt rijden (0,43 g bij > 0,6 m/s).
- Rit `134323`: na een botsing op ≈ 5,5 m kantelde de robot op zijn neus (accelerometer −0,94 g op X). De encoders telden daarna nog 7 m terugrijden terwijl de robot stilstond. Slipbewust blijft daarvan −0,17 m over.

Werking per encoderpakket:

1. Per kant: voor en achter gelijk → gemiddelde; anders het wiel dat het best past bij de snelheid die de IMU voorspelt (het andere slipt).
2. Links − rechts moet kloppen met de gyro; zo niet, dan telt de kant die het best bij de IMU past.
3. Een Kalman-filter (snelheid + versnellingsbias) voorspelt met de accelerometer, met de zwaartekracht eraf via de pitch uit de gyro. Wijken de wielen te ver af, dan telt de IMU, begrensd tussen 0 en het snelste wiel. Na 0,6 s heet dat "afstand onzeker" en stopt het rijscript na 1 s.
4. Stilstand (geen pulsen, geen trilling) zet de snelheid op nul en stelt pitch en gyrobias bij.

Het dashboard toont de slipbewuste afstand met een onzekerheid, het gemiddelde van de vier wielen ter vergelijking, en de gebeurtenissen slip, IMU overbrugt, afstand onzeker en wielen draaien door.

**Grens:** slippen alle vier de wielen evenveel aan constante snelheid (gras, zand), dan ziet niemand het: de wielen zijn het eens en de accelerometer meet geen versnelling. In de simulator gaf 15 % slip over 1,5 m 26 cm te veel. Daarvoor dient de proef met vrijlopende meetwielen:

```powershell
python test_sondewiel.py adres                 # robot OP BLOKKEN: rijden MM0 en MM1 apart? lopen de wielen vrij uit?
python test_sondewiel.py rijden --distance 3   # alleen de voorwielen trekken, de achterwielen meten de grond
python kalibreer.py 2.97 --vergelijk           # rolmeter: welke methode zat het dichtst?
python test_sondewiel.py draai --turns 2       # gyroschaal: draai ter plaatse, geef de echte hoek in
```

Wielen zonder vermogen hebben geen aandrijfslip en rollen mee met de grond, als de driver ze bij vermogen 0 niet afremt. `adres` controleert dat eerst. De Dr Robot-software stuurt alleen `MMW` (beide drivers); dat `MM0 !M` en `MM1 !M` apart werken, is nog niet op de robot getest. De draaiproef is nodig omdat de gyroschaal uit de datasheet komt: bij het kantelen in rit `134323` telde de gyro 81° tegen 85° volgens de accelerometer.

## Dashboard

Open `dashboard.html` in de projectmap en kies een rit met **Kies een rit**. Elke uitvoering van `python analyse_rit.py` werkt ditzelfde dashboard bij met alle `rit_*_analyse.json`-bestanden; de zojuist geanalyseerde rit is standaard geselecteerd. Ververs de browser na een nieuwe analyse. Het dashboard opent rechtstreeks in de browser, zonder server; voor de 3D-beelden is internet nodig (Three.js). Alle teksten zijn Nederlands en ook bedoeld voor wie niet aan het project werkte.

Opbouw, van eenvoudig naar technisch:

- **In het kort**: één alinea en vijf vragen met antwoord (hoe ver, terug op de start, rechtdoor, slip, iets bijzonders).
- **Bekijk de rit**: afspelen (½× tot 4×) met een tijdlijn van de fases en gebeurtenissen. Het bovenaanzicht toont start, doel, het gereden pad en de robot; de zijwaartse afwijking is standaard vergroot (knop voor echte schaal). Wielen kleuren geel bij slip en rood als ze doordraaien. De 3D-weergave toont hetzelfde robotmodel dat over zijn pad rijdt; de camera volgt de robot.
- **Wat gebeurde er onderweg?**: gebeurtenissen in gewone taal; klik om ernaartoe te springen. Nieuw is **tegengehouden**: vermogen aan, de robot minstens 25 % trager dan normaal en minstens 3× de normale motorstroom (≥ 1,5 A), minstens 0,3 s. Dat is geen slip: de wielen houden grip en de afstand klopt.
- **Hoe weet de robot hoe ver hij reed?**: drie animaties (wielen tellen, slip, hoe slip ontdekt wordt) en wat de robot nog niet ziet.
- **Technische details**: ingeklapt; bevat de vroegere kerncijfers, robotstand van dichtbij, motoropdracht, hoogteprofiel, rechtlijnigheid, helling, trillingen, snelheid, alle gebeurtenissen en de meetwaardentabel.

Totale afstand telt alle bewegingen op, ook de kleine kruipstoten heen en weer bij het stoppen. Getekende verplaatsing trekt terugrijden af. De baan combineert de afgelegde weg met de gyrokoers; geen GPS-positie.

CSV bevat tijd, fase, motoropdracht en ruwe sensorregel. Zonder CSV kan achteraf geen nieuw dashboard worden berekend. Analyse vereist IMU, rustfase en encoders van beide motordrivers.
