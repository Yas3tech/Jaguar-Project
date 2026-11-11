# rijden/ — de rijscripts

| Script | Wat het doet |
|---|---|
| `rechtdoor.py` | Rechtdoor rijden: `python -m jaguar.rijden.rechtdoor --power 250 --distance 2` |
| `heen_en_terug.py` | Heen en terug rijden: `python -m jaguar.rijden.heen_en_terug --distance 1.5` |
| `parcours.py` | Een reeks stappen (vooruit, achteruit, links, rechts): `python -m jaguar.rijden.parcours v:2 l:90 v:2` |
| `bediening.py` | Hetzelfde vanuit de browser, met een knop Rijden (een ontwerper met raster of kaart, bogen en simulatie), Pauze, Terug naar start, Handmatig en STOP: `python -m jaguar.rijden.bediening` |
| `gps_route.py` | Naar GPS-coördinaten rijden, één punt of een route: `python -m jaguar.rijden.gps_route --vanaf-hier 50.84010,4.32000`. Zie [navigatie/](../navigatie/README.md). |

Elke rit slaat een CSV op in `ritten/`. Alle scripts hebben `--no-csv`, `--no-live`, `--ip` en `--mqtt`. Met `--ip 127.0.0.1` rijd je tegen de nep-robot of de gesimuleerde robot (`python -m jaguar.testen.sim_robot`).

## Rechtdoor in één beweging

Een rechte afstand (bv. 5 m) rijdt het robot standaard in **één vloeiende beweging**: optrekken, rijden, afremmen, en het vermogen uit op het moment dat de uitloop het robot op het doel brengt. De uitloop is hoeveel het robot nog rolt na vermogen 0; het leert die na elke stop bij. Het rijdt dan geen kruipstoten en nooit achteruit om een paar centimeter goed te maken.

- Staat het robot na de stop meer dan de tolerantie te kort, dan rijdt het hoogstens twee keer een stukje vooruit.
- Alleen een overschot van meer dan 10 cm wordt teruggereden.
- De standaard tolerantie is `--tolerance 10` mm (was 3 mm).
- In de simulatie (`python -m jaguar.testen.test_rijden`): 5 m gevraagd, fout 2 tot 12 mm, 0 cm achteruit, 1 stop. Op de echte robot moet je dit nog controleren.
- Met `--precies` werkt de oude manier: kruipstoten en millimeterprecisie, met heen en weer aan het eind (zie hieronder).

## Met --precies: kruipstoten en de driver

Met `--precies` stopt het vloeiende rijden 2 cm voor het doel, en de laatste millimeters gaan met korte kruipstoten (`--creep`, standaard 60). Na elke stoot meet de robot in stilstand opnieuw, tot de fout binnen de tolerantie ligt.

Op de echte robot regelt de motordriver het laatste stuk (30 cm) zelf: `robot.py` zet hem even in positiemodus (Roboteq `MMOD 3`) en geeft elk wiel een absoluut doel in pulsen, waarna de driver het wiel 1000 keer per seconde op de puls nauwkeurig naar dat doel regelt (PID 255/0/120, 40 % vermogen). Daarvoor houdt het script met de gyro de koers vast. Gemeten op de robot: 1 m en 5 m, vooruit en achteruit, binnen 1 à 3 mm van het doel (volgens de encoders). Elk driver-commando wacht op zijn bevestiging en wordt herhaald, want de drivers laten commando's vallen als ze te snel na elkaar komen. Daarna zet het script de driver terug op de fabrieksinstelling (open loop, PID 30/10/0). Mislukt het, dan rijdt het zoals vroeger met kruipstoten. Uitzetten kan met `JAGUAR_DRIVER=0`; de nep-robot gebruikt het niet. Over de hele afstand laten we de driver niet regelen: elk wiel gaat dan apart naar zijn doel en het robot draait weg (28° in 5 m buiten). `python -m jaguar.kalibratie.test_driver` leest de driverinstellingen uit (alleen lezen).

Grenzen van die precisie: één encoderpuls is 2,8 mm wielomtrek, en de encoders komen maar met ongeveer 5 Hz binnen. Als de robot bij vol vermogen 1,5 s nauwelijks vooruitgaat (vastgelopen, obstakel), breekt het script het segment af. Het stopt ook als de robot meer dan 35° kantelt t.o.v. de start, of als de wielen langer dan 1 s draaien zonder dat de robot rijdt (opgetild, op zijn neus, tegen een obstakel). Beweegt de robot tijdens de rustmeting, dan meet het script de rust opnieuw.

```powershell
python -m jaguar.rijden.rechtdoor --power 250 --distance 5                 # één beweging
python -m jaguar.rijden.rechtdoor --power 250 --distance 5 --precies       # oude manier: kruipstoten
```

Een parcours met meerdere stappen en draaien ter plaatse:

```powershell
python -m jaguar.rijden.parcours vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5
python -m jaguar.rijden.parcours v:2 l:90 v:2 l:90 v:2 l:90 v:2      # vierkant
```

## Draaien

- Draaien ter plaatse gebeurt op de gyro: het vermogensverschil zakt met de resterende hoek, en de robot stopt vroeger volgens zijn draaisnelheid (uitloop). Ook die uitloop leert hij bij (`turn_lead`).
- Na de vloeiende draai corrigeert hij met korte stoten alleen als de fout groter is dan 1° (`TURN_FINE`; met `--precies` tot 0,4°). Binnen 1° volgt geen stoot meer: de koers is absoluut en het volgende rechte stuk stuurt een graad vanzelf weg. Zo zijn er geen kleine heen-en-weer stoten meer.
- Komt hij niet in beweging (wrijving), dan verhoogt hij het minimumvermogen stap voor stap.
- De robot leert hoeveel graden een stoot geeft, en stopt als de kleinste stoot de fout alleen zou vergroten.
- Draaien is zo juist als de gyroschaal: kalibreer eerst met `python -m jaguar.kalibratie.test_sondewiel draai`.
- Na elke draai vergelijkt het script gyro en encoders (`RATIO` in de CSV) en meldt het als die verhouding plots meer dan 15 % verandert.

De bijgeleerde regelwaarden (vermogen→snelheid, kruipstoot, draaistoot, uitloop) staan na een rit in `config/leerwaarden.json` en worden bij de volgende rit weer geladen, zodat je niet elke keer opnieuw hoeft in te regelen.

Rechte stappen rijden naar het geplande hoekpunt (gerekend vanaf de start) en volgen de geplande lijn. Een afwijking van de vorige stap (te ver, te kort, naast de lijn) werkt de volgende stap dus vanzelf weg, tot 50 cm per stap; een grotere correctie vertrouwt het script niet. Na de laatste stap zet het de koers recht als die meer dan 1,5° afwijkt. Hetzelfde doen `rechtdoor` en `heen_en_terug` aan het eind van elk segment (`eindkoers`). Dat corrigeert alles wat de odometrie meet; wat ze niet ziet (gyroschaal, wielstraal op een andere ondergrond) blijft staan. Het dashboard toont de geplande route en de gereden route op ware schaal.

Richting: `--distance -3` of `--power -250` rijdt achteruit. Bij heen/terug betekent een negatief vermogen of een negatieve afstand: eerst achteruit, dan terug vooruit.

Rechtdoor houden met een PID-regelaar (cascade): tijdens de rustfase meet het script de gyrobias. Tijdens het rijden geeft de zijdelingse afwijking een gewenste koers (20° per meter, max. 5°, terug naar de startlijn). Een PID op de gyrokoers zet die om in een vermogensverschil tussen links en rechts:

- P (`KP_HEADING` = 6 per graad) reageert op de koersfout;
- I (`KI_HEADING` = 4 per graad·s) compenseert een kant die blijvend zwakker trekt; bij achteruit rijden wordt die geleerde waarde gespiegeld;
- D (`KD_HEADING` = 0,4 per graad/s, op de gemeten draaisnelheid) dempt overslingeren.

Het verschil is begrensd op 30 % van het rijvermogen (60 % bij de kruipstoten). De integrator stopt met groeien zolang de uitgang begrensd is (anti-windup). Zonder regeling draaide de robot bij het optrekken 3,6° weg en eindigde hij 17 cm naast de lijn na 3 m. Na elk segment toont het script de koers en de geschatte afstand tot de startlijn. Slingert de robot heen en weer: verlaag `KP_HEADING`. Blijft hij scheef: verhoog `KI_HEADING`.

`--duration` rijdt op tijd zoals vroeger. `--power` is een open-loop motoropdracht en het maximum tijdens het rijden, geen snelheid in m/s. Het bereik is 0..1000 voor heen/terug en -1000..1000 voor rechtdoor (negatief = achteruit).

## Bediening vanuit de browser

- **Pauze** (knop of P): het robot stopt waar het is. **Hervatten** rijdt dezelfde stap verder naar hetzelfde geplande hoekpunt of dezelfde koers (een draai of boog: de rest ervan). Pauzeren mag ook vanaf een ander apparaat dat meekijkt, zoals STOP; hervatten alleen op de laptop. In de analyse telt een stap met een pauze als één stap.
- **Terug naar start**: tijdens een rit breekt dit het parcours af; het robot draait naar de start, rijdt er in een rechte lijn heen (ligt de start achter hem en dichter dan 3 m: achteruit) en draait naar de beginrichting (`naar_start` in `parcours.py`). Na de rit kan het ook nog, met de laatste positie volgens de odometrie: dan mag niemand het robot intussen verplaatst hebben. De fasen heten `terug-1-draai`, `terug-2-vooruit`, …; een aparte terugrit krijgt een CSV `_terug`. Het robot kijkt niet of de rechte lijn vrij is (alleen de camera vooruit, als Obstakels aan staat).
- **Batterij**: de spanning springt ±0,5 V tussen twee metingen, dus `robot.py` kijkt naar de mediaan over 10 s. Beginnen kan vanaf 22,2 V. Zakt ze tijdens een parcours onder 21,6 V (3,6 V per cel), dan breekt het parcours af en rijdt het robot vanzelf terug naar de start; onder 21,0 V (3,5 V per cel) stopt het helemaal, ook op de terugweg. Handmatig rijden stopt onder 21,6 V; Terug naar start kan dan nog (tot 21,0 V). Dat geldt voor alle rijscripts: zonder bediening.py stopt de rit onder 21,6 V.
- **Handmatig**: rijden met de pijltjes of WASD, de knoppen op het scherm (ingedrukt houden) of een gamepad (linkerstick, B = STOP), met een schuif voor het vermogen. De pagina stuurt 10 keer per seconde de stand; komt er 0,4 s niets (tabblad dicht, wifi weg), dan staat het robot stil. Rechtdoor houdt de gyro de koers vast. De veiligheidsstops blijven gelden (kantelen, vastgelopen, obstakel vooruit); daarna eerst alles loslaten. Alleen op de laptop. De rit krijgt een CSV `_handmatig` en de analyse werkt zoals altijd; daarna brengt Terug naar start het robot naar waar het handmatig rijden begon.
