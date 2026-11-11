# navigatie/ — GPS-coördinaten en het noorden

- `gps.py`: GPRMC lezen (met controlesom), coördinaten ↔ meters (oost, noord) rond een startpunt, het noorden uit de koers over de grond (`NoordPeiling`), `Uitlijning` (het gereden spoor op het GPS-spoor leggen, zie hieronder) en `GpsAnker` (een absolute positie bijhouden die slechte GPS weigert; `gps_route` gebruikt die niet meer). `GpsKwaliteit` zegt hoe goed de GPS op een plek is, uit de spreiding en de wegloop van de positie bij een stilstaand robot (knop GPS meten in de ontwerper).
- `route.py`: een lijst GPS-punten omzetten naar parcoursstappen (draaien en rechtdoor).

Het rijden naar coördinaten gebeurt met `python -m jaguar.rijden.gps_route` (zie hieronder en [rijden/](../rijden/README.md)). Controle zonder robot: `python -m jaguar.testen.test_navigatie`.

## Wat de GPS hier kan en niet kan

Volgens de fabrikant (Garmin GPS 18x-5Hz):

- positie: fout kleiner dan 15 m in 95 % van de gevallen; met WAAS/EGNOS kleiner dan 3 m; alleen onder open hemel;
- snelheid: 0,1 knoop RMS;
- 5 posities per seconde;
- kompas: onbruikbaar, want de motoren storen het (Dr Robot raadt het zelf af).

De controller geeft alleen `GPRMC` door (positie, snelheid en koers over de grond), geen kwaliteit en geen satellieten. Gemeten: binnen meldt de ontvanger toch een fix terwijl de positie 20 tot 50 m dwaalt bij een stilstaand robot. Een fix zegt dus niets over de kwaliteit.

Daarom levert de GPS hier alleen wat de wielen en de gyro niet kunnen: coördinaten in de wereld en de richting van het noorden. De precisie van een route komt van de encoders en de gyro. Centimeters in GPS-coördinaten zijn met deze ontvanger niet haalbaar (daarvoor is RTK nodig).

**De grens:** geen programma komt dichter bij een coördinaat dan de GPS-fout, als alleen de GPS zegt waar het robot begint. Zet het robot daarvoor op een bekend punt (het eerste coördinaat) en geef de richting op met `--richting`.

## Hoe gps_route rijdt

`python -m jaguar.rijden.gps_route` rijdt naar één punt of langs een route:

```powershell
python -m jaguar.rijden.gps_route --vanaf-hier 50.84010,4.32000       # ga naar dit punt, vanaf waar het robot nu staat
python -m jaguar.rijden.gps_route 50.84000,4.32000 50.84010,4.32000 --richting 90   # het eerste punt is de start
python -m jaguar.rijden.gps_route 50.84000,4.32000 50.84010,4.32000 --richting 0 --droog   # alleen het plan tonen
```

- **Waar het robot begint.** Standaard is het eerste punt de start: daar staat het robot (coördinaat 1 = nulpunt). Dat is het nauwkeurigst, want de route hangt dan niet af van waar de GPS denkt dat het robot staat. Met `--vanaf-hier` meet het robot zelf waar het staat: het gemiddelde van de GPS in rust (`--gps-wacht` seconden, standaard 10). Alle punten zijn dan doelen, en de GPS-fout op dat moment zit in de hele route.
- **Het noorden.** `--richting <graden>` is de kompaskoers waar het robot bij de start naartoe kijkt (0 = noord, 90 = oost). Zonder `--richting` rijdt het robot eerst 6 m rechtdoor om het noorden te vinden (`--kalibreer-noord <m>` voor een andere lengte). Dat stuk rijdt het sneller (`--noord-power`, standaard 400, ongeveer 1,1 m/s: de koers over de grond klopt pas daarboven). Het noorden komt uit de koers over de grond en uit het gereden spoor; de nauwkeurigste van de twee telt.
- **Hoe het rijdt.** Afstanden en hoeken rijdt het robot met de wielen en de gyro, elk recht stuk in één beweging. Over tientallen meters is dat veel nauwkeuriger dan de GPS.
- **Gebruik van de GPS** (`--gps auto|aan|uit`, standaard `auto`). Het script kiest per punt:

  1. altijd: het gereden spoor wordt op het GPS-spoor gelegd (`Uitlijning`). De draaiing verbetert het noorden, de schaal verbetert de afstand (slip). Een vaste fout van de GPS doet daar niets aan af, want alleen de vorm van het spoor telt;
  2. de positie van de GPS zelf telt pas als haar geschatte fout kleiner is dan die van de wielen (2 % van de gereden afstand, plus de onzekerheid op het noorden). Met de GPS van het robot is dat pas na tientallen meters; met een nauwkeurige GPS (de websimulator, RTK) meteen;
  3. past het GPS-spoor niet op het gereden spoor (binnen, tussen gebouwen), dan telt de GPS niet.
- **Vertraging.** De GPS loopt achter op de beweging (Dr Robot noemt dat zelf). Het script schat die vertraging (tot 1 s, in stappen van één GPS-zin) en rekent ze weg.
- `--gps uit`: alleen wielen en gyro. `--gps aan`: de GPS-positie altijd gebruiken zodra het spoor past.
- `--droog`: alleen tonen wat het zou rijden. Dat heeft `--richting` nodig en kan niet met `--vanaf-hier`.

## Het rapport na de rit

Het script drukt af:

- waar het robot staat volgens de wielen en volgens de GPS, en hoe ver dat van het laatste punt is;
- of het GPS-spoor op het gereden spoor past (rms), en hoe goed de GPS daarmee is;
- het noorden volgens het spoor, en het verschil met de startrichting;
- de schaal: echte meters (GPS) per meter odometrie. Wijkt die af van 1, dan rijdt het robot minder of meer ver dan de wielen tellen (slip, wielstraal);
- per punt hoe vaak alleen de wielen zijn gebruikt, het noorden of de schaal uit het spoor, of de GPS-positie.

## Uitlijning

`Uitlijning` (onderaan `gps.py`) legt het spoor van de odometrie (x vooruit, y links, vanaf de start) op het spoor van de GPS (oost, noord). Het zoekt de best passende draaiing, verschuiving en schaal (kleinste kwadraten over alle metingen). Daaruit komt:
- `b0`: het noorden, uit de vorm van het gereden spoor;
- `schaal`: echte meters per meter odometrie;
- `rms`: hoe ver de GPS-punten gemiddeld naast het gepaste spoor liggen. Klein = rustige GPS; een dwalende GPS (binnen) past op geen enkel spoor en geeft een grote rms;
- `basis`: hoe lang het gereden spoor is. Zolang dat klein is tegenover de GPS-fout, zegt de passing niets;
- `positie()` zet de odometrie om naar (oost, noord); `naar_odo()` zet een punt op de kaart om naar het stelsel van de odometrie, waar het robot naartoe moet rijden.

## Ontwerper (Buiten)

De ontwerper (knop Rijden, kaart Buiten) stuurt het startpunt en de startrichting mee met de rit. Na de rit toont de pagina wat de GPS zegt: waar het robot staat tegenover het eind van de route, of het GPS-spoor op het gereden spoor past, en of de startrichting op de kaart klopte. Zie [live/](../live/README.md).
