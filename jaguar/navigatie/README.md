# navigatie/ — GPS-coördinaten en het noorden

- `gps.py`: GPRMC lezen (met controlesom), coördinaten ↔ meters (oost, noord) rond een startpunt, het noorden uit de koers over de grond (`NoordPeiling`) en `GpsAnker`, dat de absolute positie bijhoudt maar slechte GPS weigert.
- `route.py`: een lijst GPS-punten omzetten naar parcoursstappen (draaien en rechtdoor).

Het rijden zelf gebeurt met `python -m jaguar.rijden.gps_route` (zie die map). Controle zonder robot: `python -m jaguar.testen.test_navigatie`.

## Wat de GPS hier kan en niet kan
Het robot heeft een Garmin GPS 18x-5Hz (5 posities per seconde; < 15 m, met WAAS < 3 m, alleen onder open hemel). De controller geeft alleen `GPRMC` door (positie, snelheid, koers over de grond), geen kwaliteit. Gemeten: binnen meldt de ontvanger toch een fix terwijl de positie 20 tot 50 m dwaalt bij een stilstaand robot. Daarom:
- de route (hoeken, afstanden) komt uit de opgegeven coördinaten en wordt gereden met wielen en gyro, zoals altijd;
- de GPS levert alleen de richting van het noorden (bij de start, uit de koers over de grond boven 1 m/s) en een absolute positie die pas telt als het verschil GPS − odometrie vastblijft (`GpsAnker.betrouwbaar`);
- de magnetometer (kompas) staat niet in de datastroom van dit robot, en Dr Robot raadt hem zelf af wegens motorstoring.
Centimeters in GPS-coördinaten zijn met deze ontvanger niet haalbaar (daarvoor is RTK nodig).
