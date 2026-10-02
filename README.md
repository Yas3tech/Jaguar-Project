# Jaguar-Project

## Rijden en opslaan

Beide scripts slaan standaard een CSV op in `ritten`. Gebruik `--no-csv` om dit uit te schakelen; `--csv` om het expliciet aan te zetten.

```powershell
python fase1_rechtdoor_imu.py --power 150 --duration 5 --csv
python fase1_heen_en_terug.py --power 150 --duration 2 --pause 1.5 --csv
python fase1_rechtdoor_imu.py --power 150 --duration 5 --no-csv
python analyse_rit.py
python analyse_rit.py ritten/rit_20261002_100122.csv
```

`--duration` is de rijtijd voor rechtdoor en de tijd **per richting** voor heen/terug. Met dezelfde waarde rijdt heen/terug dus twee rijsegmenten. De werkelijke afstand wordt gemeten, niet gegarandeerd door de tijd. Beide logs kunnen met dezelfde analyse worden verwerkt.

`--power` is een open-loop motoropdracht, geen snelheid in m/s. Het bereik is 0..1000 voor heen/terug en -1000..1000 voor rechtdoor (negatief = achteruit). De maximale fysieke robotsnelheid is niet vastgesteld in dit project. Een encoderpiek in een log is geen robotspecificatie en kan be?nvloed zijn door pakketvertraging.

## Dashboard

Open `dashboard.html` in de projectmap en kies een rit met **Selecteer een rit**. Elke uitvoering van `python analyse_rit.py` werkt ditzelfde dashboard bij met alle `rit_*_analyse.json`-bestanden uit de map van de geanalyseerde CSV. De zojuist geanalyseerde rit is standaard geselecteerd. Ververs de browser na een nieuwe analyse. Er wordt geen apart HTML-bestand per rit meer gemaakt; bestaande HTML-rapporten blijven beschikbaar.

Het dashboard opent rechtstreeks in de browser, zonder server. Het bevat een tijdschuif, een geschatte baan met gelijke schaal op beide assen, pitch/roll-aanzichten, motoropdracht en snelheid, en rusthoeken voor/na het rijden. Bij het wisselen van rit worden alle grafieken en cijfers bijgewerkt en begint de tijdschuif opnieuw. Alle teksten blijven Nederlands.

Totale afstand telt heen en terug op. Getekende verplaatsing trekt terugrijden af. De baan combineert encoderafstand met gyrokoers; geen GPS-positie. Pitch en roll zijn accelerometerschattingen die tijdens rijden ook reageren op acceleratie en trillingen. Eindrust gebruikt het laatste halve seconde van uitbollen; als deze fase ontbreekt wordt geen eindrusthoek getoond.

CSV bevat tijd, fase, motoropdracht en ruwe sensorregel. Zonder CSV kan achteraf geen nieuw dashboard worden berekend. Analyse vereist IMU, rustfase en encoders van beide motordrivers.
