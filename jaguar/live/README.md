# live/ — meekijken tijdens het rijden

- `view.py`: de lokale webserver (poort 8765) en de berekeningen voor de tegels en grafieken.
- `pagina.html`: de pagina die je in de browser ziet (tegels, grafieken, Rijden, STOP, Analyse, Opties).
- `ontwerp.html`: de ontwerper achter de knop Rijden (zie hieronder).
- `opties.json`: bewaarde keuzes van het venster Opties (grafieken tonen of verbergen, vanzelf analyseren).
- `mqtt_stream.py`: stuurt dezelfde waarden naar MQTT voor Grafana (zie `grafana/README.md`).

Elk rijscript start dit vanzelf. Uitschakelen kan met `--no-live`.

## Parcours ontwerpen (knop Rijden, alleen met `python -m jaguar.rijden.bediening`)
Eerst kies je tussen twee kaarten:
- **Binnen**: een raster in echte meters, zonder GPS.
- **Buiten**: een kaart of satelliet (OpenStreetMap / Esri; de tegels komen van internet, dus zonder internet werkt alleen het raster en vul je het startpunt als coördinaten in) met de coördinaten van elk punt en de richting bij de start.

**Tekenen met de muis** (standaard): houd de muisknop ingedrukt en teken vanaf de groene start; bij loslaten worden de punten en de bogen vanzelf gezet (de lijn wordt gladgestreken en vereenvoudigd met Ramer-Douglas-Peucker, een zachte bocht (minder dan 50°) wordt een boog met de grootste radius die past, hoogstens de helft van elk aangrenzend stuk; een scherpe hoek wordt een draai ter plaatse. Met de knop **Auto / Alles rond / Scherp** kies je dat zelf. Past een boog niet tussen twee hoeken, dan wordt hij vanzelf kleiner en meldt de pagina dat, in plaats van de start te blokkeren). De pagina meldt hoe ver de route van je lijn afwijkt; met de schuif **Gladheid** vereenvoudig je dezelfde lijn sterker of minder sterk. Verder tekenen vanaf het laatste punt kan door daar te beginnen, anders begint een nieuwe lijn. Alt + slepen schuift het beeld. Met **✋ Verschuiven** (of de spatiebalk ingedrukt houden) sleep je alleen de kaart. Met **● Punten** zet je zelf punten door te klikken: sleep ze om ze te verplaatsen, geef per hoek een radius voor een boog (0 = draaien ter plaatse), en pas lengte, draai en radius per stuk aan. De simulatie rijdt het robot op ware grootte over de route, met de breedte, de benodigde ruimte en de geschatte tijd. Pas na een bevestiging ("het robot staat goed, ruimte vrij") start de rit. Routes kun je bewaren en als commando kopiëren. Wil je niet zelf tekenen? **✦ Voorbeelden** zet de punten voor je: rechte lijn, heen en terug, vierkant, rechthoek, ovaal, cirkel en slalom, elk met instelbare maten en een voorvertoning; daarna kun je nog alles verslepen. Een route kun je ook als tekst plakken of kopiëren (lijst punten `[x, y, radius]` met `b0`), bijvoorbeeld een die je laat maken. De eenvoudige lijst met stappen blijft bereikbaar onderaan de keuzepagina.

Een boog kan op twee manieren gereden worden (keuze in het paneel Rijden):
- **In stukjes** (standaard, veilig): korte rechte stukjes van 10° met draaien ter plaatse (`jaguar/navigatie/route.py`, `boog_naar_stappen`). Begin, einde en eindkoers kloppen, maar het is langzaam (een boog van 90° duurt ongeveer 50 s) en elke draai ter plaatse schuift het robot een beetje.
- **Vloeiend** (nieuw, nog niet op het robot getest): `Jaguar.arc()` in `jaguar/kern/robot.py` rijdt de boog in één beweging. De doelkoers draait mee met de afstand (koers = start + hoek × afstand / booglengte), een voorspelde stuurwaarde voor de straal houdt de buitenkant sneller dan de binnenkant, en de gyro-PID stuurt de rest bij; daarna zet `eindkoers()` een resterende fout > 1° recht. Straal minstens 0,5 m, vermogen hoogstens 220. Controle zonder robot: `python -m jaguar.testen.sim_boog` (een model met slip en vertraging; in de simulatie lag het eindpunt op 1 tot 14 cm van het ideale punt). Begin op het echte robot met een ruime boog (straal vanaf 1 m), de noodstop bij de hand.

Zelftests van de ontwerper: open `ontwerp.html?selftest=1` (omzetting tekening → robotstappen) of `?selftest=2` (muisbewerkingen) in een browser; de titel zegt OK of FOUT.
