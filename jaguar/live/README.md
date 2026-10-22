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

Klik om punten te zetten, sleep ze om ze te verplaatsen, geef per hoek een radius voor een boog (0 = draaien ter plaatse), en pas lengte, draai en radius per stuk aan. De simulatie rijdt het robot op ware grootte over de route, met de breedte, de benodigde ruimte en de geschatte tijd. Pas na een bevestiging ("het robot staat goed, ruimte vrij") start de rit. Routes kun je bewaren en als commando kopiëren. De eenvoudige lijst met stappen blijft bereikbaar onderaan de keuzepagina.

Een boog kan op twee manieren gereden worden (keuze in het paneel Rijden):
- **In stukjes** (standaard, veilig): korte rechte stukjes van 10° met draaien ter plaatse (`jaguar/navigatie/route.py`, `boog_naar_stappen`). Begin, einde en eindkoers kloppen, maar het is langzaam (een boog van 90° duurt ongeveer 50 s) en elke draai ter plaatse schuift het robot een beetje.
- **Vloeiend** (nieuw, nog niet op het robot getest): `Jaguar.arc()` in `jaguar/kern/robot.py` rijdt de boog in één beweging. De doelkoers draait mee met de afstand (koers = start + hoek × afstand / booglengte), een voorspelde stuurwaarde voor de straal houdt de buitenkant sneller dan de binnenkant, en de gyro-PID stuurt de rest bij; daarna zet `eindkoers()` een resterende fout > 1° recht. Straal minstens 0,5 m, vermogen hoogstens 220. Controle zonder robot: `python -m jaguar.testen.sim_boog` (een model met slip en vertraging; in de simulatie lag het eindpunt op 1 tot 14 cm van het ideale punt). Begin op het echte robot met een ruime boog (straal vanaf 1 m), de noodstop bij de hand.

Zelftests van de ontwerper: open `ontwerp.html?selftest=1` (omzetting tekening → robotstappen) of `?selftest=2` (muisbewerkingen) in een browser; de titel zegt OK of FOUT.
