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

Het robot kan nog geen vloeiende boog: een boog wordt als korte stukjes van 10° gereden (`jaguar/navigatie/route.py`, `boog_naar_stappen`). Het begin, het einde en de eindkoers kloppen, maar het duurt langer dan de simulatie laat zien. Zelftests van de ontwerper: open `ontwerp.html?selftest=1` (omzetting tekening → robotstappen) of `?selftest=2` (muisbewerkingen) in een browser; de titel zegt OK of FOUT.
