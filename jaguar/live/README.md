# live/ — meekijken tijdens het rijden

- `view.py`: de lokale webserver (poort 8765) en de berekeningen voor de tegels en grafieken.
- `pagina.html`: de pagina die je in de browser ziet (tegels, grafieken, Rijden, STOP, Analyse, Opties).
- `opties.json`: bewaarde keuzes van het venster Opties (grafieken tonen of verbergen, vanzelf analyseren).
- `mqtt_stream.py`: stuurt dezelfde waarden naar MQTT voor Grafana (zie `grafana/README.md`).

Elk rijscript start dit vanzelf. Uitschakelen kan met `--no-live`.
