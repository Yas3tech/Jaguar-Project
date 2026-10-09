# Live dashboard met MQTT en Grafana

```
robotscript --mqtt  →  Mosquitto (MQTT)  →  Telegraf  →  InfluxDB  →  Grafana
   (laptop)            poort 1884                       poort 8086    http://localhost:3000
```

Het rijscript rekent de live waarden uit (gyro-bias, koers, snelheid, afstand, baan) en publiceert ze op MQTT. Telegraf schrijft alles in InfluxDB, en Grafana toont het dashboard **Jaguar live**. Elke rit blijft bewaard in InfluxDB en is achteraf terug te kijken via de keuzelijst **Rit**.

## Eenmalig: images ophalen (met internet)

De laptop heeft op de DriJaguar-wifi geen internet. Haal de images dus vooraf op:

```powershell
cd grafana
docker compose pull
```

## Elke keer

1. Start Docker Desktop.
2. Start de omgeving. Ze blijft daarna draaien, ook na een herstart van de pc:
   ```powershell
   cd grafana
   docker compose up -d
   ```
3. Open http://localhost:3000. Het dashboard **Jaguar live** opent meteen, zonder login.
4. Rijd met `--mqtt`:
   ```powershell
   python fase1_rechtdoor_imu.py --power 150 --duration 5 --mqtt
   python fase1_heen_en_terug.py --power 150 --duration 2 --mqtt --no-live
   ```
   `--no-live` zet de browserpagina van `live_view.py` uit. Je kunt ook beide tegelijk gebruiken.

Het dashboard ververst elke seconde en toont de laatste 2 minuten. Wil je een oude rit bekijken, kies ze dan bij **Rit** en zet de tijdsperiode rond die rit.

## Via het netwerk (twee laptops)

Docker en Grafana hoeven maar op **één** laptop te draaien. De laptop die met de robot rijdt, stuurt zijn metingen via MQTT naar die laptop:

```powershell
# laptop A: Grafana (eenmalig met internet: docker compose pull)
cd grafana; docker compose up -d
# laptop B: rijdt met de robot; 192.168.0.104 = IP van laptop A op de DriJaguar-wifi
python fase1_rechtdoor_imu.py --distance 1 --mqtt --mqtt-host 192.168.0.104
```

Iedereen op dezelfde wifi opent dan `http://192.168.0.104:3000/`. Laptop B heeft alleen Python en `paho-mqtt` nodig. Het IP van een laptop vind je met `ipconfig` (IPv4-adres van de wifi-adapter). De eerste keer vraagt Windows op laptop A om Docker door de firewall te laten: kies **Privénetwerk**.

De eenvoudige live pagina (`http://<IP-rijdende-laptop>:8765/`) werkt daarnaast ook, zonder Docker.

## Testen zonder robot

`nep_robot.py` speelt een opgenomen rit (`testdata/demo_rit.csv`) af op 127.0.0.1:10001. Gebruik twee terminals:

```powershell
python nep_robot.py
python fase1_rechtdoor_imu.py --ip 127.0.0.1 --mqtt --no-csv --no-live
```

Gebruik `--no-csv`, anders komt er een CSV met nepdata in `ritten/`. De nep-robot blijft draaien voor de volgende rit. Stoppen doe je met Ctrl+C.

## Wat er op het dashboard staat

De indeling volgt het analysedashboard (`dashboard.html`), maar dan live:

| Sectie | Panelen |
|---|---|
| Kerncijfers | verplaatsing, totale afstand, snelheid, hoogste snelheid, zijdelings, koersdrift (gyro en encoders), langs- en dwarshelling, hoogte, batterij (rood onder 22,2 V), E-Stop |
| Ritverkenner | fasebalk · **camera** (op de plek van de 3D-robotstand) · baan in bovenaanzicht · motoropdracht en snelheid |
| Pad in 3D: hoogte en gebeurtenissen | hoogteprofiel langs de afgelegde weg · gebeurtenissen: vastgelopen, schuin, vrije val, wielen vrij, één kant (zelfde drempels als `analyse_rit.py`) |
| Tracking | zijdelingse afwijking · koers gyro tegenover encoders |
| Niveau | pitch, roll en de gyro-pitch die de hoogte gebruikt |
| Trillingen en snelheid | verticale trilling (RMS per 0,2 s) · snelheid |
| Ruwe IMU en motorstatus | gyro, versnelling, batterij/stroom/temperatuur |

De 3D-weergaven bestaan niet in Grafana. Het camerapaneel toont `http://localhost:8766/camera.mjpg` zodra daar beeld is. De doorgeefserver voor de Axis-camera komt er later bij.

Panelen pas je aan in `maak_dashboard.py`, niet in de JSON. Draai daarna `python grafana/maak_dashboard.py`; Grafana leest het dashboard binnen ongeveer 10 s opnieuw in. Alle panelen delen 4 query's, want 25 losse query's haalden de verversing van 1 s niet.

## MQTT-berichten

| Topic | Frequentie | Velden |
|---|---|---|
| `jaguar/imu` | 50 Hz | `ts` (ms), `rit`, `t`, `fase`, `opdracht`, `gx gy gz` (°/s), `ax ay az` (g), `pitch roll koers koers_enc` (°), `snelheid` (m/s), `afstand x` (m), `y_cm` |
| `jaguar/status` | 2 Hz | `ts`, `rit`, `fase`, `klaar`, `estop`, `batterij_v`, `stroom_max_a`, `temp_max_c` |

Meekijken op de broker:

```powershell
docker exec jaguar-mosquitto-1 mosquitto_sub -t "jaguar/#" -v
```

## Goed om te weten

- **Poort 1884, niet 1883.** Op deze pc draait al een Windows-dienst `mosquitto` op poort 1883. De Docker-broker staat daarom op 1884 (zie `PORT` in `mqtt_stream.py`).
- **Draait de omgeving niet, dan rijdt de robot gewoon.** Het script meldt dan dat er geen broker is. Er wordt niets verstuurd, maar de CSV wordt wel opgeslagen.
- **De CSV blijft de bron.** Grafana toont de live benadering. De definitieve cijfers komen uit `analyse_rit.py` op de CSV.
- **Zelf panelen aanpassen** kan in Grafana zelf, want je bent automatisch admin. Wil je de wijziging bewaren, exporteer dan het dashboard (Share → Export) naar `dashboards/jaguar.json`.
- **Opruimen:**
  - `docker compose down` stopt alles en houdt de data.
  - `docker compose down -v` wist ook alle opgeslagen ritten in InfluxDB.
- **Wachtwoorden.** Wachtwoord en token in `docker-compose.yml` zijn alleen voor lokaal gebruik. Grafana staat open zonder login.
