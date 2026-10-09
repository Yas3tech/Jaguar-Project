"""Live meetgegevens naar MQTT sturen, voor Grafana (zie grafana/README.md).

Keten: robot -> dit script -> Mosquitto -> Telegraf -> InfluxDB -> Grafana.
De berekeningen (gyro-bias, koers, snelheid, baan) komen uit live_view.LiveView;
dit bestand publiceert alleen elke nieuwe IMU-rij en de motorstatus.

Topics (JSON, ts = unix-tijd in ms):
  jaguar/imu     elke IMU-sample (50 Hz): fase, opdracht, gyro, versnelling, hoeken, koers, snelheid, afstand,
                 x, y, totale afstand, hoogte, trilling, gebeurtenis
  jaguar/status  max. 2 per seconde: batterij, stroom, temperatuur, E-Stop
Een ontbrekende broker stopt het rijden nooit: dan wordt er gewoon niets verstuurd.
"""
import json
import time

from live_view import LiveView

BROKER = "127.0.0.1"
PORT = 1884          # Docker-broker uit grafana/docker-compose.yml (1883 = Windows-dienst mosquitto)
STATUS_PERIOD = 0.5


class MqttStream:
    def __init__(self, rit, view=None, host=BROKER, port=PORT):
        import paho.mqtt.client as mqtt   # pip install paho-mqtt
        self.rit = rit
        self.view = view or LiveView(rit)
        self.sent = 0
        self.status_t = 0.0
        self.wall0 = None     # unix-tijd die overeenkomt met t = 0 van de rit
        self.last_ms = 0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"jaguar-{rit}")
        self.client.on_connect = lambda c, u, f, rc, p: print(
            f"MQTT: verbonden met {host}:{port}" if not rc.is_failure else f"MQTT: geweigerd ({rc})")
        try:
            self.client.connect(host, port)       # eerst verbinden, dan pas de robot: anders mist de rustfase
        except OSError:
            print(f"MQTT: geen broker op {host}:{port} (docker compose up -d in grafana/); rijden gaat gewoon door.")
            self.client.connect_async(host, port)  # blijft op de achtergrond opnieuw proberen
            self.client.loop_start()
            return
        self.client.loop_start()
        for _ in range(20):                        # max. 1 s wachten op de bevestiging van de broker
            if self.client.is_connected():
                break
            time.sleep(0.05)

    def feed(self, t, phase, power, line):
        self.view.feed(t, phase, power, line)
        if self.wall0 is None:
            self.wall0 = time.time() - t
        if not self.client.is_connected():
            self.sent = len(self.view.samples)   # zonder broker niets opsparen
            return
        now = time.time()
        for r in self.view.samples[self.sent:]:
            g, a = r[3], r[4]
            # InfluxDB overschrijft punten met dezelfde tijd; gebundelde wifi-pakketten
            # hebben bijna dezelfde t, dus elke sample minstens 1 ms na de vorige.
            ts = self.last_ms = max(int((self.wall0 + r[0]) * 1000), self.last_ms + 1)
            self._publish("jaguar/imu", {
                "ts": ts, "rit": self.rit, "t": r[0], "fase": r[1], "opdracht": r[2],
                "gx": g[0], "gy": g[1], "gz": g[2], "ax": a[0], "ay": a[1], "az": a[2],
                "pitch": r[5], "roll": r[6], "koers": r[7], "koers_enc": r[8],
                "snelheid": r[9], "afstand": r[10], "x": r[11], "y_cm": round(r[12] * 100, 1),
                "afstand_totaal": r[13], "hoogte_cm": round(r[14] * 100, 2), "trilling_z": r[15],
                "gebeurtenis": r[16], "pitch_hoogte": r[17]})
        self.sent = len(self.view.samples)
        if now - self.status_t >= STATUS_PERIOD:
            self.status_t = now
            self.publish_status()

    def publish_status(self):
        s = self.view.status
        amps = [v for vals in s.get("stroom_a", {}).values() for v in vals]
        temps = [v for vals in s.get("temp_c", {}).values() for v in vals]
        faults = s.get("fault", {})
        msg = {"ts": int(time.time() * 1000), "rit": self.rit, "fase": s["fase"],
               "klaar": int(s["klaar"]), "estop": int(any(v & 16 for v in faults.values())) if faults else None,
               "batterij_v": s.get("batterij_v"), "stroom_max_a": max(amps) if amps else None,
               "temp_max_c": max(temps) if temps else None}
        self._publish("jaguar/status", {k: v for k, v in msg.items() if v is not None})

    def _publish(self, topic, msg):
        self.client.publish(topic, json.dumps(msg, separators=(",", ":")))

    def finish(self):
        self.view.finish()
        if self.client.is_connected():
            self.publish_status()
        time.sleep(0.3)   # laatste berichten nog versturen
        self.client.loop_stop()
        self.client.disconnect()
