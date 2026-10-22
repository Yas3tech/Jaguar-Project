"""Live weergave tijdens het rijden: lokale webserver + browserpagina (live.html).

De Jaguar-reader geeft elke ontvangen regel door aan LiveView.feed(); dit verandert
niets aan het rijden of aan de CSV. De browser krijgt de gegevens via Server-Sent
Events op /stream. Een nieuwe of herladen pagina krijgt eerst de hele rit tot nu toe.

Live berekeningen zijn een benadering van analyse_rit.py:
  - gyro-bias = mediaan van de rustfase (robuust tegen de oude berichten bij verbinden)
  - koers = geïntegreerde gyro-Z vanaf het einde van de rustfase, dt uit het IMU-volgnummer
  - afstand/snelheid/encoderkoers uit de encoders van beide motordrivers, nulpunt = einde rustfase
  - hoogte = som van sin(pitch) x afgelegde weg; pitch in stilstand uit de accelerometer, tijdens
    het rijden uit de gyro-Y vanaf de laatste rusthoek (zoals analyse_rit.py)
  - gebeurtenissen zoals in het dashboard: vastgelopen, schuin (> 7° minstens 1 s), vrije val (< 0,35 g)
"""
import json
import math
import os
import socket
import statistics as st
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from analyse_rit import ACC_LSB_G, GYRO_LSB_DPS, M_PER_CNT, WHEEL_DIS

PAGE = Path(__file__).parent / "live.html"
DASHBOARD = Path(__file__).parent / "dashboard.html"   # geschreven door analyse_rit.py
OPTIONS_FILE = Path(__file__).parent / "live_opties.json"   # bewaard tussen de ritten
PANELS = ("koers", "baan", "snelheid", "pitchroll", "gyro", "versnelling")
DEFAULT_OPTIONS = {"auto_analyse": False, "panelen": {p: True for p in PANELS}}
IMU_DT = 0.02             # IMU stuurt 50 Hz
SPEED_CYCLES = 3          # encodersnelheid over 3 pollcycli (~0.6 s)
ANGLE_ALPHA = 0.1         # laagdoorlaat pitch/roll (~0.2 s bij 50 Hz)
GYRO_Y_SIGN = -1          # neus omhoog = negatieve gy (Y-as naar links); bevestigd op de ritten van 2/10
VIB_SAMPLES = 10          # trillings-RMS over 0,2 s, zoals analyse_rit.py
# Drempels zoals analyse_rit.py
TILT_DEG, TILT_MIN_S = 7.0, 1.0
FREEFALL_G, FREEFALL_SAMPLES = 0.35, 3
STALL_FRAC, STALL_MIN_S = 0.15, 1.0
SPIN_FRAC, SPIN_MIN_S = 1.6, 0.6
SIDE_FRAC, SIDE_MIN_S = 0.6, 1.0
K_V = 0.0031              # m/s per vermogenseenheid (fase1_heen_en_terug.K_V): verwachte snelheid


class LiveView:
    def __init__(self, name=""):
        self.run_id = f"{time.time():.3f}"
        self.name = name
        self.samples = []     # één compacte rij per IMU-sample, alleen aangevuld
        self.status = {"fase": "verbinden", "klaar": False}
        self.rest_gz = []
        self.bias = None
        self.heading = 0.0
        self.prev_seq = None
        self.pitch = self.roll = None
        self.enc = {}         # board -> (links, rechts) in pulsen, rechts al omgedraaid
        self.enc0 = {}
        self.dist = 0.0
        self.enc_heading = 0.0
        self.speed = 0.0
        self.dist_hist = deque(maxlen=SPEED_CYCLES + 1)
        self.enc_t = None
        self.enc_gaps = deque(maxlen=20)
        self.path_dist = 0.0
        self.x = self.y = 0.0
        self.travelled = 0.0
        self.height = 0.0
        self.pitch_h = None   # pitch voor de hoogte (gyro tijdens rijden, accelerometer in stilstand)
        self.pitch0 = self.roll0 = None
        self.az_hist = deque(maxlen=VIB_SAMPLES)
        self.since = {}       # gebeurtenis -> sinds wanneer de voorwaarde geldt
        self.freefall_n = 0
        self.max_power = 0
        self.side_hist = deque(maxlen=SPEED_CYCLES + 1)   # (links, rechts) afstand per pollcyclus
        self.v_left = self.v_right = 0.0
        # Analyseknop: CSV van deze rit, pas analyseerbaar als het rijscript hem gesloten heeft.
        self.csv_path = None
        self.csv_ready = False
        self.dashboard = False
        self.analyse_lock = threading.Lock()

    def run_analysis(self):
        """analyse_rit.py op de CSV van deze rit; geeft (HTTP-status, antwoord) terug."""
        if self.csv_path is None:
            return 409, {"fout": "Deze rit heeft geen CSV (--no-csv)."}
        if not self.csv_ready:
            return 409, {"fout": "De rit loopt nog."}
        with self.analyse_lock:
            import analyse_rit
            try:
                result, out_html = analyse_rit.run(self.csv_path)
            except Exception as e:  # noqa: BLE001 - de fout hoort in de browser, de server moet blijven draaien
                return 500, {"fout": f"Analyse mislukt: {type(e).__name__}: {e}"}
            self.dashboard = out_html is not None
            return 200, {"rit": result["summary"]["rit"], "dashboard": self.dashboard}

    def feed(self, t, phase, power, line):
        self.status["fase"] = phase
        try:
            if line.startswith("#"):
                self._imu(t, phase, power, line.split(","))
            elif line[:4] in ("MM0 ", "MM1 ") and "=" in line:
                board, kv = line.split(" ", 1)
                key, val = kv.split("=", 1)
                self._motor(t, phase, board, key, [int(v) for v in val.split(":")])
        except ValueError:
            pass              # kapotte regel (twee berichten aan elkaar): staat wel in de CSV

    def _imu(self, t, phase, power, p):
        if len(p) != 17 or p[3] != "GYRO" or p[7] != "ACC":
            return
        seq = int(p[0][1:])
        g = [int(v) for v in p[4:7]]
        a = [int(v) / ACC_LSB_G for v in p[8:11]]
        if phase == "rust":
            self.rest_gz.append(g)
        elif self.bias is None and self.rest_gz:
            self.bias = [st.median(s[i] for s in self.rest_gz) for i in range(3)]
        bias = self.bias or ([st.median(s[i] for s in self.rest_gz[-100:]) for i in range(3)] if self.rest_gz else [0, 0, 0])
        dps = [(g[i] - bias[i]) / GYRO_LSB_DPS for i in range(3)]

        step = (seq - self.prev_seq) % 256 if self.prev_seq is not None else 1
        self.prev_seq = seq
        dt = IMU_DT * (step if 0 < step < 10 else 1)

        pitch = math.degrees(math.atan2(a[0], a[2]))
        roll = math.degrees(math.atan2(a[1], a[2]))
        if self.pitch is None:
            self.pitch, self.roll = pitch, roll
        self.pitch += ANGLE_ALPHA * (pitch - self.pitch)
        self.roll += ANGLE_ALPHA * (roll - self.roll)
        still = power == 0 and abs(self.speed) < 0.02
        if self.pitch_h is None or still:
            self.pitch_h = self.pitch
        else:
            self.pitch_h += GYRO_Y_SIGN * dps[1] * dt
        if phase == "rust" or self.pitch0 is None:
            self.pitch0, self.roll0 = self.pitch, self.roll   # rusthoek = vlak waarop de robot start

        if self.bias is not None:
            self.heading += dps[2] * dt
            ds = self.dist - self.path_dist
            self.path_dist = self.dist
            self.x += ds * math.cos(math.radians(self.heading))
            self.y += ds * math.sin(math.radians(self.heading))
            self.travelled += abs(ds)
            self.height += ds * math.sin(math.radians(self.pitch_h - self.pitch0))

        self.az_hist.append(a[2])
        mean_az = sum(self.az_hist) / len(self.az_hist)
        vib = math.sqrt(sum((v - mean_az) ** 2 for v in self.az_hist) / len(self.az_hist))
        event = self._event(t, phase, power, a)

        r = lambda v, d=3: round(v, d)
        self.samples.append([r(t), phase, power, [r(v, 2) for v in dps], [r(v) for v in a],
                             r(self.pitch, 2), r(self.roll, 2), r(self.heading, 2), r(self.enc_heading, 2),
                             r(self.speed), r(self.dist), r(self.x), r(self.y),
                             r(self.travelled), r(self.height, 4), r(vib, 4), event, r(self.pitch_h, 2)])

    def _event(self, t, phase, power, a):
        """Gebeurtenis op dit moment, met dezelfde drempels als analyse_rit.py."""
        self.freefall_n = self.freefall_n + 1 if math.sqrt(sum(v * v for v in a)) < FREEFALL_G else 0
        if self.freefall_n >= FREEFALL_SAMPLES:
            return "vrije val"
        self.max_power = max(self.max_power, abs(power))
        driving = power != 0 and abs(power) >= 0.5 * self.max_power
        expect = K_V * abs(power)
        fast = max(abs(self.v_left), abs(self.v_right))
        checks = [
            ("schuin", TILT_MIN_S, phase != "rust" and (abs(self.pitch_h - self.pitch0) > TILT_DEG
                                                        or abs(self.roll - self.roll0) > TILT_DEG)),
            ("vastgelopen", STALL_MIN_S, driving and abs(self.speed) < STALL_FRAC * expect),
            ("wielen vrij", SPIN_MIN_S, driving and abs(self.speed) > SPIN_FRAC * expect),
            ("één kant", SIDE_MIN_S, driving and fast > 0.05
                                     and abs(self.v_left - self.v_right) > SIDE_FRAC * fast),
        ]
        found = "geen"
        for kind, min_s, cond in checks:
            self.since[kind] = self.since.get(kind) or t if cond else None
            if cond and found == "geen" and t - self.since[kind] >= min_s:
                found = kind
        return found

    def _motor(self, t, phase, board, key, vals):
        s = self.status
        if key == "C" and len(vals) == 2:
            self.enc[board] = (vals[0], -vals[1])
            if phase == "rust" or board not in self.enc0:
                self.enc0[board] = self.enc[board]   # nulpunt = laatste stand in rust (niet de oude berichten)
            if len(self.enc) == 2:
                left = st.mean(self.enc[b][0] - self.enc0[b][0] for b in self.enc) * M_PER_CNT
                right = st.mean(self.enc[b][1] - self.enc0[b][1] for b in self.enc) * M_PER_CNT
                self.dist = (left + right) / 2
                self.enc_heading = math.degrees((right - left) / WHEEL_DIS)
            if board == "MM0":
                # Wifi bundelt soms meerdere encoderberichten; elk bericht is wel één pollcyclus.
                # Snelheid dus per cyclus rekenen, met de cyclusduur gemeten tijdens de rust.
                if self.enc_t is not None and t - self.enc_t > 0.1 and phase == "rust":
                    self.enc_gaps.append(t - self.enc_t)
                self.enc_t = t
                cycle = st.median(self.enc_gaps) if self.enc_gaps else 0.2
                self.dist_hist.append(self.dist)
                if len(self.dist_hist) > 1:
                    self.speed = (self.dist_hist[-1] - self.dist_hist[0]) / ((len(self.dist_hist) - 1) * cycle)
                if len(self.enc) == 2:
                    self.side_hist.append((left, right))
                    if len(self.side_hist) > 1:
                        span = (len(self.side_hist) - 1) * cycle
                        self.v_left = (self.side_hist[-1][0] - self.side_hist[0][0]) / span
                        self.v_right = (self.side_hist[-1][1] - self.side_hist[0][1]) / span
        elif key == "V" and len(vals) > 1:
            s["batterij_v"] = vals[1] / 10
        elif key == "A":
            s.setdefault("stroom_a", {})[board] = [abs(v) / 10 for v in vals]
        elif key == "T":
            s.setdefault("temp_c", {})[board] = vals
        elif key == "FF":
            s.setdefault("fault", {})[board] = vals[0]

    def finish(self):
        self.status["klaar"] = True

    def message(self, start):
        return {"run": self.run_id, "naam": self.name, "start": start,
                "rijen": self.samples[start:], "status": self.status, "bias": self.bias,
                "csv": self.csv_path is not None, "csv_klaar": self.csv_ready, "dashboard": self.dashboard,
                "opties": OPTIONS}


def clean_options(raw):
    """Alleen gekende sleutels met een geldige waarde; de rest blijft op de standaardwaarde."""
    o = json.loads(json.dumps(DEFAULT_OPTIONS))
    if not isinstance(raw, dict):
        return o
    for key in ("auto_analyse",):
        if isinstance(raw.get(key), bool):
            o[key] = raw[key]
    panels = raw.get("panelen")
    if isinstance(panels, dict):
        for p in PANELS:
            if isinstance(panels.get(p), bool):
                o["panelen"][p] = panels[p]
    return o


def load_options():
    try:
        return clean_options(json.loads(OPTIONS_FILE.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        return clean_options(None)


def save_options(raw):
    global OPTIONS
    OPTIONS = clean_options(raw)
    OPTIONS_FILE.write_text(json.dumps(OPTIONS, indent=2), encoding="utf-8")
    return OPTIONS


OPTIONS = load_options()


class Hub:
    """Wat de webserver toont: de LiveView van de huidige rit. Met bediening.py (control) wisselt
    die per rit en kan de pagina ritten starten en stoppen."""

    def __init__(self, view, control=None):
        self.view = view
        self.control = control


def _handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _local(self):
            """Verzoek van deze laptop zelf (niet van een ander apparaat op de wifi)."""
            return self.client_address[0] in ("127.0.0.1", "::1")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/stream":
                return self._stream()
            if path == "/options.json":
                return self._json(200, OPTIONS)
            if path == "/dashboard":
                if not DASHBOARD.exists():
                    return self.send_error(404, "Nog geen dashboard: klik eerst op Analyse.")
                return self._send(200, "text/html; charset=utf-8", DASHBOARD.read_bytes())
            self._send(200, "text/html; charset=utf-8", PAGE.read_bytes())   # ook /options: opent het opties-venster

        def do_POST(self):
            if self.path == "/analyse":
                return self._json(*hub.view.run_analysis())
            if self.path == "/stop":
                if hub.control is None:
                    return self._json(404, {"fout": "Deze pagina bestuurt de robot niet."})
                return self._json(*hub.control.stop())     # STOP mag iedereen die meekijkt
            if self.path not in ("/options", "/rit"):
                return self.send_error(404)
            if not self._local():   # opties en ritten starten bepalen hoe veilig de robot rijdt
                return self._json(403, {"fout": "Dit kan alleen op de laptop die met de robot verbonden is."})
            try:
                length = int(self.headers.get("Content-Length", 0))
                raw = json.loads(self.rfile.read(min(length, 10_000)))
            except ValueError:
                return self._json(400, {"fout": "geen geldige JSON"})
            if self.path == "/options":
                return self._json(200, save_options(raw))
            if hub.control is None:
                return self._json(404, {"fout": "Ritten starten kan alleen met python bediening.py."})
            return self._json(*hub.control.start(raw))

        def _json(self, code, answer):
            self._send(code, "application/json", json.dumps(answer).encode())

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            sent, current = 0, None
            local = self._local()
            try:
                while True:
                    view = hub.view
                    if view is not current:   # nieuwe rit (bediening.py): de pagina begint opnieuw
                        current, sent = view, 0
                    n = len(view.samples)
                    msg = view.message(sent)
                    msg["rijen"] = msg["rijen"][:n - sent]
                    if hub.control is not None:
                        msg["bediening"] = dict(hub.control.state, mag_starten=local)
                    self.wfile.write(f"data: {json.dumps(msg, separators=(',', ':'))}\n\n".encode())
                    self.wfile.flush()
                    sent = n
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
    return Handler


def lan_addresses():
    """IPv4-adressen van deze pc waarop andere apparaten hem kunnen bereiken (bv. op de DriJaguar-wifi)."""
    try:
        addrs = {a[4][0] for a in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}
    except OSError:
        addrs = set()
    return sorted(a for a in addrs if not a.startswith(("127.", "169.254.")))


class Server(ThreadingHTTPServer):
    # Op Windows laat SO_REUSEADDR een tweede script stil op dezelfde poort starten; de browser
    # krijgt dan de oude rit. Zonder die optie meldt het tweede script dat de poort bezet is.
    allow_reuse_address = os.name != "nt"

    def server_bind(self):
        if os.name == "nt":   # ook 127.0.0.1 naast 0.0.0.0 op dezelfde poort weigeren
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def start(name="", port=8765, open_browser=True, network=True, csv_path=None):
    """Start de webserver in de achtergrond en geef de LiveView terug.
    network=True: ook bereikbaar voor andere apparaten op hetzelfde netwerk (meekijken en de analyse starten).
    csv_path: CSV van deze rit, voor de knop Analyse."""
    view = LiveView(name)
    view.csv_path = csv_path
    serve(Hub(view), port, open_browser, network)
    return view


def serve(hub, port=8765, open_browser=True, network=True):
    """Webserver voor `hub` in een achtergronddraad."""
    try:
        server = Server(("0.0.0.0" if network else "127.0.0.1", port), _handler(hub))
    except OSError as e:
        raise SystemExit(f"Poort {port} is bezet ({e.strerror}): er draait nog een vorig script. "
                         "Druk daar op Enter of sluit die terminal.") from None
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{port}/"
    print(f"Live weergave: {url}")
    if network:
        for ip in lan_addresses():
            print(f"  meekijken vanaf een ander apparaat op hetzelfde netwerk: http://{ip}:{port}/")
    if open_browser:
        webbrowser.open(url)


if __name__ == "__main__":
    import bediening   # zelfde pagina, met rijden vanuit de browser
    bediening.main()
