"""Brug naar de simulator van robotics.dinno.be zonder browser: de "Training API" op de eigen pc.

De rijscripts praten met een Jaguar over TCP (tekstregels). Deze brug doet zich voor als die Jaguar op
127.0.0.1:10001 en geeft alles door aan de Training API (robotSimulatorAPI, `node server.mjs`), die de simulator
van de website zonder venster draait. Zo rijden alle scripts hier tegen de websimulator met alleen `--ip 127.0.0.1`.

Drie terminals:
  1. in de map robotSimulatorAPI (eerst eenmalig: Node.js 22 en Chrome; de sleutel staat bij je account op de site):
        $env:ROBOTSIM_KEY = "rsk_..."          (PowerShell; zet de sleutel nooit in een bestand van dit project)
        node server.mjs                          (met --show zie je de simulator in een Chrome-venster)
  2. python -m jaguar.testen.api_brug
  3. $env:JAGUAR_WIELSTRAAL = "0.135"          (de websimulator rekent met een wiel van 0,135 m)
     python -m jaguar.rijden.rechtdoor --ip 127.0.0.1 --distance 5
     python -m jaguar.rijden.bediening --ip 127.0.0.1
     python -m jaguar.rijden.gps_route --ip 127.0.0.1 --vanaf-hier <lat,lon>

Hoe het werkt: de simulatie staat stil tot iemand een stap vraagt. De brug vraagt 50 stappen van 0,02 s per seconde
(echte tijd), stuurt de regels van het script mee ({"tcp": [...]}) en maakt van elke waarneming de regels die het
robot stuurt: de IMU (50 Hz), de encoders van beide drivers, de spanning en de GPS (5 Hz), de E-Stop (1 Hz).
Antwoorden op vragen (?V, ~KP …) komen terug zoals de simulator ze geeft. Bij het sluiten van elke verbinding
toont de brug waar het robot in de simulatie ECHT staat en hoe ver het reed: daarmee controleer je de rit.

Niet hetzelfde als de pagina in de browser met jaguar_bridge.py: dezelfde simulator, maar de regels worden hier
uit de waarneming opgebouwd (de GPS-zin dus ook: positie, snelheid en koers zoals de simulator ze meldt).
"""
import argparse
import json
import math
import socket
import threading
import time
import urllib.request

from jaguar.testen.sim_robot import rmc

DT = 0.02


class Api:
    """De paar aanroepen van de Training API die de brug nodig heeft (alleen de standaardbibliotheek)."""

    def __init__(self, url):
        self.url = url.rstrip("/") + "/api/"

    def _vraag(self, pad, body=None, methode=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + pad, data=data, method=methode or ("POST" if data is not None else "GET"),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read() or b"{}")

    def maak(self, wereld):
        return self._vraag("envs", {"robot": "jaguar", "lane": "free", "options": {"world": wereld}})

    def stap(self, eid, regels):
        return self._vraag(f"envs/{eid}/step", {"action": {"tcp": regels} if regels else {}, "dt": DT})

    def sluit(self, eid):
        self._vraag(f"envs/{eid}", methode="DELETE")


class Brug:
    def __init__(self, api, wereld, plaats=None, spoor=None):
        self.api = api
        self.spoor = open(spoor, "w", encoding="utf-8") if spoor else None   # waar het robot echt reed: t,x,z,koers
        if self.spoor:
            print("t,x,z,koers", file=self.spoor)
        eerste = api.maak(wereld)
        self.eid, self.obs = eerste["id"], eerste["obs"]
        if plaats:                                    # het robot ergens neerzetten (de simulator kijkt niet of daar iets staat)
            x, z, koers = plaats
            self.obs = api._vraag(f"envs/{self.eid}/step", {"action": {"place": {"x": x, "z": z, "yaw": math.radians(koers)}}, "dt": DT})["obs"]
        self.seq = self.n = 0
        self.beeld_pad = None                 # na elke rit een beeld van de simulator bewaren
        self.lock = threading.Lock()
        self.wachtrij = []
        self.running = True

    def beeld(self, pad, view="scene", breedte=960):
        """Een beeld van de simulator (JPEG) bewaren."""
        with urllib.request.urlopen(f"{self.api.url}envs/{self.eid}/frame?view={view}&width={breedte}", timeout=30) as r:
            open(pad, "wb").write(r.read())

    def pose(self):
        p = self.obs["pose"]
        return p["x"], p["z"], math.degrees(p["yaw"])

    def luister(self, conn):
        buf = b""
        while self.running:
            try:
                data = conn.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            *regels, buf = buf.split(b"\r\n")
            with self.lock:
                self.wachtrij += [r.decode("ascii", "ignore") for r in regels if r]
        self.running = False

    def regels_uit(self, r):
        """De regels die het robot na deze stap stuurt, uit de waarneming."""
        uit = []
        for e in r.get("events", []):                 # antwoorden op vragen en instellingen
            if e.get("type") == "tcp" and e.get("reply") and not e["reply"].startswith("("):
                uit += [x for x in str(e["reply"]).splitlines() if x]
        ex = r["obs"]["extra"]
        imu = ex.get("imu")
        if imu:
            raw = imu["raw"]
            # De gyro-Y-as van de websimulator draait andersom dan die van het echte robot (daar: neus omhoog =
            # negatieve gyro Y, zie PITCH_TEKEN in odometrie.py). Zonder omkeren telt de odometrie op een heuvel de
            # helling dubbel in plaats van ze weg te rekenen en verwerpt ze de wielen (rit 121810: 40 cm kwijt).
            gyro = [raw["gyro"][0], -raw["gyro"][1], raw["gyro"][2]]
            uit.append(f"#{self.seq},YAW,{imu.get('yaw', 0):.3f},GYRO,{','.join(str(int(v)) for v in gyro)},"
                       f"ACC,{','.join(str(int(v)) for v in raw['acc'])},COMP,{','.join(str(int(v)) for v in raw['mag'])},ADC,0")
            self.seq = (self.seq + 1) % 256
        self.n += 1
        if self.n % 10 == 0:
            enc = ex["encoders"]
            uit.append(f"MM0 C={enc['FL']}:{enc['FR']}")
            uit.append(f"MM1 C={enc['RL']}:{enc['RR']}")
            if ex.get("battery_v") is not None:
                for b in ("MM0", "MM1"):
                    uit.append(f"{b} V=120:{round(ex['battery_v'] * 10)}:5000")
            gps = ex.get("gps")
            if gps and gps.get("lat") is not None:
                uit.append(rmc(gps["lat"], gps["lon"], (gps.get("speed_kn") or 0) * 0.514444, gps.get("course") or 0,
                               fix=bool(gps.get("fix"))))
        if self.n % 50 == 0:
            for b in ("MM0", "MM1"):
                uit.append(f"{b} FF={16 if ex.get('estop') else 0}")
        return uit

    def bedien(self, conn):
        self.running = True
        with self.lock:
            self.wachtrij = []
        threading.Thread(target=self.luister, args=(conn,), daemon=True).start()
        x0, z0, h0 = self.pose()
        t_volgende = time.perf_counter()
        try:
            while self.running:
                with self.lock:
                    regels, self.wachtrij = self.wachtrij, []
                r = self.api.stap(self.eid, regels)
                self.obs = r["obs"]
                if self.spoor and self.n % 10 == 0:
                    x, z, h = self.pose()
                    print(f"{self.obs['t']:.2f},{x:.4f},{z:.4f},{h:.2f}", file=self.spoor, flush=True)
                uit = self.regels_uit(r)
                if uit:
                    conn.sendall(("\r\n".join(uit) + "\r\n").encode("ascii"))
                t_volgende += DT
                time.sleep(max(0.0, t_volgende - time.perf_counter()))
        except OSError:
            pass
        self.running = False
        for _ in range(100):                           # zonder script: de watchdog stopt de motoren, het robot rolt uit
            self.obs = self.api.stap(self.eid, [])["obs"]
        x, z, h = self.pose()
        if self.beeld_pad:
            try:
                self.beeld(self.beeld_pad)
            except OSError as e:
                print(f"beeld niet bewaard: {e}")
        gps = self.obs["extra"].get("gps") or {}
        print(f"{time.strftime('%H:%M:%S')}  verbinding dicht. ECHT in de simulatie: {math.hypot(x - x0, z - z0):.3f} m van waar deze rit begon "
              f"(dx {x - x0:+.3f}, dz {z - z0:+.3f} m), koers {(h - h0 + 180) % 360 - 180:+.1f}° gedraaid; "
              f"stand x {x:+.3f}, z {z:+.3f}, koers {h:.1f}°"
              + (f"; GPS {gps['lat']:.7f}, {gps['lon']:.7f}" if gps.get("lat") is not None else ""), flush=True)


def main():
    ap = argparse.ArgumentParser(description="De websimulator (Training API) als Jaguar op 127.0.0.1:10001")
    ap.add_argument("--api", default="http://localhost:8780", help="adres van node server.mjs")
    ap.add_argument("--port", type=int, default=10001)
    ap.add_argument("--wereld", default="park", help="omgeving in de simulator (park, course, …)")
    ap.add_argument("--plaats", help="x,z,koers: het robot daar neerzetten (m en graden; koers 0 = oost, 90 = noord). "
                                     "In het park is x 8..20, z -6..6 een vlak grasveld zonder obstakels: bv. --plaats 9,4,0")
    ap.add_argument("--spoor", help="CSV waarin komt waar het robot in de simulatie echt reed (t,x,z,koers; 5 keer per seconde)")
    ap.add_argument("--beeld", help="JPEG van de simulator, bewaard na elke rit")
    args = ap.parse_args()
    try:
        plaats = tuple(float(v) for v in args.plaats.split(",")) if args.plaats else None
        if plaats and len(plaats) != 3:
            ap.error("--plaats: x,z,koers")
        brug = Brug(Api(args.api), args.wereld, plaats, args.spoor)
        brug.beeld_pad = args.beeld
    except OSError as e:
        raise SystemExit(f"De Training API antwoordt niet op {args.api} ({e}). Start eerst  node server.mjs  in robotSimulatorAPI.")
    srv = socket.create_server(("127.0.0.1", args.port))
    x, z, h = brug.pose()
    print(f"Websimulator klaar als Jaguar op 127.0.0.1:{args.port} (robot op x {x:+.2f}, z {z:+.2f}, koers {h:.0f}°). Ctrl+C = stoppen.", flush=True)
    try:
        while True:
            conn, _ = srv.accept()
            print(f"{time.strftime('%H:%M:%S')}  script verbonden", flush=True)
            with conn:
                brug.bedien(conn)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            brug.api.sluit(brug.eid)
        except OSError:
            pass


if __name__ == "__main__":
    main()
