"""Gesimuleerde robot: reageert op de motorcommando's, zodat rijden, draaien, pauzeren, terug naar de start,
handmatig rijden en GPS-routes zonder Jaguar te proberen zijn (nep_robot.py speelt alleen een opname af en negeert
de motoren).

Gebruik (twee terminals):
  python -m jaguar.testen.sim_robot                          luistert op 127.0.0.1:10001
  python -m jaguar.rijden.bediening --ip 127.0.0.1
  python -m jaguar.testen.sim_robot --batterij 22.4 --verbruik 3     batterij die snel leegloopt (V per minuut rijden)
  python -m jaguar.testen.sim_robot --gps-ruis 1.5 --gps-dwaal 0.3   GPS zoals buiten (springt 1,5 m, dwaalt langzaam)
  python -m jaguar.testen.sim_robot --slip 0.05                      de grond geeft 5 % mee (wielen tellen te veel)
  python -m jaguar.testen.sim_robot --geen-gps                       zonder GPS (zoals binnen zonder antenne)

Model zoals sim_boog.py: elke kant volgt het vermogen met 0,25 s vertraging, onder 60 gebeurt niets, de snelheid is
10 % lager dan K_V voorspelt en de grond slipt bij het draaien (effectieve spoorbreedte SPOOR_ECHT). De robot stuurt
wat de echte stuurt: IMU met 50 Hz (gyro en accelerometer, met ruis en trilling tijdens het rijden), encoders van
beide drivers met 5 Hz, spanning, E-Stop en de GPS ($GPRMC met 5 Hz, met controlesom). De watchdog (geen PING in
WATCHDOG_S = motoren 0) en de E-Stop (!MG / !EX) werken zoals op de robot. Geen driverpositie, geen camera.

De GPS: de robot begint op --lat/--lon en kijkt naar kompaskoers --richting (0 = noord, 90 = oost). Standaard is de
GPS bijna foutloos (ruis 2 cm), zoals in een simulator; met --gps-ruis en --gps-dwaal lijkt hij op de echte
(Garmin 18x: meters ruis, langzaam dwalen). Met --slip rijdt de robot minder ver dan de wielen tellen: dat ziet de
odometrie niet, de GPS wel.

Het is een oefenrobot, geen bewijs dat de echte hetzelfde doet. De simulator van robotics.dinno.be is een andere:
die bereik je met zijn eigen brug (jaguar_bridge.py) ook op 127.0.0.1:10001.
"""
import argparse
import json
import math
import random
import socket
import threading
import time
from collections import deque

from jaguar.analyse.rit import ACC_LSB_G, GYRO_LSB_DPS, M_PER_CNT
from jaguar.kern.robot import K_V
from jaguar.navigatie.gps import KT, Lokaal, enu_naar_robot, robot_naar_enu

SPOOR_ECHT = 1.2         # m: effectieve spoorbreedte bij draaien (de wielen zelf staan 0,52 m uit elkaar)
VERTRAGING = 0.25        # s
DODE_ZONE = 60           # vermogen
STIL_V = 0.05            # m/s: zonder vermogen en trager dan dit staat een kant stil (wrijving)
WATCHDOG_S = 0.3         # s zonder PING = motoren 0 (de robot vraagt een PING om de 0,2 s; de websimulator stopt na 0,3 s)
GYRO_BIAS = (600, 0, -580)   # counts, ongeveer zoals op de robot
START_LAT, START_LON = 50.8419000, 4.3229000
# Terrein (--terrein bestand.json): cirkels op de grond, in meters vanaf de start (x = oost, y = noord), zoals de
# ontwerper tekent. Wat ze met het robot doen:
#   stenen  het robot schudt (trilling en schokken in de IMU), rijdt 15 % trager en wordt telkens wat uit koers geduwd
#   plas    de wielen slippen: het robot komt PLAS_SLIP minder ver dan de wielen tellen en schudt niet
STENEN_TRIL, STENEN_SNELHEID, STENEN_DRAAI = 0.30, 0.85, 8.0      # g, deel van de snelheid, graden/s (ruis op de koers)
PLAS_SLIP = 0.35


def rmc(lat, lon, v, cog, fix=True, t=None):
    """Een $GPRMC-zin zoals de Garmin van het robot hem stuurt, met controlesom."""
    t = time.gmtime() if t is None else t
    frac = int(time.time() * 10) % 10
    la, lo = abs(lat), abs(lon)
    body = (f"GPRMC,{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.{frac},{'A' if fix else 'V'},"
            f"{int(la):02d}{(la - int(la)) * 60:08.5f},{'N' if lat >= 0 else 'S'},"
            f"{int(lo):03d}{(lo - int(lo)) * 60:08.5f},{'E' if lon >= 0 else 'W'},"
            f"{v / KT:06.2f},{cog % 360:05.1f},{t.tm_mday:02d}{t.tm_mon:02d}{t.tm_year % 100:02d},000.4,E")
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"${body}*{c:02X}"


class Sim:
    """De wereld: blijft bestaan over de verbindingen heen (positie, encoders, batterij), zoals de echte robot.

    x, y, h: waar het robot echt staat (m, graden), in het stelsel van zijn allereerste start (x vooruit, y links).
    Tellers voor de tests: vooruit_m en achteruit_m (echt afgelegde weg per richting) en stops (aantal keren dat
    het robot van rijden naar stilstand ging)."""

    def __init__(self, batterij=24.0, verbruik=0.0, lat=START_LAT, lon=START_LON, richting=0.0, gps=True,
                 gps_ruis=0.02, gps_dwaal=0.0, slip=0.0, snelheid=0.9, vertraging=VERTRAGING, watchdog=WATCHDOG_S,
                 achterdriver=True, gps_vertraging=0.0, terrein=None, spoor=None):
        self.conn = None
        self.lock = threading.Lock()
        self.pl = self.pr = 0             # gevraagd vermogen links / rechts
        self.vl = self.vr = 0.0           # m/s (wielsnelheid)
        self.h = 0.0                      # graden
        self.x = self.y = 0.0
        self.wl = self.wr = 0.0           # afgelegde weg per kant volgens de wielen (m)
        self.estop = True
        self.ping = time.monotonic()
        self.bat = batterij
        self.verbruik = verbruik          # V per minuut bij vermogen 250 op beide kanten
        self.running = True
        self.lokaal, self.richting = Lokaal(lat, lon), richting
        self.gps, self.gps_ruis, self.gps_dwaal = gps, gps_ruis, gps_dwaal
        self.gps_fout = [0.0, 0.0]        # langzaam dwalende fout (oost, noord) in m
        self._gps_oud = deque(maxlen=1 + round(gps_vertraging / 0.2))   # de GPS meldt waar het robot zoveel zinnen geleden was
        self.slip = slip                  # deel van de wielweg dat de grond niet volgt (0,05 = 5 % minder ver)
        self.snelheid = snelheid          # echte snelheid t.o.v. wat K_V voorspelt
        self.vertraging = vertraging
        self.watchdog = watchdog
        self.achterdriver = achterdriver  # False: alleen MM0 meldt encoders
        # terrein: [(soort, x, y, r)] in het stelsel van het robot bij de start (x vooruit, y links)
        self.terrein = [(z["soort"], *enu_naar_robot(z["x"], z["y"], richting), z["r"]) for z in (terrein or [])]
        self.grond = ""                   # waar het robot nu op rijdt: "", "stenen" of "plas"
        self.spoor = open(spoor, "w", encoding="utf-8") if spoor else None   # waar het robot echt reed
        if self.spoor:
            print("t,oost,noord,koers,grond", file=self.spoor)
        self.t = 0.0
        self.gereden = [(0.0, 0.0)]       # waar het robot echt reed (oost, noord), om de 0,2 s, voor de kaart
        self.tellers_nul()

    def tellers_nul(self):
        self.vooruit_m = self.achteruit_m = 0.0
        self.stops = 0
        self._reed = False

    def positie_ll(self):
        """Waar het robot echt staat (lat, lon), zonder GPS-fout."""
        return self.lokaal.naar_ll(*robot_naar_enu(self.x, self.y, self.richting))

    def listen(self):
        buf = b""
        while self.running:
            try:
                data = self.conn.recv(4096)
            except OSError:
                break
            if not data:
                break
            buf += data
            *lines, buf = buf.split(b"\r\n")
            for line in lines:
                self.command(line.decode("ascii", "ignore"))
        self.running = False

    def command(self, line):
        with self.lock:
            if line == "PING":
                self.ping = time.monotonic()
            elif line == "MMW !MG":
                self.estop = False
            elif line == "MMW !EX":
                self.estop = True
            elif line.startswith(("MMW !M ", "MM0 !M ")):
                try:
                    a, b = (int(v) for v in line.split()[2:4])
                except ValueError:
                    return
                self.pl, self.pr = a, -b      # robot.py stuurt "!M links -rechts"

    def send(self, line):
        self.conn.sendall((line + "\r\n").encode("ascii"))

    def _gps_zin(self, v, dt):
        """De positie zoals de GPS hem meldt: echt + dwalende fout + ruis. Koers over de grond uit de echte beweging."""
        if self.gps_dwaal:
            for k in range(2):            # random walk, teruggetrokken naar 0 (blijft binnen een paar keer gps_dwaal·√tijd)
                self.gps_fout[k] += random.gauss(0, self.gps_dwaal) * math.sqrt(dt) - self.gps_fout[k] * dt / 120
        self._gps_oud.append((self.x, self.y, self.h, v))
        x, y, h, v = self._gps_oud[0]
        oost, noord = robot_naar_enu(x, y, self.richting)
        lat, lon = self.lokaal.naar_ll(oost + self.gps_fout[0] + random.gauss(0, self.gps_ruis),
                                       noord + self.gps_fout[1] + random.gauss(0, self.gps_ruis))
        cog = self.richting - h + (180 if v < 0 else 0) + (random.gauss(0, 20) if abs(v) < 0.3 else random.gauss(0, 2))
        return rmc(lat, lon, abs(v), cog)

    def run(self, conn, log=True):
        self.conn, self.running, self.estop, self.pl, self.pr = conn, True, True, 0, 0
        self.ping = time.monotonic()
        threading.Thread(target=self.listen, daemon=True).start()
        dt, seq, n = 0.02, 0, 0
        v_oud = 0.0
        t_next = time.perf_counter()
        try:
            while self.running:
                with self.lock:
                    vrij = not self.estop and time.monotonic() - self.ping < self.watchdog
                    pl, pr = (self.pl, self.pr) if vrij else (0, 0)
                tl = 0.0 if abs(pl) < DODE_ZONE else K_V * self.snelheid * pl
                tr = 0.0 if abs(pr) < DODE_ZONE else K_V * self.snelheid * pr
                k = min(1.0, dt / self.vertraging)
                self.vl += (tl - self.vl) * k
                self.vr += (tr - self.vr) * k
                # Wrijving: zonder vermogen staat een kant echt stil (anders rolt hij exponentieel en eindeloos uit, en
                # meet de rustmeting van de volgende rit een draaiend robot)
                if tl == 0 and abs(self.vl) < STIL_V:
                    self.vl = 0.0
                if tr == 0 and abs(self.vr) < STIL_V:
                    self.vr = 0.0
                v_wiel = (self.vl + self.vr) / 2
                self.grond = next((soort for soort, zx, zy, zr in self.terrein if math.hypot(self.x - zx, self.y - zy) <= zr), "")
                v = v_wiel * (1 - self.slip)                  # wat de grond ervan overlaat
                rate = math.degrees((self.vr - self.vl) / SPOOR_ECHT)
                if self.grond == "plas":
                    v *= 1 - PLAS_SLIP
                elif self.grond == "stenen" and abs(v_wiel) > 0.02:
                    v *= STENEN_SNELHEID
                    rate += random.gauss(0, STENEN_DRAAI)
                self.t += dt
                self.h += rate * dt
                self.x += v * math.cos(math.radians(self.h)) * dt
                self.y += v * math.sin(math.radians(self.h)) * dt
                self.wl += self.vl * dt
                self.wr += self.vr * dt
                if v >= 0:
                    self.vooruit_m += v * dt
                else:
                    self.achteruit_m -= v * dt
                if abs(v) > 0.10:                 # echt aan het rijden (niet het trillen rond de dode zone bij het optrekken)
                    self._reed = True
                elif abs(v) < 0.02 and self._reed:
                    self.stops, self._reed = self.stops + 1, False
                self.bat -= self.verbruik / 60 * (abs(pl) + abs(pr)) / 500 * dt
                acc = (v - v_oud) / dt
                v_oud = v
                bewegen = abs(self.vl) + abs(self.vr) > 0.02
                tril = 0.08 if bewegen else 0.004
                schok = 0.0
                if bewegen and self.grond == "stenen":
                    tril, schok = STENEN_TRIL, 25.0           # schokken in de gyro (counts) om de x- en y-as
                elif bewegen and self.grond == "plas":
                    tril = 0.03
                g = [GYRO_BIAS[0] + random.gauss(0, 1 + schok), GYRO_BIAS[1] + random.gauss(0, 1 + schok),
                     GYRO_BIAS[2] + rate * GYRO_LSB_DPS + random.gauss(0, 1)]
                a = [acc / 9.81 * ACC_LSB_G + random.gauss(0, 1), random.gauss(0, 1),
                     ACC_LSB_G * (1 + random.gauss(0, tril))]
                self.send(f"#{seq},YAW,{self.h % 360:.2f},GYRO,{','.join(str(round(x)) for x in g)},"
                          f"ACC,{','.join(str(round(x)) for x in a)},ADC,0,0,0,0,")
                seq = (seq + 1) % 256
                n += 1
                if n % 10 == 0 and bewegen:
                    self.gereden.append(tuple(round(w, 3) for w in robot_naar_enu(self.x, self.y, self.richting)))
                    del self.gereden[:-6000]          # 20 minuten rijden
                if n % 10 == 0 and self.spoor:
                    oost, noord = robot_naar_enu(self.x, self.y, self.richting)
                    print(f"{self.t:.2f},{oost:.4f},{noord:.4f},{self.richting - self.h:.2f},{self.grond}", file=self.spoor, flush=True)
                if n % 10 == 0:                   # 5 Hz: encoders van beide drivers, spanning, GPS
                    cl, cr = round(self.wl / M_PER_CNT), round(self.wr / M_PER_CNT)
                    self.send(f"MM0 C={cl}:{-cr}")
                    if self.achterdriver:
                        self.send(f"MM1 C={cl}:{-cr}")
                    sag = 0.0015 * (abs(pl) + abs(pr)) / 2
                    for b in ("MM0", "MM1"):
                        self.send(f"{b} V=120:{round((self.bat - sag + random.uniform(-0.4, 0.4)) * 10)}:4900")
                    if self.gps:
                        self.send(self._gps_zin(v, 10 * dt))
                if n % 50 == 0:
                    for b in ("MM0", "MM1"):
                        self.send(f"{b} FF={16 if self.estop else 0}")
                t_next += dt
                time.sleep(max(0.0, t_next - time.perf_counter()))
        except OSError:
            pass
        self.running = False
        if log:
            lat, lon = self.positie_ll()
            print(f"{time.strftime('%H:%M:%S')}  verbinding dicht; robot op x {self.x:+.2f} m, y {self.y:+.2f} m, "
                  f"koers {self.h:+.1f}° ({lat:.7f}, {lon:.7f}, kompaskoers {(self.richting - self.h) % 360:.1f}°), "
                  f"batterij {self.bat:.1f} V")


def main():
    ap = argparse.ArgumentParser(description="Gesimuleerde robot op 127.0.0.1:10001 die op de motoren reageert")
    ap.add_argument("--port", type=int, default=10001)
    ap.add_argument("--batterij", type=float, default=24.0, help="spanning bij het begin (V)")
    ap.add_argument("--verbruik", type=float, default=0.0, help="V per minuut rijden bij vermogen 250 (standaard 0)")
    ap.add_argument("--lat", type=float, default=START_LAT, help="breedtegraad van de start")
    ap.add_argument("--lon", type=float, default=START_LON, help="lengtegraad van de start")
    ap.add_argument("--richting", type=float, default=0.0, help="kompaskoers bij de start (0 = noord, 90 = oost)")
    ap.add_argument("--geen-gps", action="store_true", help="geen $GPRMC sturen")
    ap.add_argument("--gps-ruis", type=float, default=0.02, help="ruis van de GPS-positie in m (standaard 0,02; echte GPS: 1 tot 3)")
    ap.add_argument("--gps-dwaal", type=float, default=0.0, help="langzaam dwalen van de GPS in m per wortel-seconde (echte GPS buiten: ca. 0,2)")
    ap.add_argument("--gps-vertraging", type=float, default=0.0, help="s: de GPS meldt waar het robot zoveel seconden geleden was (echte GPS: tot 1 s)")
    ap.add_argument("--terrein", help="JSON met stenen en plassen op de grond (zie testdata/terrein_demo.json)")
    ap.add_argument("--spoor", help="CSV waarin komt waar het robot echt reed (t,oost,noord,koers,grond; 5 keer per seconde)")
    ap.add_argument("--slip", type=float, default=0.0, help="deel van de wielweg dat de grond niet volgt (0,05 = 5 %% minder ver dan de wielen tellen)")
    ap.add_argument("--snelheid", type=float, default=0.9, help="echte snelheid t.o.v. wat het vermogen voorspelt (standaard 0,9)")
    ap.add_argument("--vertraging", type=float, default=VERTRAGING, help="s: hoe traag de motoren het vermogen volgen")
    ap.add_argument("--watchdog", type=float, default=WATCHDOG_S, help="s zonder PING voor de motoren stoppen")
    args = ap.parse_args()
    terrein = json.loads(open(args.terrein, encoding="utf-8").read())["zones"] if args.terrein else None
    srv = socket.create_server(("127.0.0.1", args.port))
    sim = Sim(args.batterij, args.verbruik, args.lat, args.lon, args.richting, not args.geen_gps, args.gps_ruis,
              args.gps_dwaal, args.slip, args.snelheid, args.vertraging, args.watchdog, gps_vertraging=args.gps_vertraging,
              terrein=terrein, spoor=args.spoor)
    print(f"Gesimuleerde robot klaar op 127.0.0.1:{args.port} (start {args.lat:.7f}, {args.lon:.7f}, kijkt naar "
          f"{args.richting:.0f}°, GPS {'uit' if args.geen_gps else f'ruis {args.gps_ruis:g} m'}). Ctrl+C = stoppen.")
    serve(srv, sim)


def serve(srv, sim, log=True):
    """Eén verbinding tegelijk, zoals de robot; ook gebruikt door de tests."""
    while True:
        try:
            conn, _ = srv.accept()
        except OSError:
            return                        # de test sloot de server
        if log:
            print(f"{time.strftime('%H:%M:%S')}  script verbonden")
        with conn:
            sim.run(conn, log)


if __name__ == "__main__":
    main()
