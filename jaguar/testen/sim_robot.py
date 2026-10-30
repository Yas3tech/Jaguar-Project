"""Gesimuleerde robot: reageert op de motorcommando's, zodat rijden, draaien, pauzeren, terug naar de start en
handmatig rijden zonder Jaguar te proberen zijn (nep_robot.py speelt alleen een opname af en negeert de motoren).

Gebruik (twee terminals):
  python -m jaguar.testen.sim_robot                          luistert op 127.0.0.1:10001
  python -m jaguar.rijden.bediening --ip 127.0.0.1
  python -m jaguar.testen.sim_robot --batterij 22.4 --verbruik 3     batterij die snel leegloopt (V per minuut rijden)

Model zoals sim_boog.py: elke kant volgt het vermogen met 0,25 s vertraging, onder 60 gebeurt niets, de snelheid is
10 % lager dan K_V voorspelt en de grond slipt bij het draaien (effectieve spoorbreedte SPOOR_ECHT). De robot stuurt
wat de echte stuurt: IMU met 50 Hz (gyro en accelerometer, met ruis en trilling tijdens het rijden), encoders van
beide drivers met 5 Hz, spanning en E-Stop. De watchdog (geen PING in 0,5 s = motoren 0) en de E-Stop (!MG / !EX)
werken zoals op de robot. Geen GPS, geen driverpositie (op 127.0.0.1 gebruikt robot.py die niet), geen camera.
Het is een oefenrobot, geen bewijs dat de echte hetzelfde doet.
"""
import argparse
import math
import random
import socket
import threading
import time

from jaguar.analyse.rit import ACC_LSB_G, GYRO_LSB_DPS, M_PER_CNT
from jaguar.kern.robot import K_V

SPOOR_ECHT = 1.2         # m: effectieve spoorbreedte bij draaien (de wielen zelf staan 0,52 m uit elkaar)
VERTRAGING = 0.25        # s
DODE_ZONE = 60           # vermogen
STIL_V = 0.05            # m/s: zonder vermogen en trager dan dit staat een kant stil (wrijving)
WATCHDOG_S = 0.5
GYRO_BIAS = (600, 0, -580)   # counts, ongeveer zoals op de robot


class Sim:
    """De wereld: blijft bestaan over de verbindingen heen (positie, encoders, batterij), zoals de echte robot."""

    def __init__(self, batterij, verbruik):
        self.conn = None
        self.lock = threading.Lock()
        self.pl = self.pr = 0             # gevraagd vermogen links / rechts
        self.vl = self.vr = 0.0           # m/s
        self.h = 0.0                      # graden
        self.x = self.y = 0.0
        self.wl = self.wr = 0.0           # afgelegde weg per kant (m)
        self.estop = True
        self.ping = time.monotonic()
        self.bat = batterij
        self.verbruik = verbruik          # V per minuut bij vermogen 250 op beide kanten
        self.running = True

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
                    vrij = not self.estop and time.monotonic() - self.ping < WATCHDOG_S
                    pl, pr = (self.pl, self.pr) if vrij else (0, 0)
                tl = 0.0 if abs(pl) < DODE_ZONE else K_V * 0.9 * pl
                tr = 0.0 if abs(pr) < DODE_ZONE else K_V * 0.9 * pr
                k = min(1.0, dt / VERTRAGING)
                self.vl += (tl - self.vl) * k
                self.vr += (tr - self.vr) * k
                # Wrijving: zonder vermogen staat een kant echt stil (anders rolt hij exponentieel en eindeloos uit, en
                # meet de rustmeting van de volgende rit een draaiend robot)
                if tl == 0 and abs(self.vl) < STIL_V:
                    self.vl = 0.0
                if tr == 0 and abs(self.vr) < STIL_V:
                    self.vr = 0.0
                v = (self.vl + self.vr) / 2
                rate = math.degrees((self.vr - self.vl) / SPOOR_ECHT)
                self.h += rate * dt
                self.x += v * math.cos(math.radians(self.h)) * dt
                self.y += v * math.sin(math.radians(self.h)) * dt
                self.wl += self.vl * dt
                self.wr += self.vr * dt
                self.bat -= self.verbruik / 60 * (abs(pl) + abs(pr)) / 500 * dt
                acc = (v - v_oud) / dt
                v_oud = v
                bewegen = abs(self.vl) + abs(self.vr) > 0.02
                tril = 0.08 if bewegen else 0.004
                g = [GYRO_BIAS[0] + random.gauss(0, 1), GYRO_BIAS[1] + random.gauss(0, 1),
                     GYRO_BIAS[2] + rate * GYRO_LSB_DPS + random.gauss(0, 1)]
                a = [acc / 9.81 * ACC_LSB_G + random.gauss(0, 1), random.gauss(0, 1),
                     ACC_LSB_G * (1 + random.gauss(0, tril))]
                self.send(f"#{seq},YAW,{self.h % 360:.2f},GYRO,{','.join(str(round(x)) for x in g)},"
                          f"ACC,{','.join(str(round(x)) for x in a)},ADC,0,0,0,0,")
                seq = (seq + 1) % 256
                n += 1
                if n % 10 == 0:                   # 5 Hz: encoders van beide drivers, spanning
                    cl, cr = round(self.wl / M_PER_CNT), round(self.wr / M_PER_CNT)
                    self.send(f"MM0 C={cl}:{-cr}")
                    self.send(f"MM1 C={cl}:{-cr}")
                    sag = 0.0015 * (abs(pl) + abs(pr)) / 2
                    for b in ("MM0", "MM1"):
                        self.send(f"{b} V=120:{round((self.bat - sag + random.uniform(-0.4, 0.4)) * 10)}:4900")
                if n % 50 == 0:
                    for b in ("MM0", "MM1"):
                        self.send(f"{b} FF={16 if self.estop else 0}")
                t_next += dt
                time.sleep(max(0.0, t_next - time.perf_counter()))
        except OSError:
            pass
        self.running = False
        if log:
            print(f"{time.strftime('%H:%M:%S')}  verbinding dicht; robot op x {self.x:+.2f} m, y {self.y:+.2f} m, "
                  f"koers {self.h:+.1f}°, batterij {self.bat:.1f} V")


def main():
    ap = argparse.ArgumentParser(description="Gesimuleerde robot op 127.0.0.1:10001 die op de motoren reageert")
    ap.add_argument("--port", type=int, default=10001)
    ap.add_argument("--batterij", type=float, default=24.0, help="spanning bij het begin (V)")
    ap.add_argument("--verbruik", type=float, default=0.0, help="V per minuut rijden bij vermogen 250 (standaard 0)")
    args = ap.parse_args()
    srv = socket.create_server(("127.0.0.1", args.port))
    print(f"Gesimuleerde robot klaar op 127.0.0.1:{args.port}. Ctrl+C = stoppen.")
    serve(srv, Sim(args.batterij, args.verbruik))


def serve(srv, sim, log=True):
    """Eén verbinding tegelijk, zoals de robot; ook gebruikt door test_bediening.py."""
    while True:
        conn, _ = srv.accept()
        if log:
            print(f"{time.strftime('%H:%M:%S')}  script verbonden")
        with conn:
            sim.run(conn, log)


if __name__ == "__main__":
    main()
