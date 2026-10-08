"""Fase 1 - Jaguar 4x4: rechtdoor heen en terug rijden, op afstand (encoders) of op tijd.

Protocol (uit de C#-broncode van de Dr Robot Jaguar Control-app):
  - TCP 192.168.0.60:10001, elk commando eindigt op CRLF
  - "PING" minstens elke 200 ms, anders stopt de robot (watchdog)
  - "MMW !M <links> <-rechts>"  open-loop vermogen, bereik -1000..1000
  - "MMW !MG" = E-Stop vrijgeven, "MMW !EX" = E-Stop activeren
  - encoders "MM0/MM1 C=<links>:<rechts>" komen ongevraagd binnen, ongeveer elke 0,2 s
  - IMU "#<nr>,YAW,..,GYRO,gx,gy,gz,ACC,..." met ongeveer 50 Hz; gz houdt de koers recht

Afstand en koers komen uit odometrie.py (slipbewust: vier wielen + gyro + accelerometer).

PC-IP op de DriJaguar-wifi moet 192.168.0.104/24 zijn.
"""
import argparse
import csv
from contextlib import contextmanager
from datetime import datetime
from collections import deque
from pathlib import Path
import math
import socket
import threading
import time

from analyse_rit import M_PER_CNT, WHEEL_R, WHEEL_DIS, GYRO_LSB_DPS, ACC_LSB_G
from odometrie import Odometrie

ROBOT_IP = "192.168.0.60"
ROBOT_PORT = 10001
TICK = 0.1                # PING + motorcommando elke 100 ms
LOOP = 0.05               # regellus bij rijden op afstand
MIN_BATTERY_V = 22.2      # onder deze spanning niet rijden (LiPo)
FAULT_ESTOP = 16          # FF-bit 4 = emergency stop
K_V = 0.0031              # m/s per vermogenseenheid op vlakke grond (uit ritten: 150 → 0,46 m/s); wordt bijgeleerd
APPROACH_MARGIN = 0.02   # vloeiend rijden stopt 2 cm voor het doel; de rest gaat met kruipstoten
STALL_S = 1.5             # zo lang < 15 % van de verwachte snelheid bij vol vermogen = vastgelopen
IMU_HZ = 50.0
# PID-koersregelaar: uitgang = vermogensverschil links/rechts (+ = rechts sneller = naar links)
KP_HEADING = 6.0          # per graad koersfout
KI_HEADING = 4.0          # per graad·seconde: compenseert een kant die blijvend zwakker trekt
KD_HEADING = 0.4          # per graad/s draaisnelheid (gyro): dempt overslingeren
K_LINE = 20.0             # gewenste koers (graden) per meter zijdelingse afwijking, terug naar de startlijn
MAX_LINE_DEG = 5.0
STEER_FRAC = 0.3          # stuurcorrectie hoogstens 30 % van het rijvermogen
JOG_STEER_FRAC = 0.6      # bij kruipstoten mag de zwakke kant meer extra krijgen, anders komt ze niet los
MAX_KANTEL = 35.0         # graden t.o.v. de rusthoek: schuiner = stoppen (rit 134323 kantelde op zijn neus)
ODO_ALARM_S = 1.0         # zo lang "onzeker" of "wielen in de lucht" tijdens het rijden = stoppen
REST_MAX_G = 0.03         # spreiding van de accelerometer in rust (normaal ≈ 0,012 g); meer = robot bewoog


class Stalled(RuntimeError):
    pass


class PID:
    """PID-regelaar.
    - D-term op de meting (bv. de gyrosnelheid) i.p.v. op de fout: geen stoot als het setpoint verspringt.
    - Anti-windup: de integrator groeit niet verder zolang de uitgang verzadigd is en de fout
      dezelfde kant op duwt."""

    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0

    def update(self, setpoint, measured, rate, dt, limit):
        err = setpoint - measured
        u = self.kp * err + self.ki * (self.integral + err * dt) - self.kd * rate
        if abs(u) < limit or (u > 0) != (err > 0):
            self.integral += err * dt
        return max(-limit, min(limit, self.kp * err + self.ki * self.integral - self.kd * rate))


class Jaguar:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT, writer=None, achter_vrij=False, live=None):
        self.writer = writer
        self.live = live      # live weergave (live_view / mqtt_stream): krijgt elke ontvangen regel
        self.t0 = time.perf_counter()
        self.phase = "rust"
        self.power = 0
        self.sock = socket.create_connection((ip, port), timeout=3)
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.running = True
        self.fault = {}       # "MM0"/"MM1" -> FF
        self.encoder = {}     # "MM0"/"MM1" -> (kanaal1, kanaal2)
        self.enc_t = {}       # "MM0"/"MM1" -> ontvangsttijd (perf_counter)
        self.battery_v = None
        self.k_v = K_V
        self.jog_gain = 0.08  # m per seconde kruipstoot; wordt bijgeleerd
        # Afstand, koers (graden, + = links) en zijdelingse afwijking (m, + = links) t.o.v. de
        # startlijn; koers en IMU-correcties pas na zero_heading() in rust.
        self.odo = Odometrie(M_PER_CNT, GYRO_LSB_DPS, ACC_LSB_G, WHEEL_DIS)
        # Achterwielen zonder vermogen (alleen MM0 trekt): de achterwielen meten dan de grond.
        self.achter_vrij = achter_vrij
        if achter_vrij:
            self.odo.meetwielen = "achter"
        self.heading_pid = PID(KP_HEADING, KI_HEADING, KD_HEADING)
        self._steer_t = None
        self._steer_dir = 1
        self._rest = deque(maxlen=150)   # (gyro, acc) bij vermogen 0, voor zero_heading()
        self._imu_seq = None
        # Robotklok (s): telt IMU-volgnummers (50 Hz). Wifi levert pakketten soms 0,5-0,7 s te laat
        # en gebundeld; de volgorde blijft wel juist. Met de ontvangsttijd leek een vertraagd
        # encoderpakket dan te traag en verwierp de odometrie de wielen (rit 151423: 9 cm te ver).
        self._clock = 0.0
        self._imu_since_enc = False
        self._t_last_enc = None
        self._enc_new = set()
        self._alarm_since = None
        self._parse_error = False
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def send(self, cmd):
        with self.lock:
            self.sock.sendall((cmd + "\r\n").encode("ascii"))

    def _read_loop(self):
        buf = b""
        while self.running:
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            buf += data
            *lines, buf = buf.split(b"\r\n")
            for line in lines:
                try:
                    self._parse(line.decode("ascii", "ignore"))
                except (ValueError, IndexError):
                    pass        # beschadigd pakket; de leesdraad mag nooit stoppen tijdens het rijden
                except Exception as e:  # noqa: BLE001 - zonder leesdraad rijdt de robot blind door
                    if not self._parse_error:
                        print(f"Fout bij het verwerken van '{line[:60]!r}': {e!r} (verdere fouten niet getoond)")
                    self._parse_error = True

    def note(self, line):
        """Eigen regel in de CSV (bv. de doelafstand); de sensorparser van de analyse negeert ze."""
        if self.writer is not None:
            with self.log_lock:
                self.writer.writerow([f"{time.perf_counter() - self.t0:.4f}", self.phase, self.power, line])

    def _parse(self, line):
        self.note(line)
        if self.live is not None:
            self.live.feed(time.perf_counter() - self.t0, self.phase, self.power, line)
        if line.startswith("#"):
            p = line.split(",")
            if len(p) == 17 and p[3] == "GYRO" and p[7] == "ACC":
                self._imu(int(p[0][1:]), [int(v) for v in p[4:7]], [int(v) for v in p[8:11]])
            return
        parts = line.split(" ", 1)
        if len(parts) != 2 or parts[0] not in ("MM0", "MM1"):
            return
        board, value = parts
        if value.startswith("FF="):
            self.fault[board] = int(value[3:])
        elif value.startswith("C="):
            self.encoder[board] = tuple(int(v) for v in value[2:].split(":"))
            self.enc_t[board] = time.perf_counter()
            self._enc_new.add(board)
            if {"MM0", "MM1"} <= self._enc_new:           # beide drivers: één odometriestap
                self._enc_new.clear()
                now = time.perf_counter()
                if not self._imu_since_enc and self._t_last_enc is not None:
                    self._clock += now - self._t_last_enc     # geen IMU: dan toch de ontvangsttijd
                self._imu_since_enc, self._t_last_enc = False, now
                (fl, fr), (rl, rr) = self.encoder["MM0"], self.encoder["MM1"]
                self.odo.encoders(self._clock, (fl, -fr, rl, -rr))
        elif value.startswith("V="):
            self.battery_v = int(value[2:].split(":")[1]) / 10

    def _imu(self, seq, gyro, acc):
        """IMU-sample naar de odometrie. Pakketten komen gebundeld via wifi, dus de tijdstap volgt
        het volgnummer (50 Hz, 0..255) in plaats van de ontvangsttijd."""
        gap = 1 if self._imu_seq is None else (seq - self._imu_seq) % 256
        self._imu_seq = seq
        if self.power == 0:
            self._rest.append((gyro, acc))
        steps = gap if 1 <= gap <= 10 else 1
        self._clock += steps / IMU_HZ
        self._imu_since_enc = True
        self.odo.imu(self._clock, steps / IMU_HZ, gyro, acc)

    # Koers en zijdelingse afwijking komen uit de odometrie.
    heading = property(lambda self: self.odo.koers)
    yaw_rate = property(lambda self: self.odo.draaisnelheid)
    lateral = property(lambda self: self.odo.y)

    def zero_heading(self):
        """In rust: gyrobias en zwaartekracht uit de laatste ≈ 3 s; afstand, koers en zijdelingse afwijking op nul.
        Geeft False als de robot tijdens die meting bewoog (dan zijn de biases fout)."""
        rest = list(self._rest)
        if len(rest) < 50:
            print("Geen IMU-data: rijden zonder koerscorrectie en met het gemiddelde van de 4 wielen.")
            return True
        gyro = [sum(g[k] for g, _ in rest) / len(rest) for k in range(3)]
        acc = [sum(a[k] for _, a in rest) / len(rest) for k in range(3)]
        spread = math.sqrt(sum(sum((a[k] - acc[k]) ** 2 for _, a in rest) / len(rest) for k in range(3))) / ACC_LSB_G
        self.odo.nul(gyro, acc)
        return spread < REST_MAX_G

    def _guard(self, p):
        """Stoppen als de robot te schuin staat of de wielen de grond niet meer betrouwbaar raken."""
        if not self.odo.klaar:
            return
        if self.odo.kanteling > MAX_KANTEL:
            raise Stalled(f"robot {self.odo.kanteling:.0f}° gekanteld t.o.v. de start")
        if p != 0 and self.odo.modus in ("onzeker", "lucht"):
            self._alarm_since = self._alarm_since or time.perf_counter()
            if time.perf_counter() - self._alarm_since > ODO_ALARM_S:
                raise Stalled("wielen draaien maar de robot rijdt niet (opgetild, gekanteld of tegen een obstakel)"
                              if self.odo.modus == "lucht" else
                              "wielen en IMU zijn het oneens (alle wielen slippen of vastgelopen)")
        else:
            self._alarm_since = None

    def _steer(self, p, frac=STEER_FRAC):
        """Vermogensverschil om recht op de startlijn te blijven (cascade):
        buiten: zijdelingse afwijking → gewenste koers; binnen: PID op de gyrokoers.
        Rechts sneller dan links draait altijd naar links, ook achteruit."""
        now = time.perf_counter()
        if not self.odo.klaar or p == 0:
            self._steer_t = None
            return 0
        direction = 1 if p > 0 else -1
        if direction != self._steer_dir:
            # Een zwakkere kant duwt achteruit de andere kant op: geleerde integrator spiegelen.
            self.heading_pid.integral *= -1
            self._steer_dir = direction
        dt = min(0.2, now - self._steer_t) if self._steer_t else LOOP
        self._steer_t = now
        target = max(-MAX_LINE_DEG, min(MAX_LINE_DEG, -K_LINE * self.lateral * direction))
        return int(self.heading_pid.update(target, self.heading, self.yaw_rate, dt, frac * abs(p)))

    def _motor(self, p, steer=0):
        """Vermogen `p` op beide kanten; `steer` > 0 maakt rechts sneller (draait naar links).
        Kanaal 1 = links (vooruit omhoog), kanaal 2 = rechts (vooruit omlaag)."""
        self.power = p
        left = max(-1000, min(1000, p - steer))
        right = max(-1000, min(1000, p + steer))
        self.send("PING")
        if self.achter_vrij:
            self.send(f"MM0 !M {left} {-right}")
            self.send("MM1 !M 0 0")
        else:
            self.send(f"MMW !M {left} {-right}")

    def drive(self, power, seconds, ramp=1.5):
        """Rechtdoor rijden met vermogen `power` (-1000..1000) gedurende `seconds`, met zachte op- en afbouw."""
        ramp = min(ramp, seconds / 2)
        t0 = time.time()
        try:
            while (t := time.time() - t0) < seconds:
                scale = min(1.0, t / ramp, (seconds - t) / ramp) if ramp > 0 else 1.0
                p = int(power * scale)
                self._guard(p)
                self._motor(p, self._steer(p))
                time.sleep(TICK)
        except Stalled as e:
            print(f"{self.phase}: GESTOPT - {e}.")
            self.note(f"VAST {self.phase}")
        self._motor(0)

    def drive_distance(self, power, distance, ramp=1.5, creep=60, brake=0.4, tolerance=0.003):
        """Rijdt `distance` m volgens de slipbewuste odometrie; het teken van `power` is de richting.
        Vermogen bouwt lineair op over `ramp` s, zakt met de resterende afstand (vertraging
        `brake` m/s²) tot kruipvermogen `creep` en stopt `APPROACH_MARGIN` voor het doel.
        Encoders komen maar met ≈ 5 Hz binnen, dus de laatste millimeters gaan met korte
        kruipstoten die telkens in stilstand gemeten worden, tot de fout binnen `tolerance` m ligt.
        Geeft de gemeten verplaatsing (m, met teken) terug."""
        self._wait_encoders()
        sign = 1 if power >= 0 else -1
        self.creep = max(creep, getattr(self, "creep", creep))
        start = self._settled()
        corr0, unc0 = self.odo.gecorrigeerd, self.odo.onzekerheid
        goal = start + sign * distance
        self.note(f"DOEL {self.phase} {sign * distance:.4f} r={WHEEL_R:.5f}")
        try:
            self._approach(goal - sign * APPROACH_MARGIN, abs(power), ramp, brake)
            self._jog(goal, abs(power), tolerance)
        except Stalled as e:
            self._motor(0)
            print(f"{self.phase}: VASTGELOPEN - {e}. Segment afgebroken.")
            self.note(f"VAST {self.phase}")
        moved = self._settled() - start
        err = moved - sign * distance
        self.note(f"STOP {self.phase} {moved:.4f}")
        print(f"{self.phase}: doel {sign * distance * 1000:+.0f} mm, gemeten {moved * 1000:+.1f} mm, "
              f"fout {err * 1000:+.1f} mm (1 encoderpuls = {M_PER_CNT * 1000:.1f} mm)")
        corr, unc = self.odo.gecorrigeerd - corr0, self.odo.onzekerheid - unc0
        if corr > 0.005 or unc > 0.005:
            print(f"{self.phase}: slip: {corr * 100:.1f} cm niet uit het gemiddelde van de 4 wielen, "
                  f"onzekerheid ± {unc * 100:.1f} cm")
        if self.odo.klaar:
            print(f"{self.phase}: koers {self.heading:+.1f}°, naast de startlijn {self.lateral * 100:+.1f} cm (+ = links, schatting)")
        return moved

    def _approach(self, goal, top, ramp, brake):
        """Eén vloeiende rijbeweging tot vlak voor `goal`."""
        pos = self.odo.positie()
        direction = 1 if goal >= pos else -1
        t0, last_n = time.perf_counter(), self.odo.n_enc
        moved_at, moved_pos, slow_since = t0, pos, None
        timeout = 10 + abs(goal - pos) / 0.05
        p = 0
        while True:
            now = time.perf_counter()
            if now - t0 > timeout:
                raise Stalled(f"doel niet bereikt na {timeout:.0f} s")
            # Positie nu = laatste encoderpakket + IMU-integratie sindsdien (50 Hz i.p.v. 5 Hz).
            pos, v = self.odo.positie(), self.odo.v
            if self.odo.n_enc != last_n:                     # nieuw encoderpakket (≈ 5 Hz)
                last_n = self.odo.n_enc
                if p >= 0.9 * top and abs(v) > 0.05:        # verband vermogen → snelheid bijleren
                    self.k_v = 0.8 * self.k_v + 0.2 * abs(v) / p
            if abs(pos - moved_pos) >= M_PER_CNT:
                moved_at, moved_pos = now, pos
            remaining = direction * (goal - pos)
            if remaining <= 0:
                break
            p_ramp = top * min(1.0, (now - t0) / ramp) if ramp > 0 else top
            p_brake = max(self.creep, math.sqrt(2 * brake * remaining) / self.k_v)
            p = int(min(top, p_ramp, p_brake))
            # Kruipvermogen te laag om te bewegen: stap voor stap verhogen (blijft voor volgende segmenten).
            if p <= self.creep < top and now - t0 > ramp and now - moved_at > 0.6:
                self.creep = min(top, self.creep + 10)
                self.jog_gain *= 1.5
                moved_at = now
            # Vol vermogen maar (bijna) geen beweging: vastgelopen of op een obstakel.
            if p >= 0.9 * top and abs(v) < 0.15 * self.k_v * p:
                slow_since = slow_since or now
                if now - slow_since > STALL_S:
                    raise Stalled(f"{abs(v):.2f} m/s bij vermogen {p}")
            else:
                slow_since = None
            self._guard(p)
            self._motor(direction * p, self._steer(direction * p))
            time.sleep(LOOP)
        self._motor(0)

    def _jog(self, goal, top, tolerance, tries=10):
        """Korte kruipstoten tot binnen `tolerance`; elke stoot wordt in stilstand nagemeten
        en leert hoeveel meter een seconde kruipen oplevert."""
        last_err = None
        for _ in range(tries):
            pos = self._settled()
            err = goal - pos
            if abs(err) <= tolerance:
                return
            if last_err is not None and err * last_err < 0:  # doorgeschoten: voortaan kortere stoten
                self.jog_gain *= 2
            last_err = err
            pulse = min(0.6, max(0.04, abs(err) / self.jog_gain))
            t0 = time.perf_counter()
            while (left := pulse - (time.perf_counter() - t0)) > 0:
                c = int(math.copysign(self.creep, err))
                self._guard(c)
                self._motor(c, self._steer(c, JOG_STEER_FRAC))
                time.sleep(min(LOOP, left))
            self._motor(0)
            moved = abs(self._settled() - pos)
            if moved < M_PER_CNT / 2:                        # stoot te zwak om los te komen
                if self.creep >= top:
                    raise Stalled("kruipstoten bewegen de robot niet")
                self.creep = min(top, self.creep + 10)
                self.jog_gain *= 1.5                         # meer vermogen = verder per stoot
            else:
                self.jog_gain = max(0.5 * self.jog_gain + 0.5 * moved / pulse, moved / pulse)

    def _wait_encoders(self, wait=2.0):
        t0 = time.perf_counter()
        while len(self.enc_t) < 2:
            if time.perf_counter() - t0 > wait:
                raise SystemExit("Geen encoderwaarden van beide motordrivers ontvangen.")
            self._motor(0)
            time.sleep(TICK)

    def _settled(self, wait=2.0):
        """Stilstaan tot twee opeenvolgende encoderpakketten geen beweging tonen; geeft de positie (m)."""
        t0 = time.perf_counter()
        prev, seen, stable = None, None, 0
        while time.perf_counter() - t0 < wait:
            self._motor(0)
            time.sleep(LOOP)
            if self.odo.n_enc != seen:
                seen, pos = self.odo.n_enc, self.odo.s
                stable = stable + 1 if prev is not None and abs(pos - prev) < M_PER_CNT / 2 else 0
                prev = pos
                if stable >= 2:
                    break
        return self.odo.s

    def hold(self, seconds):
        """Stilstaan maar de watchdog levend houden."""
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._motor(0)
            time.sleep(TICK)

    def close(self):
        try:
            self.send("MMW !M 0 0")
            self.send("MMW !EX")
        finally:
            self.running = False
            self.sock.close()
            self.reader.join(1.0)



@contextmanager
def robot_session(record=True, suffix="", ip=ROBOT_IP, port=ROBOT_PORT, live=True, mqtt=False):
    path = None
    f = None
    robot = None
    view = None
    try:
        writer = None
        if record:
            directory = Path(__file__).parent / "ritten"
            directory.mkdir(exist_ok=True)
            path = directory / f"rit_{datetime.now():%Y%m%d_%H%M%S_%f}{suffix}.csv"
            f = path.open("w", newline="", encoding="utf-8")
            writer = csv.writer(f)
            writer.writerow(["t", "fase", "vermogen", "regel"])
        name = path.stem if path else f"rit_{datetime.now():%Y%m%d_%H%M%S}_zonder_csv"
        if live:
            import live_view
            view = live_view.start(name)
        if mqtt:
            from mqtt_stream import MqttStream
            view = MqttStream(name, view)   # rekent via dezelfde LiveView en publiceert naar MQTT/Grafana
        robot = Jaguar(ip, port, writer=writer, live=view)
        yield robot
    finally:
        try:
            if robot is not None:
                robot.close()
                print("Motoren 0, E-Stop terug actief.")
            if view is not None:
                view.finish()
                time.sleep(0.3)  # laatste gegevens nog naar de browser sturen
        finally:
            if f is not None:
                f.close()
                print(f"Log: {path}")


def prepare(robot, rest):
    robot.hold(rest)
    for _ in range(3):
        if robot.zero_heading():
            break
        print("Robot bewoog tijdens de rustmeting (gyrobias en helling zouden fout zijn): niet aanraken, opnieuw meten.")
        robot.hold(rest)
    else:
        print("Rustmeting blijft onrustig: koers en IMU-correcties zijn minder betrouwbaar.")
    print(f"Batterij: {robot.battery_v} V, fault flags: {robot.fault}")
    if robot.battery_v is not None and robot.battery_v < MIN_BATTERY_V:
        raise SystemExit(f"Batterij te laag ({robot.battery_v} V), niet rijden.")
    robot.phase = "vrijgeven"
    robot.send("MMW !MG")
    robot.hold(1.0)
    if any(ff & FAULT_ESTOP for ff in robot.fault.values()):
        raise SystemExit(f"E-Stop blijft actief (FF={robot.fault}). Check de fysieke noodstop.")


def nonzero(value):
    value = float(value)
    if value == 0 or not abs(value) < float("inf"):
        raise argparse.ArgumentTypeError("waarde mag niet nul zijn")
    return value


def positive_seconds(value):
    value = float(value)
    if not 0 < value < float("inf"):
        raise argparse.ArgumentTypeError("waarde moet positief en eindig zijn")
    return value


def add_drive_args(ap, distance):
    """Gemeenschappelijke rij-opties: afstand (standaard) of tijd, en de zachte opbouw."""
    how = ap.add_mutually_exclusive_group()
    how.add_argument("--distance", type=nonzero, default=distance, help=f"meter volgens de encoders (standaard {distance}); negatief = achteruit")
    how.add_argument("--duration", type=positive_seconds, help="seconden rijden in plaats van een afstand")
    ap.add_argument("--ramp", type=float, default=1.5, help="seconden om van 0 naar het volle vermogen te gaan")
    ap.add_argument("--creep", type=int, default=60, help="kruipvermogen vlak voor het doel")
    ap.add_argument("--tolerance", type=float, default=3.0, help="toegelaten stopfout in mm (min. ±1 encoderpuls ca. 2,8 mm)")
    ap.add_argument("--ip", default=ROBOT_IP, help="IP van de robot; 127.0.0.1 = nep_robot.py")
    ap.add_argument("--port", type=int, default=ROBOT_PORT)
    ap.add_argument("--live", action=argparse.BooleanOptionalAction, default=True, help="live weergave in de browser (standaard aan)")
    ap.add_argument("--mqtt", action=argparse.BooleanOptionalAction, default=False, help="live naar MQTT/Grafana sturen (zie grafana/README.md)")


def run_segment(robot, args, power):
    """Rijrichting = teken van power × teken van distance; zo werkt zowel --power -250 als --distance -3."""
    if args.duration:
        robot.drive(power, args.duration, args.ramp)
    else:
        sign = -1 if args.distance < 0 else 1
        robot.drive_distance(sign * power, abs(args.distance), args.ramp, args.creep, tolerance=args.tolerance / 1000)


def main():
    ap = argparse.ArgumentParser(description="Fase 1: heen en terug rijden")
    ap.add_argument("--power", type=int, default=250, help="maximaal motorcommando, geen snelheid in m/s; negatief = eerst achteruit, dan terug vooruit")
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.5)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True, help="CSV opslaan (standaard aan); --no-csv schakelt uit")
    args = ap.parse_args()
    if not -1000 <= args.power <= 1000:
        ap.error("--power moet tussen -1000 en 1000 liggen")
    with robot_session(args.csv, "_heen_terug", args.ip, args.port, args.live, args.mqtt) as robot:
        prepare(robot, args.rest)
        robot.phase = "heen"
        run_segment(robot, args, args.power)
        robot.phase = "pauze"
        robot.hold(args.pause)
        robot.phase = "terug"
        run_segment(robot, args, -args.power)
        robot.phase = "uitbollen"
        robot.hold(2.0)


if __name__ == "__main__":
    main()
