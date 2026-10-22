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
import json
from contextlib import contextmanager
from datetime import datetime
from collections import deque
from pathlib import Path
import math
import os
import socket
import threading
import time

from jaguar.analyse.rit import M_PER_CNT, WHEEL_R, WHEEL_DIS, GYRO_LSB_DPS, ACC_LSB_G
from jaguar.kern.odometrie import Odometrie

ROBOT_IP = "192.168.0.60"
ROBOT_PORT = 10001
TICK = 0.1                # PING + motorcommando elke 100 ms
LOOP = 0.05               # regellus bij rijden op afstand
MIN_BATTERY_V = 22.2      # onder deze spanning niet rijden (LiPo)
FAULT_ESTOP = 16          # FF-bit 4 = emergency stop
K_V = 0.0031              # m/s per vermogenseenheid op vlakke grond (uit ritten: 150 → 0,46 m/s); wordt bijgeleerd
APPROACH_MARGIN = 0.02   # vloeiend rijden stopt 2 cm voor het doel; de rest gaat met kruipstoten
# Positieregeling in de motordrivers (Roboteq, MMOD 3): de driver brengt elk wiel op de puls nauwkeurig
# naar een absoluut doel (1000x per seconde); het PC-programma houdt tot dan de koers met de gyro.
# Gemeten: 1 m en 5 m heen en terug binnen 2 tot 6 pulsen (6 tot 17 mm) per wiel. Alleen voor het laatste
# stuk: op lange afstand regelt elk wiel apart en kan het robot wegdraaien (28 graden in 5 m buiten).
DRIVER_KP, DRIVER_KI, DRIVER_KD = 255, 0, 120   # KI > 0 maakt de wielen onrustig; maximum KP is 255
DRIVER_LIMIT = 40         # % vermogen in positiemodus
DRIVER_MARGIN = 0.30      # m: zo ver voor het doel neemt de driver het over
DRIVER_AFWIJKING = 150    # pulsen: wielen die zoveel van elkaar afwijken = noodstop (draait weg)
DRIVER_STUUR_KI = 0.4     # deel van de koersfout dat per stap (5 Hz) wordt weggestuurd
DRIVER_STUUR_RATIO = 2.5  # pulsverschil per graad t.o.v. de wielgeometrie, zolang er nog geen draai gemeten is
DRIVER_STUUR_MAX = 60     # pulsen: zoveel mogen de doelen links en rechts uit elkaar worden geschoven
DRIVER_KOERS_OK = 0.3     # graden: binnen deze koersfout stuurt hij niet meer bij
DRIVER_FABRIEK = {"KP": 30, "KI": 10, "KD": 0, "MXPF": 100, "MXPR": 100, "MMOD": 0}
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
TURN_TOP = 380            # draaien ter plaatse: maximaal vermogensverschil …
TURN_MIN = 220            # … en minimum om de wrijving van skid-steer te overwinnen (wordt bijgeleerd)
TURN_K = 6.0              # vermogen per graad resterende hoek
TURN_LEAD = 0.25          # s: uitloop na vermogen 0, voorspeld met de draaisnelheid
TURN_TOL = 1.0            # graden: einde van de vloeiende draai (met voorspelde uitloop)
EINDKOERS_TOL = 1.0       # graden: grotere koersfout na een segment wordt ter plaatse rechtgezet
TURN_FINE = 0.4           # graden: eindtolerantie na het nadraaien met korte stoten
TURN_TRIES = 10
TURN_MIN_PULSE = 0.04     # s: kortste draaistoot
TURN_RATIO_ALARM = 0.15   # gyro/encoders: meer dan 15 % anders dan de vorige draai = melding
LEERWAARDEN = Path(__file__).resolve().parents[2] / "config" / "leerwaarden.json"
REST_MAX_G = 0.03         # spreiding van de accelerometer in rust (normaal ≈ 0,012 g); meer = robot bewoog


class Stalled(RuntimeError):
    pass


class Aborted(RuntimeError):
    """STOP op de live pagina (bediening.py): motoren 0, rit afgebroken."""


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
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT, writer=None, achter_vrij=False, live=None, driver=None):
        self.writer = writer
        self.live = live      # live weergave (live.view / mqtt_stream): krijgt elke ontvangen regel
        self.t0 = time.perf_counter()
        self.phase = "rust"
        self.power = 0
        self.stop_requested = False   # True = elk volgend motorcommando wordt 0 en de rit breekt af
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
        # Driver-positie alleen op de echte robot (de nep-robot kent de commando's niet) en niet met vrije achterwielen.
        local = ip in ("127.0.0.1", "localhost")
        self.driver = (not local and not achter_vrij and os.environ.get("JAGUAR_DRIVER") != "0") if driver is None else driver
        self._acks = {"MM0": 0, "MM1": 0}
        self._echo = {}                  # laatste commando zoals de driver het ontving (hij herhaalt het in zijn logregel)
        self._cfg = {}                   # "MM0 KP" -> [kanaal1, kanaal2]
        self._driver_gewijzigd = False
        self._alarm_since = None
        self._parse_error = False
        # Te volgen rechte lijn: oorsprong (m) en koers (graden). Na een draai begint een nieuwe lijn.
        self.target_heading = 0.0
        self.line = (0.0, 0.0)
        self.turn_min = TURN_MIN
        self.turn_gain = 40.0            # graden per seconde kruipstoot bij draaien; wordt bijgeleerd
        self.turn_ratio = None           # encoders/gyro bij de vorige draai
        self._leerwaarden_laden()
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
        if value.startswith(("!P", "^")):
            self._echo[board] = line
        if value == "+":
            self._acks[board] += 1
        elif value.startswith("FF="):
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
        elif "=" in value:                                  # antwoord op een ~instelling
            naam, _, waarde = value.partition("=")
            if naam not in ("A", "T", "S", "P", "D", "CR", "AI", "CB"):
                self._cfg[f"{board} {naam}"] = [int(x) for x in waarde.split(":")]

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
    @property
    def lateral(self):
        """Afstand (m) links (+) of rechts (−) van de te volgen lijn."""
        h = math.radians(self.target_heading)
        dx, dy = self.odo.x - self.line[0], self.odo.y - self.line[1]
        return -dx * math.sin(h) + dy * math.cos(h)

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
        self.target_heading, self.line = 0.0, (0.0, 0.0)
        return spread < REST_MAX_G

    # Bijgeleerde regelwaarden blijven bewaard tussen sessies, zodat de eerste stap van een rit niet
    # weer met de standaardwaarden begint (die hangen van batterij en ondergrond af). Binnen de
    # grenzen: een vastgelopen sessie mag geen onzin bewaren.
    GRENZEN = {"k_v": (0.0015, 0.006), "jog_gain": (0.02, 0.5), "turn_min": (100, TURN_TOP),
               "turn_gain": (10.0, 200.0)}

    def _leerwaarden_laden(self):
        try:
            data = json.loads(LEERWAARDEN.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        for naam, (lo, hi) in self.GRENZEN.items():
            if isinstance(data.get(naam), (int, float)) and lo <= data[naam] <= hi:
                setattr(self, naam, data[naam])

    def _leerwaarden_bewaren(self):
        data = {n: getattr(self, n) for n in self.GRENZEN if getattr(self, n) is not None}
        data = {n: round(v, 4) for n, v in data.items() if self.GRENZEN[n][0] <= v <= self.GRENZEN[n][1]}
        try:
            LEERWAARDEN.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except OSError:
            pass

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
        """Vermogensverschil om recht op de lijn te blijven (cascade):
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
        target = self.target_heading + max(-MAX_LINE_DEG, min(MAX_LINE_DEG, -K_LINE * self.lateral * direction))
        return int(self.heading_pid.update(target, self.heading, self.yaw_rate, dt, frac * abs(p)))

    def _spin(self, p):
        """Ter plaatse draaien: p > 0 = naar links (linkerwielen achteruit, rechterwielen vooruit)."""
        self._motor(0, int(p))
        self.power = abs(int(p))          # de motoren draaien: niet als rust tellen, wel loggen

    def _settle_turn(self, wait=1.5):
        """Stilstaan tot de draaisnelheid onder 1°/s zakt."""
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < wait:
            self._motor(0)
            time.sleep(LOOP)
            if time.perf_counter() - t0 > 0.3 and abs(self.yaw_rate) < 1.0:
                break

    def turn(self, angle, top=TURN_TOP, label="DRAAI"):
        """Ter plaatse `angle` graden draaien volgens de gyro (+ = links, − = rechts).
        Vermogen zakt met de resterende hoek; stopt vroeger met de voorspelde uitloop en
        corrigeert daarna met korte stoten tot binnen TURN_TOL. Geeft de gedraaide hoek terug.
        angle = 0 zet de koers terug op de gewenste koers; label="KOERS" telt dan niet als stap van het plan."""
        if not self.odo.klaar:
            raise SystemExit("Draaien kan niet zonder gyro (geen IMU-data in de rustmeting).")
        start, target = self.heading, self.target_heading + angle
        enc0 = self._enc_heading()
        self.note(f"{label} {self.phase} {angle:.2f}")
        t0 = moved_at = time.perf_counter()
        timeout = 8 + abs(angle) / 10
        try:
            while True:
                now = time.perf_counter()
                err = target - self.heading
                ahead = err - self.yaw_rate * TURN_LEAD
                if abs(ahead) <= TURN_TOL or ahead * err < 0:
                    break
                if now - t0 > timeout:
                    raise Stalled(f"draai niet af na {timeout:.0f} s")
                if self.odo.kanteling > MAX_KANTEL:
                    raise Stalled(f"robot {self.odo.kanteling:.0f}° gekanteld t.o.v. de start")
                p = max(self.turn_min, min(top, TURN_K * abs(err))) * min(1.0, 0.4 + (now - t0) / 0.5)
                # Draait hij niet (wrijving te groot)? Minimum stap voor stap verhogen; blijft voor volgende draaien.
                if abs(self.yaw_rate) > 3:
                    moved_at = now
                elif now - t0 > 0.8 and now - moved_at > 0.5:
                    self.turn_min, moved_at = min(top, self.turn_min + 20), now
                self._spin(math.copysign(p, err))
                time.sleep(LOOP)
            self._motor(0)
            for _ in range(TURN_TRIES):                      # nadraaien met korte stoten, tot TURN_FINE
                self._settle_turn()
                err = target - self.heading
                # Een stoot is minstens TURN_MIN_PULSE s; is de fout kleiner dan de helft daarvan, dan
                # maakt nog een stoot het slechter (heen en weer slingeren).
                if abs(err) <= max(TURN_FINE, 0.5 * self.turn_gain * TURN_MIN_PULSE):
                    break
                pulse = min(0.4, max(TURN_MIN_PULSE, abs(err) / self.turn_gain))
                before, t1 = self.heading, time.perf_counter()
                while time.perf_counter() - t1 < pulse:
                    self._spin(math.copysign(self.turn_min, err))
                    time.sleep(min(LOOP, pulse))
                self._motor(0)
                self._settle_turn()
                moved = (self.heading - before) * math.copysign(1, err)
                if moved < 0.1:                              # stoot te zwak: meer vermogen
                    self.turn_min = min(top, self.turn_min + 10)
                else:                                        # conservatief: liever te korte stoten
                    self.turn_gain = max(0.5 * self.turn_gain + 0.5 * moved / pulse, moved / pulse)
        except Stalled as e:
            self._motor(0)
            print(f"{self.phase}: GESTOPT - {e}.")
            self.note(f"VAST {self.phase}")
        self._settle_turn()
        turned = self.heading - start
        self.note(f"GEDRAAID {self.phase} {turned:.2f}")
        # Volgende rechte stukken volgen de gewenste koers vanaf hier.
        self.target_heading, self.line = target, (self.odo.x, self.odo.y)
        self.heading_pid.integral = 0.0
        print(f"{self.phase}: doel {angle:+.1f}°, gedraaid {turned:+.1f}° volgens de gyro, fout {turned - angle:+.1f}°")
        self._check_ratio(turned, enc0)
        return turned

    def eindkoers(self, turn_power=TURN_TOP, tol=EINDKOERS_TOL):
        """Na een rechte stap: staat de koers meer dan `tol` graden naast de gewenste koers, dan ter plaatse rechtzetten.
        Geeft True als er gedraaid is."""
        if not self.odo.klaar or abs(self.target_heading - self.heading) <= tol:
            return False
        phase, self.phase = self.phase, "eindkoers"
        try:
            self.turn(0, turn_power, label="KOERS")
        finally:
            self.phase = phase
        return True

    def _enc_heading(self):
        """Koers (graden) uit het verschil links/rechts van de vier wielen; skid-steer slipt, dus alleen een controle."""
        w = self.odo._wielen
        if w is None:
            return None
        return math.degrees(((w[1] + w[3]) - (w[0] + w[2])) / 2 * M_PER_CNT / WHEEL_DIS)

    def _check_ratio(self, turned, enc0):
        """Verhouding encoders/gyro van een draai. De waarde zelf hangt af van de ondergrond (slip) maar is
        van draai tot draai stabiel; een plotse sprong = gyro (bias, schaal) of wielen (slip, vast) deugen niet."""
        enc1 = self._enc_heading()
        if enc0 is None or enc1 is None or abs(turned) < 20:
            return
        ratio = (enc1 - enc0) / turned
        self.note(f"RATIO {self.phase} {ratio:.3f}")
        if self.turn_ratio and abs(ratio / self.turn_ratio - 1) > TURN_RATIO_ALARM:
            print(f"{self.phase}: let op, encoders/gyro = {ratio:.2f} (vorige draai {self.turn_ratio:.2f}): "
                  f"gyro of wielen gedragen zich anders dan eerder")
        self.turn_ratio = ratio if self.turn_ratio is None else 0.7 * self.turn_ratio + 0.3 * ratio

    def _motor(self, p, steer=0):
        """Vermogen `p` op beide kanten; `steer` > 0 maakt rechts sneller (draait naar links).
        Kanaal 1 = links (vooruit omhoog), kanaal 2 = rechts (vooruit omlaag)."""
        if self.stop_requested:   # alle rijlussen komen hier langs: STOP werkt in elke fase
            self.power = 0
            self.send("PING")
            self.send("MMW !M 0 0")
            raise Aborted("STOP op de live pagina")
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

    # --- positieregeling in de drivers -------------------------------------------------
    # De drivers laten commando's vallen als ze te snel na elkaar komen: elk commando wacht daarom
    # op zijn bevestiging ("+") en wordt zo nodig herhaald. Alleen commando's die je veilig kunt
    # herhalen (instellingen en absolute doelen !P), nooit relatieve (!PR).
    def _acked(self, board, text, tries=6):
        """Commando sturen tot de driver het bevestigt ("+") EN zijn echo precies gelijk is: de controller
        verliest soms het eind van een commando (gezien: "!P 2 -474PING" i.p.v. "-4740"), en zo'n kapot
        doel bevestigt hij toch. Een afwijkende echo wordt meteen vervangen door het goede commando."""
        for _ in range(tries):
            before = self._acks[board]
            self._echo.pop(board, None)
            self.send(f"{board} {text}")
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < 0.3:
                if self._acks[board] > before:
                    t1 = time.perf_counter()
                    while self._echo.get(board) is None and time.perf_counter() - t1 < 0.15:
                        time.sleep(0.01)              # de echo kan iets na de bevestiging komen
                    if self._echo.get(board) == f"{board} {text}":
                        time.sleep(0.03)              # rust vóór het volgende commando of de PING
                        return
                    break
                time.sleep(0.01)
            self.note(f"DRIVERCOMMANDO verminkt: {self._echo.get(board)!r} i.p.v. {board} {text}")
            self.send("PING")                         # de watchdog stopt de robot na 0,2 s zonder PING
        raise Stalled(f"{board} {text}: de driver bevestigt niet of ontvangt het verminkt")

    def _set_cfg(self, naam, vals):
        """Instelling per kanaal zetten en teruglezen tot ze klopt (vals = [kanaal 1, kanaal 2])."""
        for board in ("MM0", "MM1"):
            for ch in (1, 2):
                for _ in range(6):
                    self.send("PING")
                    self.send(f"{board} ^{naam} {ch} {vals[ch - 1]}")
                    time.sleep(0.12)
                    self._cfg.pop(f"{board} {naam}", None)
                    self.send(f"{board} ~{naam}")
                    t0 = t_ping = time.perf_counter()
                    while time.perf_counter() - t0 < 0.5 and f"{board} {naam}" not in self._cfg:
                        time.sleep(0.02)
                        if time.perf_counter() - t_ping > 0.1:
                            self.send("PING")
                            t_ping = time.perf_counter()
                    if self._cfg.get(f"{board} {naam}", [None, None])[ch - 1] == vals[ch - 1]:
                        break
                else:
                    raise Stalled(f"driver {board}: instelling {naam} lukt niet")

    def _driver_mode(self, aan):
        self.send("MMW !M 0 0")
        self._set_cfg("MMOD", [3, 3] if aan else [0, 0])
        if not aan:
            self.send("MMW !MG")          # was de watchdog toch geslagen, dan zou het rijden stilvallen: opnieuw vrijgeven
            self.send("PING")

    def driver_fabrieksinstelling(self):
        """Alles terug zoals de fabriek het had (open loop, PID 30/10/0, volle limiet)."""
        if not self._driver_gewijzigd:
            return
        try:
            self.send("MMW !M 0 0")
            self._set_cfg("MMOD", [0, 0])
            for naam in ("KP", "KI", "KD", "MXPF", "MXPR"):
                self._set_cfg(naam, [DRIVER_FABRIEK[naam]] * 2)
            self._driver_gewijzigd = False
        except (Stalled, OSError) as e:
            print(f"Let op: driverinstellingen niet hersteld ({e}); zet het robot uit en aan.")

    def _driver_targets(self, raw, pulses, diff):
        """Absolute doelen in tellerstanden: alle wielen `pulses` vooruit, rechts `diff` pulsen méér dan links
        (diff > 0 draait naar links). Kanaal 2 telt vooruit negatief."""
        links, rechts = pulses - diff / 2, pulses + diff / 2
        return {"MM0": (round(raw["MM0"][0] + links), round(raw["MM0"][1] - rechts)),
                "MM1": (round(raw["MM1"][0] + links), round(raw["MM1"][1] - rechts))}

    def _driver_move(self, pulses):
        """Alle vier de wielen `pulses` impulsen vooruit (negatief = achteruit) door de driver laten regelen.
        Absolute doelen uit de huidige tellerstand, dus herhalen is veilig. Wacht tot alle wielen stilstaan.
        Tijdens het rijden meet de gyro de koersfout en schuift de doelen links/rechts een paar pulsen uit
        elkaar tot het robot weer op de lijn staat (de driver regelt elk wiel apart en zou anders wegdraaien)."""
        self._driver_gewijzigd = True
        for naam, waarde in (("KP", DRIVER_KP), ("KI", DRIVER_KI), ("KD", DRIVER_KD), ("MXPF", DRIVER_LIMIT), ("MXPR", DRIVER_LIMIT)):
            self._set_cfg(naam, [waarde, waarde])
        sign = 1 if pulses >= 0 else -1
        self._driver_mode(True)
        try:
            time.sleep(0.3)
            raw = {b: list(self.encoder[b]) for b in ("MM0", "MM1")}
            start = [raw["MM0"][0], -raw["MM0"][1], raw["MM1"][0], -raw["MM1"][1]]   # vooruit positief
            diff, sent = 0.0, 0
            for b, (t1, t2) in self._driver_targets(raw, pulses, 0).items():
                self._acked(b, f"!P 1 {t1}")
                self._acked(b, f"!P 2 {t2}")
            self.power = 1                     # de motoren draaien: niet als rust tellen
            t0 = last_move = t_stuur = time.perf_counter()
            last = None
            timeout = 6 + abs(pulses) * M_PER_CNT * 14
            while True:
                if self.stop_requested:
                    raise Aborted("STOP op de live pagina")
                self.send("PING")
                time.sleep(LOOP)
                (fl, fr), (rl, rr) = self.encoder["MM0"], self.encoder["MM1"]
                now_c = [fl, -fr, rl, -rr]
                moved = [c - s0 for c, s0 in zip(now_c, start)]
                if any(sign * m > abs(pulses) + 60 for m in moved) or max(moved) - min(moved) > DRIVER_AFWIJKING:
                    raise Stalled(f"wielen lopen uit elkaar tijdens de driverpositie: {moved}")
                if self.odo.klaar and time.perf_counter() - t_stuur >= 0.2:
                    t_stuur = time.perf_counter()
                    doel = self.target_heading + max(-MAX_LINE_DEG, min(MAX_LINE_DEG, -K_LINE * self.lateral * sign))
                    fout = doel - self.heading
                    if abs(fout) > DRIVER_KOERS_OK:
                        # fout > 0 = naar links nodig = rechts verder dan links (ook achteruit)
                        # Skid-steer slipt: voor één graad zijn `verhouding` keer zoveel pulsen verschil nodig als de
                        # wielgeometrie zegt (gemeten 1,8 tot 3,9 afhankelijk van de ondergrond, zie turn()).
                        verhouding = min(5.0, max(1.0, self.turn_ratio or DRIVER_STUUR_RATIO))
                        diff = max(-DRIVER_STUUR_MAX, min(DRIVER_STUUR_MAX, diff + DRIVER_STUUR_KI * verhouding * fout / math.degrees(M_PER_CNT / WHEEL_DIS)))
                    if round(diff) != sent:
                        sent = round(diff)
                        for b, (t1, t2) in self._driver_targets(raw, pulses, sent).items():
                            self._acked(b, f"!P 1 {t1}")
                            self._acked(b, f"!P 2 {t2}")
                if now_c != last:
                    last, last_move = now_c, time.perf_counter()
                if time.perf_counter() - t0 > 2.0 and time.perf_counter() - last_move > 1.0:
                    break
                if time.perf_counter() - t0 > timeout:
                    raise Stalled(f"driverpositie niet bereikt na {timeout:.0f} s")
        finally:
            try:
                self._driver_mode(False)
            finally:
                self.power = 0
        eind = [(a - s0) - pulses for a, s0 in zip(now_c, start)]
        self.note(f"DRIVER {self.phase} {pulses} fout {eind} bijsturing {sent} pulsen koers {self.heading:.2f}")
        return eind

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
            if self.driver and self.odo.klaar:
                self._drive_with_driver(goal, sign, power, ramp, brake, tolerance)
            else:
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
            print(f"{self.phase}: koers {self.heading:+.1f}°, naast de lijn {self.lateral * 100:+.1f} cm (+ = links, schatting)")
        return moved

    def _drive_with_driver(self, goal, sign, power, ramp, brake, tolerance):
        """Tot DRIVER_MARGIN voor het doel rijdt dit programma met de gyro; het laatste stuk regelt de driver."""
        pos = self.odo.positie()
        if sign * (goal - pos) > DRIVER_MARGIN + 0.02:
            self._approach(goal - sign * DRIVER_MARGIN, abs(power), ramp, brake)
        pos = self._settled()
        pulses = round((goal - pos) / M_PER_CNT)
        if abs(pulses) >= 1:
            try:
                self._driver_move(pulses)
            except Stalled as e:                 # de driver doet het niet: kruipstoten zoals vroeger
                self.driver_fabrieksinstelling()
                print(f"{self.phase}: driverpositie mislukt ({e}); verder met kruipstoten.")
        self._jog(goal, abs(power), tolerance)       # corrigeert alleen wat de driver nog overlaat (> tolerance)

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
            self._guard(direction * p)
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
        self._leerwaarden_bewaren()
        self.driver_fabrieksinstelling()
        try:
            self.send("MMW !M 0 0")
            self.send("MMW !EX")
        finally:
            self.running = False
            self.sock.close()
            self.reader.join(1.0)



@contextmanager
def robot_session(record=True, suffix="", ip=ROBOT_IP, port=ROBOT_PORT, live=True, mqtt=False,
                  mqtt_host="127.0.0.1", network=True, live_page=None):
    """live_page: LiveView van een webserver die al draait (bediening.py); dan start hier geen
    eigen server en wacht het script na de rit niet op Enter."""
    path = None
    f = None
    robot = None
    view = None
    page = live_page  # view.LiveView achter de webpagina (voor de knop Analyse)
    error = None
    try:
        writer = None
        if record:
            directory = Path(__file__).resolve().parents[2] / "ritten"
            directory.mkdir(exist_ok=True)
            path = directory / f"rit_{datetime.now():%Y%m%d_%H%M%S_%f}{suffix}.csv"
        name = path.stem if path else f"rit_{datetime.now():%Y%m%d_%H%M%S}_zonder_csv"
        if live_page is not None:
            view = live_page
            view.name, view.csv_path = name, path
        elif live:   # eerst de pagina: is de poort bezet, dan blijft er geen lege CSV achter
            from jaguar.live import view as live_view
            view = page = live_view.start(name, network=network, csv_path=path)
        if record:
            f = path.open("w", newline="", encoding="utf-8")
            writer = csv.writer(f)
            writer.writerow(["t", "fase", "vermogen", "regel"])
        if mqtt:
            from jaguar.live.mqtt_stream import MqttStream
            view = MqttStream(name, view, host=mqtt_host)   # rekent via dezelfde LiveView en publiceert naar MQTT/Grafana
        robot = Jaguar(ip, port, writer=writer, live=view)
        yield robot
    except BaseException as e:
        error = e
        raise
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
        if page is not None and path is not None:
            page.csv_ready = True
        if live_page is None and page is not None and path is not None and not isinstance(error, KeyboardInterrupt):
            # De webserver stopt met het script: openhouden zodat de knop Analyse en het dashboard werken.
            if error is not None:
                print(f"Rit afgebroken: {error}")
            print("Live pagina blijft open: klik op Analyse voor het dashboard. Enter (of Ctrl+C) = afsluiten.")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass


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
    ap.add_argument("--network", action=argparse.BooleanOptionalAction, default=True,
                    help="live pagina ook bereikbaar voor andere apparaten op het netwerk (standaard aan)")
    ap.add_argument("--mqtt", action=argparse.BooleanOptionalAction, default=False, help="live naar MQTT/Grafana sturen (zie grafana/README.md)")
    ap.add_argument("--mqtt-host", default="127.0.0.1",
                    help="pc waarop Grafana/Docker draait; bv. 192.168.0.104 als dat een andere laptop is")


def run_segment(robot, args, power):
    """Rijrichting = teken van power × teken van distance; zo werkt zowel --power -250 als --distance -3."""
    if args.duration:
        robot.drive(power, args.duration, args.ramp)
    else:
        sign = -1 if args.distance < 0 else 1
        robot.drive_distance(sign * power, abs(args.distance), args.ramp, args.creep, tolerance=args.tolerance / 1000)
