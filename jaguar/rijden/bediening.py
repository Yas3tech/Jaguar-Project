"""Robot besturen vanaf de live pagina: parcours samenstellen, starten en stoppen in de browser.

  python -m jaguar.rijden.bediening                   echte robot (192.168.0.60)
  python -m jaguar.rijden.bediening --ip 127.0.0.1    nep_robot.py

Opent http://127.0.0.1:8765/ met de knop Rijden. Elke rit is een parcours zoals parcours.py
(vooruit, achteruit, links, rechts) en krijgt zijn eigen CSV in ritten/, dus de knop Analyse werkt zoals altijd.
Ritten starten kan alleen op deze laptop; STOP werkt ook vanaf andere apparaten die meekijken.
De knop GPS meten (ontwerper, Buiten) verbindt met het robot zonder de E-Stop vrij te geven en toont hoe goed de
GPS op die plek is (GpsKwaliteit in jaguar/navigatie/gps.py). Een rit starten stopt die meting eerst.
STOP zet de motoren op 0 en de E-Stop weer aan. De fysieke noodstop blijft de echte noodstop.

Tijdens een parcours:
  - Pauze: het robot stopt; na Hervatten rijdt de stap verder naar hetzelfde doel. Pauzeren mag ook vanaf een
    ander apparaat (zoals STOP), hervatten alleen op deze laptop.
  - Terug naar start: het parcours stopt en het robot rijdt in een rechte lijn terug naar de start en draait naar
    de beginrichting (naar_start in parcours.py). Na de rit kan dat ook nog, met de laatste positie volgens de
    odometrie: dan mag niemand het robot intussen verplaatst hebben.
  - Batterij: zakt de spanning tijdens het rijden onder BAT_TERUG_V (robot.py), dan breekt het parcours af en rijdt
    het robot vanzelf terug naar de start; onder BAT_NOOD_V stopt het helemaal.
Handmatig rijden (knop Handmatig): de pagina stuurt 10 keer per seconde de stand van de pijltjes, de knoppen op het
scherm of een gamepad. Komt er 0,4 s niets (pagina dicht, wifi weg), dan staat het robot stil (dodemansknop).
"""
import argparse
import math
import sys
import threading
import time

from jaguar.live import view as live_view
from jaguar.navigatie.gps import GpsKwaliteit, checksum_ok, parse_rmc
from jaguar.navigatie.route import boog_naar_stappen
from jaguar.kern.robot import (BAT_NOOD_V, LOOP, MIN_BATTERY_V, ROBOT_IP, ROBOT_PORT, TURN_TOP, Aborted, BatterijLaag, BatterijLeeg,
                               Jaguar, NaarStart, Obstakel, Stalled, prepare, robot_session)
from jaguar.rijden.parcours import STAPPEN, naar_start, rijd

MAX_STAPPEN = 40         # wat de gebruiker opgeeft; een boog telt als één stap
MAX_UITGEROLD = 400      # na het uitrollen van bogen in korte stukjes
MAX_RADIUS = 30.0
MIN_RADIUS_VLOEIEND = 0.5
MAX_METER = 20.0      # per stap; langer is eerder een tikfout
MAX_GRADEN = 720.0
MAX_POWER = 600       # bovengrens vanaf de pagina (≈ 1,9 m/s); hoger kan alleen met parcours.py
GPS_ZONDER_PAGINA = 30   # s: vraagt geen pagina meer naar de GPS-meting, dan stopt ze (pagina dicht)
HAND_DODEMAN_S = 0.4     # s zonder stuurbericht van de pagina = stilstaan
HAND_ZONDER_PAGINA = 30  # s zonder stuurbericht = de handmatige rit stopt
HAND_OPTREK = 500        # vermogen per seconde bij het optrekken …
HAND_REM = 1500          # … en bij loslaten of afremmen
HAND_STUUR = 1500        # stuurwaarde per seconde
HAND_BOCHT = 0.8         # in een bocht: de binnenkant draait met 20 % van het vermogen
SUFFIX = {"parcours": "_parcours", "terug": "_terug", "handmatig": "_handmatig"}


def samenstellen(oorsprong, x, y, koers):
    """Positie (x, y, koers) uit het stelsel van een rit naar het stelsel van de start, als die rit begon op
    `oorsprong` (x, y, koers in het stelsel van de start)."""
    ox, oy, ok = oorsprong
    c, s = math.cos(math.radians(ok)), math.sin(math.radians(ok))
    return ox + c * x - s * y, oy + s * x + c * y, ok + koers


class GpsLuisteraar:
    """Krijgt elke regel van het robot (zoals de live weergave) en geeft de GPRMC-zinnen aan de meting,
    ook die zonder positie (geen fix)."""

    def __init__(self, meting):
        self.meting = meting

    def feed(self, t, phase, power, line):
        if line.startswith("$GPRMC") and checksum_ok(line):
            self.meting.voeg_toe(time.monotonic(), parse_rmc(line))


class Control:
    """Eén rit tegelijk, in een achtergronddraad; de webserver toont `state` op de pagina."""

    def __init__(self, hub, args):
        self.hub, self.args = hub, args
        self.lock = threading.Lock()
        self.thread = None
        self.robot = None
        self.stop_requested = False
        self.state = {"bezig": False, "soort": None, "resultaat": None, "fout": None}
        # Positie (x, y, koers) t.o.v. de start van het laatste parcours of de laatste handmatige rit, na de rit.
        # Een terugrit begint daar: de terugrit zelf rekent dus vanaf `oorsprong` in plaats van vanaf zijn eigen start.
        self.thuis = None
        self.power = 250
        self.hand = {"v": 0.0, "w": 0.0, "t": 0.0, "max": 250}
        self.hand_klaar = threading.Event()
        self.gps_thread = None
        self.gps_stop = threading.Event()
        self.gps_meting = None
        self.gps_fout = None
        self.gps_gevraagd = 0.0

    def start(self, raw):
        """raw = {"stappen": [["vooruit", 1.0], ...], "power": 250}; geeft (HTTP-status, antwoord)."""
        try:
            stappen, power = self._check(raw)
        except ValueError as e:
            return 400, {"fout": str(e)}
        return self._begin("parcours", stappen, power, (0.0, 0.0, 0.0))

    def _begin(self, soort, stappen, power, oorsprong):
        """Een rit starten in de achtergrond: "parcours", "terug" (naar de start) of "handmatig"."""
        with self.lock:
            if self.state["bezig"]:
                return 409, {"fout": "Er loopt al een rit."}
            self._gps_stoppen()            # één verbinding met het robot tegelijk
            self.stop_requested = False
            self.power = power
            view = live_view.LiveView("")
            self.hub.view = view          # de pagina schakelt naar de nieuwe rit
            self.state = {"bezig": True, "soort": soort, "resultaat": None, "fout": None}
            self.thread = threading.Thread(target=self._run, args=(view, soort, stappen, power, oorsprong), daemon=True)
            self.thread.start()
        if stappen:
            print(f"Rit ({soort}) vanaf de pagina: " + ", ".join(" ".join(f"{w:g}" if not isinstance(w, str) else w for w in st)
                                                       for st in stappen) + f" (vermogen {power})")
        else:
            print(f"Rit ({soort}) vanaf de pagina, vermogen hoogstens {power}")
        return 200, {"ok": True}

    def toestand(self):
        """Voor de pagina (elke 0,1 s in /stream)."""
        r = dict(self.state)
        robot = self.robot
        r["gepauzeerd"] = bool(robot is not None and robot.pause_requested)
        r["terug_gevraagd"] = bool(robot is not None and robot.terug_requested)
        r["thuis"] = None
        if not r["bezig"] and self.thuis is not None:
            stappen = naar_start(*self.thuis)
            if stappen:
                r["thuis"] = {"afstand": round(math.hypot(self.thuis[0], self.thuis[1]), 2), "stappen": stappen}
        return r

    @staticmethod
    def _check(raw):
        if not isinstance(raw, dict) or not isinstance(raw.get("stappen"), list) or not raw["stappen"]:
            raise ValueError("Geen stappen.")
        if len(raw["stappen"]) > MAX_STAPPEN:
            raise ValueError(f"Hoogstens {MAX_STAPPEN} stappen.")
        vloeiend = raw.get("bogen") == "vloeiend"       # anders: bogen als korte stukjes van 10°
        stappen = []
        for item in raw["stappen"]:
            getal = lambda x: isinstance(x, (int, float)) and not isinstance(x, bool) and x == x
            if isinstance(item, list) and len(item) == 3 and item[0] == "boog" and getal(item[1]) and getal(item[2]):
                radius, hoek = float(item[1]), float(item[2])      # hoek: + = naar links
                if not 0.1 <= radius <= MAX_RADIUS or not 0 < abs(hoek) <= MAX_GRADEN:
                    raise ValueError(f"boog {radius:g} m, {hoek:g}°: radius 0,1 tot {MAX_RADIUS:g} m, hoek tot {MAX_GRADEN:g}°.")
                if vloeiend:
                    if radius < MIN_RADIUS_VLOEIEND:
                        raise ValueError(f"vloeiende boog: radius minstens {MIN_RADIUS_VLOEIEND:g} m (kleiner kan niet zonder dat de wielen achteruit moeten).")
                    stappen.append(("boog", radius, hoek))
                else:
                    stappen.extend(boog_naar_stappen(radius, hoek))
                continue
            if not (isinstance(item, list) and len(item) == 2 and item[0] in STAPPEN and getal(item[1])):
                raise ValueError(f"Ongeldige stap: {item!r}")
            soort, value = STAPPEN[item[0]], float(item[1])
            limit = MAX_METER if soort in ("vooruit", "achteruit") else MAX_GRADEN
            if not 0 < value <= limit:
                raise ValueError(f"{soort} {value:g}: moet tussen 0 en {limit:g} liggen.")
            stappen.append((soort, value))
        if len(stappen) > MAX_UITGEROLD:
            raise ValueError(f"Hoogstens {MAX_UITGEROLD} stappen na het uitrollen van bogen.")
        power = raw.get("power", 250)
        if not isinstance(power, int) or isinstance(power, bool) or not 50 <= power <= MAX_POWER:
            raise ValueError(f"Vermogen moet tussen 50 en {MAX_POWER} liggen.")
        return stappen, power

    def _run(self, view, soort, stappen, power, oorsprong):
        a = self.args
        result, error = "klaar", None
        robot = None
        try:
            with robot_session(a.csv, SUFFIX[soort], a.ip, a.port, mqtt=a.mqtt, mqtt_host=a.mqtt_host,
                               live_page=view) as robot:
                self.robot = robot
                robot.stop_requested = self.stop_requested     # STOP tijdens het verbinden
                robot.batterij_terug = soort == "terug"        # naar de start rijden mag tot BAT_NOOD_V
                prepare(robot, a.rest, min_v=BAT_NOOD_V if soort == "terug" else MIN_BATTERY_V)
                try:
                    if soort == "handmatig":
                        self._handmatig(robot)
                    else:
                        rijd(robot, stappen, power, prefix="terug-" if soort == "terug" else "")
                except NaarStart as e:
                    if soort == "handmatig" and isinstance(e, BatterijLaag):
                        robot.melding(f"{e}: handmatig rijden gestopt. Terug naar start kan nog (tot {BAT_NOOD_V:.1f} V).")
                        result, error = "batterij", f"{e}: handmatig rijden gestopt"
                    else:
                        result, error = self._terugrijden(robot, e, power, oorsprong)
            if robot.obstakel:
                result, error = "obstakel", robot.obstakel
        except Aborted:
            result = "gestopt"
        except BatterijLeeg as e:
            result, error = "batterij", str(e)
        except OSError as e:
            result, error = "fout", f"Robot niet bereikbaar op {a.ip}:{a.port} ({e})."
        except SystemExit as e:          # batterij te laag, E-Stop blijft actief, geen encoders
            result, error = "fout", str(e)
        except Exception as e:  # noqa: BLE001 - de fout hoort op de pagina, de server moet blijven draaien
            result, error = "fout", f"{type(e).__name__}: {e}"
        finally:
            if robot is not None and robot.odo.klaar:
                self.thuis = samenstellen(oorsprong, robot.odo.x, robot.odo.y, robot.odo.koers)
            self.robot = None
            self.state = {"bezig": False, "soort": self.state["soort"], "resultaat": result, "fout": error}
            print(f"Rit {result}" + (f": {error}" if error else "") + ". Klaar voor de volgende rit op de pagina.")

    def _terugrijden(self, robot, reden, power, oorsprong):
        """In dezelfde verbinding terug naar de start (knop Terug naar start, of de batterij)."""
        batterij = isinstance(reden, BatterijLaag)
        robot.pause_requested = robot.terug_requested = False
        robot.batterij_terug = True
        self.state = dict(self.state, soort="terug")
        x, y, koers = samenstellen(oorsprong, robot.odo.x, robot.odo.y, robot.target_heading)
        stappen = naar_start(x, y, koers)
        robot.melding(f"{reden}: parcours afgebroken, terug naar de start ({math.hypot(x, y):.2f} m)")
        robot.hold(1.0)
        rijd(robot, stappen, power, prefix="terug-")
        if batterij:
            return "batterij", f"{reden}: parcours afgebroken en terug naar de start gereden"
        return "terug", None

    def pauze(self, raw):
        """raw = {"aan": true/false}: het lopende parcours pauzeren of hervatten."""
        if not isinstance(raw, dict) or not isinstance(raw.get("aan"), bool):
            return 400, {"fout": 'Verwacht {"aan": true} of {"aan": false}.'}
        robot = self.robot
        if not self.state["bezig"] or robot is None or self.state["soort"] == "handmatig":
            return 409, {"fout": "Er loopt geen parcours om te pauzeren."}
        robot.pause_requested = raw["aan"]
        print("Pauze." if raw["aan"] else "Hervatten.")
        return 200, {"ok": True, "gepauzeerd": raw["aan"]}

    def terug(self, raw=None):
        """Terug naar de start: tijdens een rit breekt dat de rit af, daarna start het een terugrit."""
        robot = self.robot
        if self.state["bezig"]:
            if self.state["soort"] == "terug":
                return 409, {"fout": "Het robot rijdt al terug naar de start."}
            if robot is None:
                return 409, {"fout": "Het robot is nog aan het verbinden; probeer zo opnieuw."}
            robot.terug_requested = True
            return 200, {"ok": True}
        if self.thuis is None:
            return 409, {"fout": "Geen startpunt bekend: rijd eerst een parcours (of handmatig)."}
        stappen = naar_start(*self.thuis)
        if not stappen:
            return 409, {"fout": "Het robot staat al op de start."}
        return self._begin("terug", stappen, self.power, self.thuis)

    def handmatig(self, raw):
        """raw = {"aan": true, "max": 250} om handmatig te beginnen, {"aan": false} om te stoppen."""
        if not isinstance(raw, dict) or not isinstance(raw.get("aan"), bool):
            return 400, {"fout": 'Verwacht {"aan": true} of {"aan": false}.'}
        if not raw["aan"]:
            self.hand_klaar.set()
            return 200, {"ok": True}
        top = raw.get("max", 250)
        if not isinstance(top, int) or isinstance(top, bool) or not 50 <= top <= MAX_POWER:
            return 400, {"fout": f"Vermogen moet tussen 50 en {MAX_POWER} liggen."}
        self.hand = {"v": 0.0, "w": 0.0, "t": time.monotonic(), "max": top}
        self.hand_klaar.clear()
        return self._begin("handmatig", None, top, (0.0, 0.0, 0.0))

    def stuur(self, raw):
        """raw = {"v": -1..1 (vooruit +), "w": -1..1 (links +), "max": vermogen}: de stand van de bediening op de pagina."""
        getal = lambda x: isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
        if not isinstance(raw, dict) or not getal(raw.get("v")) or not getal(raw.get("w")):
            return 400, {"fout": 'Verwacht {"v": getal, "w": getal}.'}
        top = raw.get("max", self.hand["max"])
        if not isinstance(top, int) or isinstance(top, bool) or not 50 <= top <= MAX_POWER:
            top = self.hand["max"]
        self.hand = {"v": max(-1.0, min(1.0, float(raw["v"]))), "w": max(-1.0, min(1.0, float(raw["w"]))),
                     "t": time.monotonic(), "max": top}
        return 200, {"ok": True, "bezig": self.state["bezig"] and self.state["soort"] == "handmatig"}

    def _handmatig(self, robot):
        """Rijden met de stand van de bediening op de pagina, tot Klaar, STOP of tot de pagina wegblijft.
        Rechtdoor houdt de gyro de koers vast; de veiligheidsstops van robot.py blijven gelden (kantelen, vastlopen,
        obstakel vooruit, batterij). Na een stop eerst alles loslaten."""
        robot.phase = "handmatig"
        robot.melding("Handmatig rijden: pijltjes of WASD, de knoppen op het scherm of een gamepad")
        p = s = 0.0
        los_nodig, laatste = False, None
        robot.target_heading, robot.line = robot.heading, (robot.odo.x, robot.odo.y)
        t_last = time.perf_counter()
        while not self.hand_klaar.is_set():
            now = time.perf_counter()
            dt, t_last = min(0.2, now - t_last), now
            h = self.hand
            oud = time.monotonic() - h["t"]
            if oud > HAND_ZONDER_PAGINA:
                robot.melding(f"Handmatig: {HAND_ZONDER_PAGINA} s niets van de pagina, gestopt")
                break
            if robot.terug_requested:
                raise NaarStart("terug naar de start (knop op de pagina)")
            v, w = (h["v"], h["w"]) if oud <= HAND_DODEMAN_S else (0.0, 0.0)
            if los_nodig:
                los_nodig = abs(v) >= 0.05 or abs(w) >= 0.05
                v = w = 0.0
            top = h["max"]
            rijden = abs(v) >= 0.05
            doel_p = v * top if rijden else 0.0
            if rijden:
                doel_s = w * HAND_BOCHT * abs(doel_p)
            elif abs(w) >= 0.05:                         # ter plaatse draaien: skid-steer heeft minstens turn_min nodig
                draai_top = max(robot.turn_min, min(TURN_TOP, 1.3 * top))
                doel_s = math.copysign(robot.turn_min + abs(w) * (draai_top - robot.turn_min), w)
            else:
                doel_s = 0.0
            if oud > HAND_DODEMAN_S:
                p = s = 0.0                              # pagina of wifi weg: meteen stil
            else:
                rate = HAND_OPTREK if abs(doel_p) > abs(p) and doel_p * p >= 0 else HAND_REM
                p += max(-rate * dt, min(rate * dt, doel_p - p))
                s += max(-HAND_STUUR * dt, min(HAND_STUUR * dt, doel_s - s))
            pi, steer = int(p), int(s)
            if rijden and abs(w) < 0.05 and pi != 0:    # rechtdoor: koers vasthouden met de gyro
                robot.line = (robot.odo.x, robot.odo.y)
                steer = robot._steer(pi)
            else:
                robot.target_heading = robot.heading
                robot.heading_pid.integral = 0.0
            try:
                robot._guard(pi)
                robot._motor(pi, steer)
                laatste = None
            except Obstakel as e:                        # alleen vooruit geblokkeerd: draaien en achteruit mag
                p = 0.0
                robot._motor(0, 0 if rijden else steer)
                if laatste != "obstakel":
                    robot.melding(f"Handmatig: {e}")
                laatste = "obstakel"
            except Stalled as e:
                p = s = 0.0
                robot._motor(0)
                los_nodig = True
                robot.melding(f"Handmatig: gestopt - {e}. Laat alles los om verder te rijden.")
            time.sleep(LOOP)
        robot.phase = "uitbollen"
        robot.hold(1.0)

    def stop(self):
        self.stop_requested = True
        robot = self.robot
        if robot is not None:
            robot.stop_requested = True
        return 200, {"ok": True, "bezig": self.state["bezig"]}

    def gps(self, raw):
        """raw = {"aan": true/false}: de GPS-meting starten of stoppen; geeft (HTTP-status, antwoord)."""
        if not isinstance(raw, dict) or not isinstance(raw.get("aan"), bool):
            return 400, {"fout": 'Verwacht {"aan": true} of {"aan": false}.'}
        with self.lock:
            if not raw["aan"]:
                self._gps_stoppen()
            elif self.state["bezig"]:
                return 409, {"fout": "Er loopt een rit: de GPS meten kan alleen als het robot stilstaat."}
            elif self.gps_thread is None or not self.gps_thread.is_alive():
                self.gps_meting, self.gps_fout, self.gps_gevraagd = GpsKwaliteit(time.monotonic()), None, time.monotonic()
                self.gps_stop.clear()
                self.gps_thread = threading.Thread(target=self._gps_run, args=(self.gps_meting,), daemon=True)
                self.gps_thread.start()
        return 200, self.gps_rapport()

    def gps_rapport(self):
        """Stand van de GPS-meting voor de pagina; elke vraag houdt de meting levend."""
        self.gps_gevraagd = time.monotonic()
        bezig = self.gps_thread is not None and self.gps_thread.is_alive()
        r = {"bezig": bezig, "fout": self.gps_fout}
        m = self.gps_meting
        if m is not None:         # gestopt: het oordeel van het einde van de meting blijft staan
            r.update(m.rapport(time.monotonic() if bezig or not m.metingen else m.metingen[-1][0]))
        return r

    def _gps_run(self, meting):
        a = self.args
        try:
            robot = Jaguar(a.ip, a.port, live=GpsLuisteraar(meting))
        except OSError as e:
            self.gps_fout = f"Robot niet bereikbaar op {a.ip}:{a.port} ({e})."
            return
        print("GPS meten: verbonden, de E-Stop blijft actief.")
        try:
            while not self.gps_stop.is_set():
                if time.monotonic() - self.gps_gevraagd > GPS_ZONDER_PAGINA:
                    print("GPS meten: geen pagina meer open, gestopt.")
                    break
                robot.hold(0.5)            # watchdog levend houden, vermogen 0
        except OSError as e:
            self.gps_fout = f"Verbinding met het robot verbroken ({e})."
        finally:
            try:
                robot.close()
            except OSError:
                pass                       # verbinding was al weg
            print("GPS meten: gestopt.")

    def _gps_stoppen(self):
        self.gps_stop.set()
        if self.gps_thread is not None:
            self.gps_thread.join(5)

    def shutdown(self):
        """Enter in de terminal: een lopende rit eerst stoppen."""
        self._gps_stoppen()
        self.hand_klaar.set()
        if self.state["bezig"]:
            print("Rit loopt nog: STOP.")
            self.stop()
            if self.thread is not None:
                self.thread.join(10)


def main():
    ap = argparse.ArgumentParser(description="Robot besturen vanaf de live pagina")
    ap.add_argument("--ip", default=ROBOT_IP, help="IP van de robot; 127.0.0.1 = nep_robot.py")
    ap.add_argument("--port", type=int, default=ROBOT_PORT)
    ap.add_argument("--rest", type=float, default=3.0, help="seconden rustmeting vóór elke rit")
    ap.add_argument("--web-port", type=int, default=8765, help="poort van de live pagina")
    ap.add_argument("--network", action=argparse.BooleanOptionalAction, default=True,
                    help="pagina ook bereikbaar voor andere apparaten (meekijken en STOP; starten alleen op deze laptop)")
    ap.add_argument("--mqtt", action=argparse.BooleanOptionalAction, default=False, help="ook naar MQTT/Grafana sturen")
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True, help="elke rit als CSV in ritten/ (standaard aan)")
    args = ap.parse_args()
    # Een teken dat de console niet kent (bv. → in parcours) mag nooit een rit of de server laten crashen.
    sys.stdout.reconfigure(errors="replace")
    if live_view.OPTIONS["obstakel"]:   # ook tussen de ritten: de pagina toont wat de camera ziet (kalibreren)
        try:
            from jaguar.camera import obstakel
            obstakel.get(live_view.OPTIONS["obstakel_stop_m"])
        except ImportError as e:
            print(f"Obstakels: {e} (pip install opencv-python numpy).")
    hub = live_view.Hub(live_view.LiveView("nog geen rit"))
    hub.view.status["fase"] = "geen rit"   # niet "verbinden": er is nog geen verbinding gevraagd
    hub.control = Control(hub, args)
    live_view.serve(hub, args.web_port, network=args.network)
    print(f"Robot: {args.ip}:{args.port}. Klik op Rijden op de pagina. Enter (of Ctrl+C) = afsluiten.")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    hub.control.shutdown()


if __name__ == "__main__":
    main()
