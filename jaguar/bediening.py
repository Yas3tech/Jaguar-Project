"""Robot besturen vanaf de live pagina: parcours samenstellen, starten en stoppen in de browser.

  python -m jaguar.bediening                   echte robot (192.168.0.60)
  python -m jaguar.bediening --ip 127.0.0.1    nep_robot.py

Opent http://127.0.0.1:8765/ met de knop Rijden. Elke rit is een parcours zoals parcours.py
(vooruit, achteruit, links, rechts) en krijgt zijn eigen CSV in ritten/, dus de knop Analyse werkt zoals altijd.
Ritten starten kan alleen op deze laptop; STOP werkt ook vanaf andere apparaten die meekijken.
STOP zet de motoren op 0 en de E-Stop weer aan. De fysieke noodstop blijft de echte noodstop.
"""
import argparse
import sys
import threading

from jaguar.live import view as live_view
from jaguar.robot import ROBOT_IP, ROBOT_PORT, Aborted, prepare, robot_session
from jaguar.parcours import STAPPEN, rijd

MAX_STAPPEN = 30
MAX_METER = 20.0      # per stap; langer is eerder een tikfout
MAX_GRADEN = 720.0
MAX_POWER = 600       # bovengrens vanaf de pagina (≈ 1,9 m/s); hoger kan alleen met parcours.py


class Control:
    """Eén rit tegelijk, in een achtergronddraad; de webserver toont `state` op de pagina."""

    def __init__(self, hub, args):
        self.hub, self.args = hub, args
        self.lock = threading.Lock()
        self.thread = None
        self.robot = None
        self.stop_requested = False
        self.state = {"bezig": False, "resultaat": None, "fout": None}

    def start(self, raw):
        """raw = {"stappen": [["vooruit", 1.0], ...], "power": 250}; geeft (HTTP-status, antwoord)."""
        try:
            stappen, power = self._check(raw)
        except ValueError as e:
            return 400, {"fout": str(e)}
        with self.lock:
            if self.state["bezig"]:
                return 409, {"fout": "Er loopt al een rit."}
            self.stop_requested = False
            view = live_view.LiveView("")
            self.hub.view = view          # de pagina schakelt naar de nieuwe rit
            self.state = {"bezig": True, "resultaat": None, "fout": None}
            self.thread = threading.Thread(target=self._run, args=(view, stappen, power), daemon=True)
            self.thread.start()
        print("Rit vanaf de pagina: " + ", ".join(f"{s} {v:g}" for s, v in stappen) + f" (vermogen {power})")
        return 200, {"ok": True}

    @staticmethod
    def _check(raw):
        if not isinstance(raw, dict) or not isinstance(raw.get("stappen"), list) or not raw["stappen"]:
            raise ValueError("Geen stappen.")
        if len(raw["stappen"]) > MAX_STAPPEN:
            raise ValueError(f"Hoogstens {MAX_STAPPEN} stappen.")
        stappen = []
        for item in raw["stappen"]:
            if not (isinstance(item, list) and len(item) == 2 and item[0] in STAPPEN
                    and isinstance(item[1], (int, float)) and not isinstance(item[1], bool)):
                raise ValueError(f"Ongeldige stap: {item!r}")
            soort, value = STAPPEN[item[0]], float(item[1])
            limit = MAX_METER if soort in ("vooruit", "achteruit") else MAX_GRADEN
            if not 0 < value <= limit:
                raise ValueError(f"{soort} {value:g}: moet tussen 0 en {limit:g} liggen.")
            stappen.append((soort, value))
        power = raw.get("power", 250)
        if not isinstance(power, int) or isinstance(power, bool) or not 50 <= power <= MAX_POWER:
            raise ValueError(f"Vermogen moet tussen 50 en {MAX_POWER} liggen.")
        return stappen, power

    def _run(self, view, stappen, power):
        a = self.args
        result, error = "klaar", None
        try:
            with robot_session(True, "_parcours", a.ip, a.port, mqtt=a.mqtt, mqtt_host=a.mqtt_host,
                               live_page=view) as robot:
                self.robot = robot
                robot.stop_requested = self.stop_requested     # STOP tijdens het verbinden
                prepare(robot, a.rest)
                rijd(robot, stappen, power)
        except Aborted:
            result = "gestopt"
        except OSError as e:
            result, error = "fout", f"Robot niet bereikbaar op {a.ip}:{a.port} ({e})."
        except SystemExit as e:          # batterij te laag, E-Stop blijft actief, geen encoders
            result, error = "fout", str(e)
        except Exception as e:  # noqa: BLE001 - de fout hoort op de pagina, de server moet blijven draaien
            result, error = "fout", f"{type(e).__name__}: {e}"
        finally:
            self.robot = None
            self.state = {"bezig": False, "resultaat": result, "fout": error}
            print(f"Rit {result}" + (f": {error}" if error else "") + ". Klaar voor de volgende rit op de pagina.")

    def stop(self):
        self.stop_requested = True
        robot = self.robot
        if robot is not None:
            robot.stop_requested = True
        return 200, {"ok": True, "bezig": self.state["bezig"]}

    def shutdown(self):
        """Enter in de terminal: een lopende rit eerst stoppen."""
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
    args = ap.parse_args()
    # Een teken dat de console niet kent (bv. → in parcours) mag nooit een rit of de server laten crashen.
    sys.stdout.reconfigure(errors="replace")
    hub = live_view.Hub(live_view.LiveView("nog geen rit"))
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
