"""Fase 1 - Jaguar 4x4: rechtdoor heen en terug rijden.

Protocol (uit de C#-broncode van de Dr Robot Jaguar Control-app):
  - TCP 192.168.0.60:10001, elk commando eindigt op CRLF
  - "PING" minstens elke 200 ms, anders stopt de robot (watchdog)
  - "MMW !M <links> <-rechts>"  open-loop vermogen, bereik -1000..1000
  - "MMW !MG" = E-Stop vrijgeven, "MMW !EX" = E-Stop activeren

PC-IP op de DriJaguar-wifi moet 192.168.0.104/24 zijn.
"""
import argparse
import csv
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
import socket
import threading
import time

ROBOT_IP = "192.168.0.60"
ROBOT_PORT = 10001
TICK = 0.1                # PING + motorcommando elke 100 ms
MIN_BATTERY_V = 22.2      # onder deze spanning niet rijden (LiPo)
FAULT_ESTOP = 16          # FF-bit 4 = emergency stop


class Jaguar:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT, writer=None):
        self.writer = writer
        self.t0 = time.perf_counter()
        self.phase = "rust"
        self.power = 0
        self.sock = socket.create_connection((ip, port), timeout=3)
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.running = True
        self.fault = {}       # "MM0"/"MM1" -> FF
        self.encoder = {}     # "MM0"/"MM1" -> (kanaal1, kanaal2)
        self.battery_v = None
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
                self._parse(line.decode("ascii", "ignore"))

    def _parse(self, line):
        if self.writer is not None:
            self.writer.writerow([f"{time.perf_counter() - self.t0:.4f}", self.phase, self.power, line])
        parts = line.split(" ", 1)
        if len(parts) != 2 or parts[0] not in ("MM0", "MM1"):
            return
        board, value = parts
        if value.startswith("FF="):
            self.fault[board] = int(value[3:])
        elif value.startswith("C="):
            self.encoder[board] = tuple(int(v) for v in value[2:].split(":"))
        elif value.startswith("V="):
            self.battery_v = int(value[2:].split(":")[1]) / 10

    def drive(self, power, seconds, ramp=0.5):
        """Rechtdoor rijden met vermogen `power` (-1000..1000), met zachte op- en afbouw."""
        t0 = time.time()
        while (t := time.time() - t0) < seconds:
            scale = min(1.0, t / ramp, (seconds - t) / ramp)
            p = int(power * scale)
            self.power = p
            self.send("PING")
            self.send(f"MMW !M {p} {-p}")
            time.sleep(TICK)
        self.power = 0
        self.send("MMW !M 0 0")

    def hold(self, seconds):
        """Stilstaan maar de watchdog levend houden."""
        self.power = 0
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.send("PING")
            self.send("MMW !M 0 0")
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
def robot_session(record=True, suffix=""):
    path = None
    f = None
    robot = None
    try:
        writer = None
        if record:
            directory = Path(__file__).parent / "ritten"
            directory.mkdir(exist_ok=True)
            path = directory / f"rit_{datetime.now():%Y%m%d_%H%M%S_%f}{suffix}.csv"
            f = path.open("w", newline="", encoding="utf-8")
            writer = csv.writer(f)
            writer.writerow(["t", "fase", "vermogen", "regel"])
        robot = Jaguar(writer=writer)
        yield robot
    finally:
        try:
            if robot is not None:
                robot.close()
                print("Motoren 0, E-Stop terug actief.")
        finally:
            if f is not None:
                f.close()
                print(f"Log: {path}")


def prepare(robot, rest):
    robot.hold(rest)
    print(f"Batterij: {robot.battery_v} V, fault flags: {robot.fault}")
    if robot.battery_v is not None and robot.battery_v < MIN_BATTERY_V:
        raise SystemExit(f"Batterij te laag ({robot.battery_v} V), niet rijden.")
    robot.phase = "vrijgeven"
    robot.send("MMW !MG")
    robot.hold(1.0)
    if any(ff & FAULT_ESTOP for ff in robot.fault.values()):
        raise SystemExit(f"E-Stop blijft actief (FF={robot.fault}). Check de fysieke noodstop.")


def positive_seconds(value):
    value = float(value)
    if not 0 < value < float("inf"):
        raise argparse.ArgumentTypeError("tijd moet positief en eindig zijn")
    return value


def main():
    ap = argparse.ArgumentParser(description="Fase 1: heen en terug rijden")
    ap.add_argument("--power", type=int, default=250, help="motorcommando 0..1000, geen snelheid in m/s")
    ap.add_argument("--duration", type=positive_seconds, default=2.0, help="seconden per richting")
    ap.add_argument("--pause", type=positive_seconds, default=1.5)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True, help="CSV opslaan (standaard aan); --no-csv schakelt uit")
    args = ap.parse_args()
    if not 0 <= args.power <= 1000:
        ap.error("--power moet tussen 0 en 1000 liggen")
    with robot_session(args.csv, "_heen_terug") as robot:
        prepare(robot, args.rest)
        robot.phase = "heen"
        robot.drive(args.power, args.duration)
        robot.phase = "pauze"
        robot.hold(args.pause)
        robot.phase = "terug"
        robot.drive(-args.power, args.duration)
        robot.phase = "uitbollen"
        robot.hold(2.0)


if __name__ == "__main__":
    main()
