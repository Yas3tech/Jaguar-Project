"""Fase 1 - Jaguar 4x4: rechtdoor heen en terug rijden.

Protocol (uit de C#-broncode van de Dr Robot Jaguar Control-app):
  - TCP 192.168.0.60:10001, elk commando eindigt op CRLF
  - "PING" minstens elke 200 ms, anders stopt de robot (watchdog)
  - "MMW !M <links> <-rechts>"  open-loop vermogen, bereik -1000..1000
  - "MMW !MG" = E-Stop vrijgeven, "MMW !EX" = E-Stop activeren

PC-IP op de DriJaguar-wifi moet 192.168.0.104/24 zijn.
"""
import argparse
import socket
import threading
import time

ROBOT_IP = "192.168.0.60"
ROBOT_PORT = 10001
TICK = 0.1                # PING + motorcommando elke 100 ms
MIN_BATTERY_V = 22.2      # onder deze spanning niet rijden (LiPo)
FAULT_ESTOP = 16          # FF-bit 4 = emergency stop


class Jaguar:
    def __init__(self, ip=ROBOT_IP, port=ROBOT_PORT):
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
            self.send("PING")
            self.send(f"MMW !M {p} {-p}")
            time.sleep(TICK)
        self.send("MMW !M 0 0")

    def hold(self, seconds):
        """Stilstaan maar de watchdog levend houden."""
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


def main():
    ap = argparse.ArgumentParser(description="Fase 1: heen en terug rijden")
    ap.add_argument("--power", type=int, default=250, help="open-loop vermogen 0..1000 (standaard 250)")
    ap.add_argument("--duration", type=float, default=2.0, help="seconden per richting (standaard 2.0)")
    ap.add_argument("--pause", type=float, default=1.5, help="seconden stilstand tussen heen en terug")
    args = ap.parse_args()
    power = max(0, min(1000, args.power))

    robot = Jaguar()
    try:
        robot.hold(1.5)  # status binnenhalen
        print(f"Batterij: {robot.battery_v} V, fault flags: {robot.fault}")
        if robot.battery_v is not None and robot.battery_v < MIN_BATTERY_V:
            raise SystemExit(f"Batterij te laag ({robot.battery_v} V < {MIN_BATTERY_V} V), niet rijden.")

        robot.send("MMW !MG")
        robot.hold(1.0)
        if any(ff & FAULT_ESTOP for ff in robot.fault.values()):
            raise SystemExit(f"E-Stop blijft actief (FF={robot.fault}). Check de fysieke noodstop.")
        start = dict(robot.encoder)
        print(f"E-Stop vrij. Encoders start: {start}")

        print(f"Heen: vermogen {power}, {args.duration} s")
        robot.drive(power, args.duration)
        robot.hold(args.pause)
        mid = dict(robot.encoder)
        print(f"Encoders na heen: {mid}")

        print(f"Terug: vermogen {-power}, {args.duration} s")
        robot.drive(-power, args.duration)
        robot.hold(1.0)
        print(f"Encoders einde: {robot.encoder}")
    finally:
        robot.close()
        print("Motoren 0, E-Stop terug actief.")


if __name__ == "__main__":
    main()
