"""Fase 1 - Jaguar 4x4: rechtdoor rijden en alle sensordata loggen.

Volgorde: stilstand (gyro-bias en niveau in rust meten), E-Stop vrijgeven,
rechtdoor rijden, uitbollen, E-Stop terug actief.
Elke ontvangen regel (IMU '#...', GPS '$GPRMC', motordrivers 'MM0/MM1 ...')
wordt met tijdstempel, fase en gecommandeerd vermogen naar een CSV geschreven.
Analyse gebeurt achteraf met analyse_rit.py.
"""
import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from fase1_heen_en_terug import FAULT_ESTOP, MIN_BATTERY_V, TICK, Jaguar

RIT_DIR = Path(__file__).parent / "ritten"


class LoggingJaguar(Jaguar):
    def __init__(self, writer):
        self.writer = writer
        self.t0 = time.perf_counter()
        self.phase = "rust"
        self.power = 0
        super().__init__()

    def _parse(self, line):
        self.writer.writerow([f"{time.perf_counter() - self.t0:.4f}", self.phase, self.power, line])
        super()._parse(line)

    def drive(self, power, seconds, ramp=0.5):
        t0 = time.time()
        while (t := time.time() - t0) < seconds:
            scale = min(1.0, t / ramp, (seconds - t) / ramp)
            self.power = int(power * scale)
            self.send("PING")
            self.send(f"MMW !M {self.power} {-self.power}")
            time.sleep(TICK)
        self.power = 0
        self.send("MMW !M 0 0")


def main():
    ap = argparse.ArgumentParser(description="Fase 1: rechtdoor rijden met IMU-logging")
    ap.add_argument("--power", type=int, default=250, help="open-loop vermogen 0..1000 (standaard 250)")
    ap.add_argument("--duration", type=float, default=5.0, help="rijtijd in seconden (standaard 5.0)")
    ap.add_argument("--rest", type=float, default=3.0, help="seconden stilstand vooraf (bias/niveau)")
    args = ap.parse_args()
    power = max(-1000, min(1000, args.power))

    RIT_DIR.mkdir(exist_ok=True)
    path = RIT_DIR / f"rit_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "fase", "vermogen", "regel"])
        robot = LoggingJaguar(writer)
        try:
            robot.hold(args.rest)
            print(f"Batterij: {robot.battery_v} V, fault flags: {robot.fault}")
            if robot.battery_v is not None and robot.battery_v < MIN_BATTERY_V:
                raise SystemExit(f"Batterij te laag ({robot.battery_v} V), niet rijden.")

            robot.phase = "vrijgeven"
            robot.send("MMW !MG")
            robot.hold(1.0)
            if any(ff & FAULT_ESTOP for ff in robot.fault.values()):
                raise SystemExit(f"E-Stop blijft actief (FF={robot.fault}).")

            print(f"Rijden: vermogen {power}, {args.duration} s, encoders start {robot.encoder}")
            robot.phase = "rijden"
            robot.drive(power, args.duration)
            robot.phase = "uitbollen"
            robot.hold(2.0)
            print(f"Encoders einde: {robot.encoder}")
        finally:
            robot.close()
            robot.reader.join(1.0)  # reader stoppen voor het CSV-bestand sluit
            print("Motoren 0, E-Stop terug actief.")
    print(f"Log: {path}")


if __name__ == "__main__":
    main()
