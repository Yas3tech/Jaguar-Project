"""Nep-robot om zonder Jaguar te testen: speelt een opgenomen rit-CSV af via TCP.

Gebruik (twee terminals):
  python nep_robot.py                                   # standaard testdata/demo_rit.csv
  python fase1_rechtdoor_imu.py --ip 127.0.0.1 --no-csv --duration 5

De nep-robot luistert op 127.0.0.1:10001, net als de echte op 192.168.0.60:10001, en stuurt
elke ontvangen regel uit de CSV opnieuw door in het oorspronkelijke tempo. Hij negeert
de motorcommando's, behalve de E-Stop: na "MMW !MG" meldt hij FF=0 (vrij), na "MMW !EX"
weer FF=16. Na elke rit wacht hij op de volgende verbinding. Stoppen met Ctrl+C.
"""
import argparse
import csv
import socket
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent


def serve(conn, rows):
    estop = [True]

    def listen():
        buf = b""
        while True:
            try:
                data = conn.recv(4096)
            except OSError:
                return
            if not data:
                return
            buf += data
            *lines, buf = buf.split(b"\r\n")
            for line in lines:
                if line == b"MMW !MG":
                    estop[0] = False
                elif line == b"MMW !EX":
                    estop[0] = True

    threading.Thread(target=listen, daemon=True).start()
    t0 = time.perf_counter()
    for r in rows:
        line = r["regel"]
        if line[:4] in ("MM0 ", "MM1 ") and line[4:7] == "FF=":
            line = f"{line[:4]}FF={16 if estop[0] else 0}"
        while time.perf_counter() - t0 < float(r["t"]):
            time.sleep(0.002)
        try:
            conn.sendall((line + "\r\n").encode("ascii"))
        except OSError:
            return            # script heeft de verbinding gesloten
    # Opname op: robot blijft "staan" tot het script de verbinding sluit.
    while True:
        try:
            if not conn.recv(4096):
                return
        except OSError:
            return


def main():
    ap = argparse.ArgumentParser(description="Nep-robot: speelt een rit-CSV af op 127.0.0.1:10001")
    ap.add_argument("csv", nargs="?", default=HERE / "testdata" / "demo_rit.csv", type=Path)
    ap.add_argument("--port", type=int, default=10001)
    args = ap.parse_args()
    rows = list(csv.DictReader(open(args.csv, newline="", encoding="utf-8")))
    srv = socket.create_server(("127.0.0.1", args.port))
    print(f"Nep-robot klaar op 127.0.0.1:{args.port} met {args.csv.name} ({len(rows)} regels, {float(rows[-1]['t']):.0f} s). Ctrl+C = stoppen.")
    while True:
        conn, _ = srv.accept()
        print(f"{time.strftime('%H:%M:%S')}  script verbonden, rit wordt afgespeeld")
        with conn:
            serve(conn, rows)
        print(f"{time.strftime('%H:%M:%S')}  rit klaar, wacht op volgende verbinding")


if __name__ == "__main__":
    main()
