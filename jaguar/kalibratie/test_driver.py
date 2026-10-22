"""Vraagt de motordrivers (MM0 voor, MM1 achter) welke regelmodus ze hebben. Laat NIETS bewegen.

  python -m jaguar.kalibratie.test_driver

De drivers praten Roboteq (!M, !MG, !EX, C=, FF=): daar bestaan naast open loop ook
snelheidsregeling en positieregeling in de driver zelf (1000x per seconde, zonder wifi).
Dit script leest alleen instellingen uit (een ~ vooraan leest een instelling, een ? een meetwaarde). Roboteq-namen:
  MMOD  motormodus: 0 open loop, 1 snelheid met encoder, 2 snelheid met encoder (anders), 3 positie
  EMOD  waarvoor elke encoder gebruikt wordt
  EPPR  encoderpulsen per omwenteling
  KP/KI/KD  PID voor de snelheids- of positieregeling
  MXRPM maximaal toerental dat 100 % betekent
  MAC/MDEC  versnelling en vertraging
Het antwoord komt als `NAAM=waarde`. Een `-` of niets terug = die vraag kent de driver niet.
"""
import argparse
import socket
import time

ROBOT_IP = "192.168.0.60"
ROBOT_PORT = 10001
VRAGEN = ["MMOD", "EMOD", "EPPR", "KP", "KI", "KD", "ICAP", "MXRPM", "MAC", "MDEC", "MVEL", "CLERD", "ALIM", "MXPF", "MXPR"]


def main():
    ap = argparse.ArgumentParser(description="Welke regelmodus hebben de motordrivers? (alleen lezen)")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--port", type=int, default=ROBOT_PORT)
    args = ap.parse_args()
    sock = socket.create_connection((args.ip, args.port), timeout=3)
    sock.settimeout(0.3)
    buf = b""
    antwoorden = {}

    def lees(seconden):
        nonlocal buf
        t0 = time.time()
        while time.time() - t0 < seconden:
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            buf += data
            *lines, buf = buf.split(b"\r\n")
            for line in lines:
                text = line.decode("ascii", "ignore")
                if text.startswith(("MM0 ", "MM1 ")) and "=" in text and text.split(" ", 1)[1].split("=")[0] in VRAGEN:
                    antwoorden.setdefault(text.split(" ", 1)[0], []).append(text.split(" ", 1)[1])

    sock.sendall(b"PING\r\n")
    for board in ("MM0", "MM1"):
        for naam in VRAGEN:
            sock.sendall(f"{board} ~{naam}\r\n".encode())
            sock.sendall(b"PING\r\n")
            lees(0.25)
    lees(0.5)
    sock.close()
    for board, regels in sorted(antwoorden.items()):
        print(f"\n{board}:")
        for r in regels:
            print("  ", r)
    if not antwoorden:
        print("Geen antwoorden: de drivers kennen deze vragen niet, of ze komen niet door de controller.")


if __name__ == "__main__":
    main()
