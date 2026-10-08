"""Proeven voor een exacte afstand op elke ondergrond (zie README, "Afstand op elke ondergrond").

1. Robot OP BLOKKEN (wielen vrij van de grond):
     python test_sondewiel.py adres
   Kunnen voor- (MM0) en achterdriver (MM1) apart rijden? Lopen de wielen vrij uit bij vermogen 0,
   of remt de driver ze af? Alleen als beide "ja / vrij" zijn, kunnen de achterwielen als meetwiel dienen.

2. Op de grond (eerst de mat, dan gras/grind), met de rolmeter erbij:
     python test_sondewiel.py rijden --distance 3
   Alleen de voorwielen trekken; de achterwielen krijgen geen vermogen en rollen mee met de grond.
   Ze slippen dus niet door aandrijving en meten de echte afstand. Het script stopt op de afstand
   van de achterwielen en toont hoeveel de voorwielen slipten. Meet daarna met de rolmeter:
     python kalibreer.py <gemeten> --vergelijk

3. Op de grond, met een rechte lijn (tape) langs de robot:
     python test_sondewiel.py draai --turns 2
   Draait ter plaatse tot de gyro 720° telt. Geef daarna de echte hoek in (bv. 712); de gyroschaal
   komt in kalibratie.json en geldt voor de rijscripts en de analyse.
"""
import argparse
import json
import time

from analyse_rit import CALIBRATION, KALIBRATIE, M_PER_CNT
from fase1_heen_en_terug import TICK, positive_seconds, prepare, robot_session

ROLL_FREE_M = 0.03        # na vermogen 0 nog zoveel uitlopen (op blokken) = de driver remt niet


def wheels(robot):
    """Afstand (m) per wiel: voor links, voor rechts, achter links, achter rechts (vooruit positief)."""
    (fl, fr), (rl, rr) = robot.encoder["MM0"], robot.encoder["MM1"]
    return [fl * M_PER_CNT, -fr * M_PER_CNT, rl * M_PER_CNT, -rr * M_PER_CNT]


def spin_wheels(robot, cmd, power, seconds):
    """Commando `cmd` (bv. "MM0 !M") met vermogen op links en rechts, watchdog levend houden."""
    robot.power = power
    t0 = time.time()
    while time.time() - t0 < seconds:
        robot.send("PING")
        robot.send(f"{cmd} {power} {-power}")
        time.sleep(TICK)
    robot.send("MMW !M 0 0")
    robot.power = 0


def adres(robot, args):
    prepare(robot, args.rest)
    moved = {}
    for board in ("MM0", "MM1"):
        robot.phase = f"adres {board}"
        w0 = wheels(robot)
        spin_wheels(robot, f"{board} !M", args.power, 1.5)
        robot.hold(1.0)
        moved[board] = [b - a for a, b in zip(w0, wheels(robot))]
        print(f"{board} !M {args.power}: voor L/R {moved[board][0]:+.3f} {moved[board][1]:+.3f} m · "
              f"achter L/R {moved[board][2]:+.3f} {moved[board][3]:+.3f} m")
    robot.phase = "uitloop"
    spin_wheels(robot, "MMW !M", args.power, 1.5)
    w0, t0 = wheels(robot), time.time()
    last, last_change = w0, t0
    while time.time() - t0 < 3.0:              # vermogen 0: hoe ver en hoe lang lopen de wielen nog?
        robot._motor(0)
        time.sleep(TICK)
        w = wheels(robot)
        if any(abs(a - b) > M_PER_CNT / 2 for a, b in zip(w, last)):
            last, last_change = w, time.time()
    roll = [abs(b - a) for a, b in zip(w0, last)]
    print(f"uitloop na vermogen 0: {' '.join(f'{r * 100:.1f}' for r in roll)} cm (FL FR RL RR), "
          f"stil na {last_change - t0:.1f} s")

    alleen = lambda m, eigen: all(abs(m[i]) > 0.05 for i in eigen) and all(abs(m[i]) < 0.01 for i in range(4) if i not in eigen)
    apart = alleen(moved["MM0"], (0, 1)) and alleen(moved["MM1"], (2, 3))
    vrij = min(roll[2], roll[3]) > ROLL_FREE_M
    print()
    if apart:
        print("Adressering: OK, MM0 en MM1 rijden apart.")
    elif all(abs(x) < 0.01 for m in moved.values() for x in m):
        print("Adressering: NEE, 'MM0 !M' / 'MM1 !M' laten niets bewegen. Sondewiel kan zo niet.")
    else:
        print("Adressering: NEE, een driveradres stuurt ook andere wielen aan. Sondewiel kan zo niet.")
    print("Uitloop achterwielen:", "vrij (driver remt niet)" if vrij else
          "kort: de driver remt bij vermogen 0, of de overbrenging heeft veel wrijving. Proef 2 op de mat toont of ze meerollen.")
    if apart:
        print("Volgende stap: python test_sondewiel.py rijden --distance 3 (eerst op de mat).")


def rijden(robot, args):
    prepare(robot, args.rest)
    robot.phase = "rijden"
    w0 = wheels(robot)
    moved = robot.drive_distance(args.power, args.distance, args.ramp, args.creep)
    robot.phase = "uitbollen"
    robot.hold(2.0)
    d = [b - a for a, b in zip(w0, wheels(robot))]
    front, rear = (d[0] + d[1]) / 2, (d[2] + d[3]) / 2
    print()
    print(f"voorwielen (aangedreven) {front:+.4f} m · achterwielen (vrij) {rear:+.4f} m · gestopt op {moved:+.4f} m")
    if abs(rear) < 0.5 * abs(front):
        print("De achterwielen rolden niet mee (remmen of slepen): als meetwiel onbruikbaar.")
    elif abs(front) > 0:
        print(f"Slip van de voorwielen: {(front - rear) / front * 100:+.1f} % (achterwielen = grond, als ze vrij rollen).")
    print("Meet nu met de rolmeter en vergelijk: python kalibreer.py <gemeten_m> --vergelijk")


def draai(robot, args):
    prepare(robot, args.rest)
    if not robot.odo.klaar:
        raise SystemExit("Geen gyrodata: draaiproef onmogelijk.")
    robot.phase = "draai"
    target = 360.0 * args.turns
    t0 = time.time()
    while True:
        ramp = min(1.0, (time.time() - t0) / 1.0)
        # Uitlopen na vermogen 0 voorspellen met de draaisnelheid (≈ 0,3 s remweg).
        if robot.heading + robot.yaw_rate * 0.3 >= target or time.time() - t0 > 20 * args.turns:
            break
        robot._motor(0, int(args.power * ramp))      # links achteruit, rechts vooruit = naar links
        robot.power = args.power                     # niet als rust tellen voor de gyrobias
        time.sleep(TICK)
    robot.phase = "uitbollen"
    robot.hold(2.5)
    return robot.heading


def main():
    ap = argparse.ArgumentParser(description="Proeven: driveradressering, vrijlopende meetwielen, gyroschaal")
    ap.add_argument("proef", choices=("adres", "rijden", "draai"))
    ap.add_argument("--power", type=int, help="vermogen (standaard 150 adres, 200 rijden, 350 draai)")
    ap.add_argument("--distance", type=float, default=3.0, help="rijden: meter volgens de achterwielen")
    ap.add_argument("--turns", type=float, default=2.0, help="draai: aantal rondes")
    ap.add_argument("--ramp", type=float, default=1.5)
    ap.add_argument("--creep", type=int, default=80, help="rijden: kruipvermogen (2 wielen trekken)")
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--ip", default="192.168.0.60")
    ap.add_argument("--port", type=int, default=10001)
    args = ap.parse_args()
    args.power = args.power or {"adres": 150, "rijden": 200, "draai": 350}[args.proef]
    if not 0 < args.power <= 600:
        ap.error("--power moet tussen 1 en 600 liggen")
    with robot_session(True, f"_{args.proef}", args.ip, args.port) as robot:
        if args.proef == "rijden":
            robot.achter_vrij = True
            robot.odo.meetwielen = "achter"
        result = {"adres": adres, "rijden": rijden, "draai": draai}[args.proef](robot, args)
    if args.proef == "draai":
        print(f"\nDe gyro telde {result:.1f}°.")
        echt = float(input("Hoeveel graden draaide de robot echt (bv. 712 of 725)? "))
        schaal = KALIBRATIE.get("gyro_schaal", 1.0) * echt / result
        data = dict(KALIBRATIE, gyro_schaal=round(schaal, 4))
        CALIBRATION.write_text(json.dumps(data, indent=2), encoding="utf-8")
        print(f"Gyroschaal {KALIBRATIE.get('gyro_schaal', 1.0):.4f} -> {schaal:.4f}, opgeslagen in {CALIBRATION.name}")


if __name__ == "__main__":
    main()
