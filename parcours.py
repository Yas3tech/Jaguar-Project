"""Parcours rijden: een reeks stappen vooruit, achteruit en ter plaatse draaien.

  python parcours.py vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5
  python parcours.py v:2 l:90 v:2 l:90 v:2 l:90 v:2          (vierkant)

Stappen: vooruit:<m>, achteruit:<m>, links:<graden>, rechts:<graden> (afgekort v, a, l, r).
Rijden gebruikt de slipbewuste odometrie (zoals fase1_heen_en_terug.py), draaien de gyro.
Na een draai volgt de robot de nieuwe koers; opeenvolgende rechte stukken blijven op dezelfde lijn.
Daarna: python analyse_rit.py, en het dashboard toont de geplande en de gereden route.
Draaien is zo juist als de gyroschaal: kalibreer eerst met  python test_sondewiel.py draai.
"""
import argparse

from fase1_heen_en_terug import add_drive_args, positive_seconds, prepare, robot_session

STAPPEN = {"vooruit": "vooruit", "v": "vooruit", "achteruit": "achteruit", "a": "achteruit",
           "links": "links", "l": "links", "rechts": "rechts", "r": "rechts"}


def stap(text):
    """'vooruit:5' -> ('vooruit', 5.0)."""
    soort, _, waarde = text.lower().partition(":")
    if soort not in STAPPEN or not waarde:
        raise argparse.ArgumentTypeError(f"onbekende stap '{text}': gebruik vooruit:<m>, achteruit:<m>, links:<graden> of rechts:<graden>")
    try:
        value = float(waarde.replace(",", "."))
    except ValueError:
        raise argparse.ArgumentTypeError(f"geen getal in '{text}'") from None
    if not 0 < value < 1000:
        raise argparse.ArgumentTypeError(f"waarde in '{text}' moet positief zijn")
    return STAPPEN[soort], value


def main():
    ap = argparse.ArgumentParser(description="Parcours rijden: vooruit, achteruit, links/rechts draaien")
    ap.add_argument("stappen", nargs="+", type=stap, help="bv. vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5")
    ap.add_argument("--power", type=int, default=250, help="maximaal rijvermogen (0..1000)")
    ap.add_argument("--turn-power", type=int, default=380, help="maximaal vermogensverschil bij draaien")
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.0, help="seconden stilstaan tussen de stappen")
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()
    if not 0 < args.power <= 1000 or not 0 < args.turn_power <= 1000:
        ap.error("--power en --turn-power moeten tussen 1 en 1000 liggen")
    print("Parcours: " + " → ".join(f"{s} {v:g} {'m' if s in ('vooruit', 'achteruit') else '°'}" for s, v in args.stappen))
    with robot_session(args.csv, "_parcours", args.ip, args.port) as robot:
        prepare(robot, args.rest)
        for i, (soort, value) in enumerate(args.stappen, 1):
            if i > 1:
                robot.phase = "pauze"
                robot.hold(args.pause)
            if soort in ("vooruit", "achteruit"):
                robot.phase = f"{i}-{soort}"
                power = args.power if soort == "vooruit" else -args.power
                robot.drive_distance(power, value, args.ramp, args.creep, tolerance=args.tolerance / 1000)
            else:
                robot.phase = f"{i}-draai"
                robot.turn(value if soort == "links" else -value, args.turn_power)
        robot.phase = "uitbollen"
        robot.hold(2.0)
        x, y = robot.odo.x, robot.odo.y
        print(f"Einde: {x:+.3f} m vooruit, {y * 100:+.1f} cm links t.o.v. de start, koers {robot.heading:+.1f}° (schatting)")


if __name__ == "__main__":
    main()
