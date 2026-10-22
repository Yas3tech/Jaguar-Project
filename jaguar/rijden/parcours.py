"""Parcours rijden: een reeks stappen vooruit, achteruit en ter plaatse draaien.

  python -m jaguar.rijden.parcours vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5
  python -m jaguar.rijden.parcours v:2 l:90 v:2 l:90 v:2 l:90 v:2          (vierkant)

Stappen: vooruit:<m>, achteruit:<m>, links:<graden>, rechts:<graden> (afgekort v, a, l, r).
Rijden gebruikt de slipbewuste odometrie (zoals robot.py), draaien de gyro.
Na een draai volgt de robot de nieuwe koers; opeenvolgende rechte stukken blijven op dezelfde lijn.
Daarna: python -m jaguar.analyse.rit, en het dashboard toont de geplande en de gereden route.
Draaien is zo juist als de gyroschaal: kalibreer eerst met  python -m jaguar.kalibratie.test_sondewiel draai.
"""
import argparse
import math

from jaguar.kern.robot import add_drive_args, positive_seconds, prepare, robot_session

EINDKOERS_TOL = 3.0   # graden: grotere koersfout na de laatste stap wordt ter plaatse rechtgezet
MAX_CORRECTIE = 0.5   # m: grotere correctie voor de vorige stappen vertrouwen we niet (bv. haperende odometrie)
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
    with robot_session(args.csv, "_parcours", args.ip, args.port, args.live, args.mqtt,
                       args.mqtt_host, args.network) as robot:
        prepare(robot, args.rest)
        rijd(robot, args.stappen, args.power, args.turn_power, args.pause, args.ramp, args.creep, args.tolerance)


def rijd(robot, stappen, power=250, turn_power=380, pause=1.0, ramp=1.5, creep=60, tolerance=3.0):
    """De stappen na elkaar rijden (na prepare). Ook gebruikt door bediening.py (knop op de live pagina).

    Rechte stukken rijden naar het geplande hoekpunt (gerekend vanaf de start) en volgen de geplande
    lijn, niet telkens de nominale afstand vanaf waar de robot toevallig staat. Zo werkt elke stap de
    afwijking van de vorige (te ver, te kort, naast de lijn) vanzelf weg in plaats van ze op te stapelen
    (rit 122109: 25 cm aan het einde na 15 stappen). Draaien gaat al naar de geplande absolute koers.
    Alles volgens de odometrie: wat die niet ziet (slip zonder signaal, gyroschaal) blijft staan."""
    px, py = robot.odo.x, robot.odo.y                 # gepland hoekpunt
    for i, (soort, value) in enumerate(stappen, 1):
        if i > 1:
            robot.phase = "pauze"
            robot.hold(pause)
        if soort in ("vooruit", "achteruit"):
            robot.phase = f"{i}-{soort}"
            planned = value if soort == "vooruit" else -value
            robot.note(f"PLAN {robot.phase} {planned:.4f}")
            px, py = rechtdoor(robot, (px, py), planned, power, turn_power, ramp, creep, tolerance)
        else:
            robot.phase = f"{i}-draai"
            robot.turn(value if soort == "links" else -value, turn_power)
    # Eindkoers: bv. een kant die bij het kruipen vastzat en de robot liet zwenken (rit 122109: 20°).
    if robot.odo.klaar and abs(robot.target_heading - robot.heading) > EINDKOERS_TOL:
        robot.phase = "eindkoers"
        robot.turn(0, turn_power, label="KOERS")
    robot.phase = "uitbollen"
    robot.hold(2.0)
    x, y = robot.odo.x, robot.odo.y
    print(f"Einde: {x:+.3f} m vooruit, {y * 100:+.1f} cm links t.o.v. de start, koers {robot.heading:+.1f}° (schatting)")


def rechtdoor(robot, start, planned, power, turn_power, ramp, creep, tolerance):
    """Eén rechte stap van het geplande hoekpunt `start` over `planned` m (− = achteruit) op de
    gewenste koers. Geeft het geplande hoekpunt terug."""
    heading = robot.target_heading
    h = math.radians(heading)
    ux, uy = math.cos(h), math.sin(h)

    def along():                                       # afgelegd op de geplande lijn vanaf start
        return (robot.odo.x - start[0]) * ux + (robot.odo.y - start[1]) * uy

    togo = planned - along()
    if abs(togo - planned) > MAX_CORRECTIE:
        print(f"{robot.phase}: correctie {(togo - planned) * 100:+.0f} cm is te groot (odometrie onbetrouwbaar?): "
              f"de geplande {planned:+.3f} m rijden zonder correctie")
        start = (start[0] + (planned - togo) * ux, start[1] + (planned - togo) * uy)   # lijn verschuift mee
    elif abs(togo - planned) >= 0.005:
        print(f"{robot.phase}: gepland {planned:+.3f} m, te rijden {togo:+.3f} m "
              f"(correctie {(togo - planned) * 100:+.1f} cm voor de vorige stappen)")
    end = (start[0] + planned * ux, start[1] + planned * uy)
    robot.target_heading, robot.line = heading, start   # de geplande lijn
    togo = planned - along()
    robot.drive_distance(power if togo >= 0 else -power, abs(togo), ramp, creep, tolerance=tolerance / 1000)
    return end


if __name__ == "__main__":
    main()
