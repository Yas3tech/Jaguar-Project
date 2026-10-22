"""Een route rijden die uit GPS-coördinaten bestaat.

  python -m jaguar.rijden.gps_route 50.84000,4.32000 50.84010,4.32000 50.84010,4.32015 --richting 0 --droog
  python -m jaguar.rijden.gps_route <lat,lon> <lat,lon> ...  --richting 90
  python -m jaguar.rijden.gps_route <lat,lon> <lat,lon> ...  --kalibreer-noord 8

Het eerste punt is coördinaat 1: daar staat het robot bij de start (nulpunt). De hoeken en afstanden tussen de
punten komen uit de coördinaten en worden gereden met de wielen en de gyro (parcours.py); de GPS van het robot
is te ruw (3 tot 15 m, binnen onbruikbaar) om er de route op te sturen. De GPS is alleen nodig voor het noorden:
  --richting <graden>   waar het robot bij de start naartoe kijkt (kompaskoers, 0 = noord, 90 = oost)
  --kalibreer-noord <m> rijdt eerst <m> vooruit en weer terug en leidt de richting af uit de koers over de grond
                        (alleen buiten, boven ~1 m/s; anders stopt het script voor de route begint)
  --droog               alleen tonen wat het zou rijden, zonder het robot
Na de rit toont het script of de GPS betrouwbaar was (drift van het verschil GPS − odometrie) en waar het robot
volgens GPS en wielen samen staat.
"""
import argparse
import math

from jaguar.kern.robot import add_drive_args, positive_seconds, prepare, robot_session
from jaguar.navigatie.gps import GpsAnker, Lokaal, NoordPeiling, robot_naar_enu
from jaguar.navigatie.route import beschrijf, route_naar_stappen
from jaguar.rijden.parcours import rijd


def coordinaat(tekst):
    try:
        lat, lon = (float(x) for x in tekst.replace(";", ",").split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{tekst}': gebruik breedte,lengte, bv. 50.84000,4.32000") from None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise argparse.ArgumentTypeError(f"'{tekst}' ligt niet op aarde")
    return lat, lon


def main():
    ap = argparse.ArgumentParser(description="Route van GPS-coördinaten rijden")
    ap.add_argument("punten", nargs="+", type=coordinaat, help="breedte,lengte; het eerste is het startpunt")
    ap.add_argument("--richting", type=float, help="kompaskoers van het robot bij de start (0 = noord)")
    ap.add_argument("--kalibreer-noord", type=float, metavar="METER", help="eerst zoveel meter vooruit en terug om het noorden te vinden")
    ap.add_argument("--droog", action="store_true", help="alleen het plan tonen")
    ap.add_argument("--power", type=int, default=250)
    ap.add_argument("--turn-power", type=int, default=380)
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.0)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()
    if len(args.punten) < 2:
        ap.error("een route heeft minstens twee punten")
    if args.richting is None and args.kalibreer_noord is None:
        ap.error("geef --richting of --kalibreer-noord op")
    if args.droog and args.richting is None:
        ap.error("--droog heeft --richting nodig")
    lokaal = Lokaal(*args.punten[0])

    if args.droog:
        stappen, lengtes = route_naar_stappen(args.punten, args.richting, lokaal)
        print(f"Startkoers {args.richting:.1f}° · {len(lengtes)} rechte stukken · {sum(lengtes):.2f} m")
        print(beschrijf(stappen))
        return

    peiling = NoordPeiling()
    with robot_session(args.csv, "_gps_route", args.ip, args.port, args.live, args.mqtt,
                       args.mqtt_host, args.network) as robot:
        b0 = args.richting
        anker = GpsAnker(lokaal, b0) if b0 is not None else None

        def hook(gps, koers, draai, x, y):
            if anker is not None:
                anker.meting(gps, x, y)
            elif robot.odo.v > 0.5:               # alleen heen: tijdens het achteruit terugrijden is COG omgekeerd
                peiling.voeg_toe(gps, koers, draai)
        robot.gps_hook = hook
        prepare(robot, args.rest)
        if b0 is None:
            robot.phase = "noord-heen"
            robot.drive_distance(args.power, args.kalibreer_noord, args.ramp, args.creep, tolerance=args.tolerance / 1000)
            robot.phase = "noord-terug"
            robot.drive_distance(-args.power, args.kalibreer_noord, args.ramp, args.creep, tolerance=args.tolerance / 1000)
            res = peiling.resultaat()
            if res is None:
                raise SystemExit(f"Het noorden is niet te bepalen: {len(peiling.monsters)} bruikbare GPS-koersen "
                                 f"(nodig: 15 boven 1 m/s met een rustige gyro, en weinig spreiding). Buiten rijden, "
                                 f"verder of sneller (--power), of geef --richting op.")
            b0, spreiding = res
            print(f"Noord: het robot keek bij de start naar kompaskoers {b0:.1f}° (spreiding {spreiding:.1f}°, "
                  f"{len(peiling.monsters)} metingen)")
            anker = GpsAnker(lokaal, b0)
        stappen, lengtes = route_naar_stappen(args.punten, b0, lokaal)
        print(f"Route: {len(lengtes)} rechte stukken · {sum(lengtes):.2f} m\n{beschrijf(stappen)}")
        rijd(robot, stappen, args.power, args.turn_power, args.pause, args.ramp, args.creep, args.tolerance)
        robot.gps_hook = None
        pos = anker.positie(robot.odo.x, robot.odo.y)
        oe, on = robot_naar_enu(robot.odo.x, robot.odo.y, b0)
        # waar het robot volgens de wielen staat, in coördinaten (het startpunt + wat de odometrie meldt)
        lat, lon = lokaal.naar_ll(oe, on)
        print(f"Volgens wielen en gyro: {lat:.7f}, {lon:.7f}")
        if anker.betrouwbaar and pos is not None:
            lat, lon = lokaal.naar_ll(*pos)
            print(f"GPS betrouwbaar (drift {anker.drift:.1f} m, {anker.geaccepteerd} van {anker.geaccepteerd + anker.geweigerd} metingen): "
                  f"absolute positie {lat:.7f}, {lon:.7f} (± enkele meters)")
        else:
            print(f"GPS niet betrouwbaar ({anker.geaccepteerd} geaccepteerd, {anker.geweigerd} geweigerd, drift {anker.drift:.1f} m): "
                  f"alleen de wielen en de gyro tellen")


if __name__ == "__main__":
    main()
