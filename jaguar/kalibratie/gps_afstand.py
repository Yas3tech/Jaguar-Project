"""Controleren (en bijstellen) of een gevraagde meter ook een echte meter is, met de GPS als meetlat.

  python -m jaguar.kalibratie.gps_afstand --distance 20                rijdt 20 m rechtdoor en vergelijkt
  python -m jaguar.kalibratie.gps_afstand --distance 20 --bewaar       past de wielstraal aan als de meting goed genoeg is
  python -m jaguar.kalibratie.gps_afstand --distance 10 --ip 127.0.0.1 tegen een simulator

Het robot telt wielomwentelingen. Rijdt het minder ver dan de wielen tellen (zachte grond, slip, een andere
bandenspanning), dan is "5 m" geen 5 m en ziet de odometrie dat niet. De GPS ziet het wel: dit script legt het
gereden spoor op het GPS-spoor (Uitlijning in jaguar/navigatie/gps.py) en leest de schaal af.

Hoe goed dat is hangt af van de GPS: de fout op de schaal is ongeveer GPS-fout / afstand. Met een goede GPS (de
websimulator, RTK: centimeters) volstaat 10 m. Met de Garmin van het robot (meters fout, dwaalt) is 20 m rijden
niet genoeg voor procenten: dan zegt het script dat de meting te ruw is en bewaart het niets. Gebruik daar de
rolmeter (python -m jaguar.kalibratie.kalibreer). Kalibreer op de ondergrond waarop je gaat rijden.
"""
import argparse
import json

from jaguar.analyse.rit import CALIBRATION, KALIBRATIE, WHEEL_R
from jaguar.kern.robot import add_drive_args, positive_seconds, prepare, robot_session
from jaguar.navigatie.gps import Lokaal, Uitlijning


def meet(robot, afstand, power=250, ramp=1.5, creep=60, tolerance=10.0, rest=3.0):
    """`afstand` m rechtdoor rijden en de schaal bepalen. Geeft (resultaat van Uitlijning of None, gereden m volgens de odometrie)."""
    staat = {"uit": None}

    def hook(gps, koers, draai, x, y):
        if not gps["fix"]:
            return
        if staat["uit"] is None:
            staat["uit"] = Uitlijning(Lokaal(gps["lat"], gps["lon"]))
        staat["uit"].meting(gps, x, y)
    robot.gps_hook = hook
    prepare(robot, rest)
    robot.phase = "rijden"
    gereden = robot.drive_distance(power, afstand, ramp, creep, tolerance=tolerance / 1000)
    robot.eindkoers()
    robot.phase = "uitbollen"
    robot.hold(2.0)
    robot.gps_hook = None
    return (staat["uit"].resultaat() if staat["uit"] else None), gereden


def main():
    ap = argparse.ArgumentParser(description="Afstand controleren met de GPS: is een gevraagde meter een echte meter?")
    ap.add_argument("--power", type=int, default=250)
    add_drive_args(ap, distance=10.0)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--bewaar", action="store_true", help="de wielstraal aanpassen in config/kalibratie.json (alleen bij een goede meting)")
    args = ap.parse_args()
    if args.duration or args.distance < 2:
        ap.error("gebruik --distance (minstens 2 m, vooruit)")
    with robot_session(args.csv, "_gps_afstand", args.ip, args.port, args.live, args.mqtt, args.mqtt_host, args.network) as robot:
        res, gereden = meet(robot, args.distance, args.power, args.ramp, args.creep, args.tolerance, args.rest)
    if res is None:
        raise SystemExit("Geen GPS-posities ontvangen (geen GPS of geen fix): niets te vergelijken.")
    echt = gereden * res["schaal"]
    print(f"\nGevraagd {args.distance:.2f} m · volgens de wielen {gereden:.3f} m · volgens de GPS {echt:.3f} m "
          f"(schaal {res['schaal']:.4f}, het GPS-spoor past binnen {res['rms']:.2f} m, {res['n']} posities)")
    if not Uitlijning.schaal_bruikbaar(res):
        print("Te ruw om de afstand op te controleren: de GPS-fout is te groot tegenover de gereden afstand. "
              "Rijd verder, of meet met de rolmeter (python -m jaguar.kalibratie.kalibreer).")
        return
    if abs(res["schaal"] - 1) < 0.01:
        print("De afstand klopt binnen 1 %: niets aan te passen.")
        return
    nieuw = WHEEL_R * res["schaal"]
    print(f"Het robot rijdt {abs(res['schaal'] - 1) * 100:.1f} % {'minder' if res['schaal'] < 1 else 'meer'} ver dan de wielen tellen. "
          f"Wielstraal {WHEEL_R:.5f} → {nieuw:.5f} m zou dat goedmaken.")
    if args.bewaar:
        data = dict(KALIBRATIE, wielstraal_m=round(nieuw, 5), bron="gps_afstand", encoder_m=round(gereden, 4), gemeten_m=round(echt, 4))
        CALIBRATION.write_text(json.dumps(data, indent=2), encoding="utf-8")
        print(f"Opgeslagen in {CALIBRATION.name}; geldt vanaf de volgende rit. (De rolmeter blijft de betere meetlat.)")
    else:
        print("Niet opgeslagen: voeg --bewaar toe om de wielstraal aan te passen.")


if __name__ == "__main__":
    main()
