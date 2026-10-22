"""Fase 1: heen en terug rijden.  python -m jaguar.rijden.heen_en_terug --power 250 --distance 1.5"""
import argparse

from jaguar.kern.robot import add_drive_args, positive_seconds, prepare, robot_session, run_segment


def main():
    ap = argparse.ArgumentParser(description="Fase 1: heen en terug rijden")
    ap.add_argument("--power", type=int, default=250, help="maximaal motorcommando, geen snelheid in m/s; negatief = eerst achteruit, dan terug vooruit")
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.5)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True, help="CSV opslaan (standaard aan); --no-csv schakelt uit")
    args = ap.parse_args()
    if not -1000 <= args.power <= 1000:
        ap.error("--power moet tussen -1000 en 1000 liggen")
    with robot_session(args.csv, "_heen_terug", args.ip, args.port, args.live, args.mqtt,
                       args.mqtt_host, args.network) as robot:
        prepare(robot, args.rest)
        robot.phase = "heen"
        run_segment(robot, args, args.power)
        robot.phase = "pauze"
        robot.hold(args.pause)
        robot.phase = "terug"
        run_segment(robot, args, -args.power)
        robot.phase = "uitbollen"
        robot.hold(2.0)


if __name__ == "__main__":
    main()
