"""Rechtdoor rijden met optionele CSV; analyse achteraf met analyse_rit.py."""
import argparse
from fase1_heen_en_terug import robot_session, prepare, positive_seconds, add_drive_args, run_segment


def main():
    ap = argparse.ArgumentParser(description="Rechtdoor rijden met optionele IMU-logging")
    ap.add_argument("--power", type=int, default=250, help="maximaal motorcommando -1000..1000; negatief = achteruit (of gebruik --distance -3)")
    add_drive_args(ap, distance=2.0)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True, help="CSV opslaan; uitschakelen met --no-csv")
    args = ap.parse_args()
    if not -1000 <= args.power <= 1000:
        ap.error("--power moet tussen -1000 en 1000 liggen")
    with robot_session(args.csv, ip=args.ip, port=args.port, live=args.live) as robot:
        prepare(robot, args.rest)
        robot.phase = "rijden"
        run_segment(robot, args, args.power)
        robot.phase = "uitbollen"
        robot.hold(2.0)


if __name__ == "__main__":
    main()
