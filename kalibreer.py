"""Wielstraal kalibreren met een rolmeter.

1. Rij een rechte afstand op de mat:  python fase1_rechtdoor_imu.py --power 250 --distance 3
2. Meet met de rolmeter hoe ver hetzelfde punt van de robot (bv. de voorbumper) echt verplaatst is.
3. python kalibreer.py 2.968            (gemeten meter; standaard de nieuwste rit)
   python kalibreer.py 2.968 ritten/rit_....csv --fase heen

De encoders tellen wielomwentelingen; de effectieve rolstraal van de noppenbanden op de
ondergrond wijkt af van de nominale 0,135 m. Nieuwe straal = gebruikte straal × echt / encoder.
Het resultaat komt in kalibratie.json en geldt voor de rijscripts en de analyse.
"""
import argparse
import csv
import json
from pathlib import Path

from analyse_rit import CALIBRATION

HERE = Path(__file__).parent


def main():
    ap = argparse.ArgumentParser(description="Wielstraal kalibreren uit een gemeten afstand")
    ap.add_argument("gemeten", type=float, help="echte verplaatsing in meter (rolmeter)")
    ap.add_argument("rit", nargs="?", help="rit-CSV (standaard de nieuwste)")
    ap.add_argument("--fase", help="segment, bv. heen (standaard het eerste met een doel)")
    args = ap.parse_args()
    path = Path(args.rit) if args.rit else max((HERE / "ritten").glob("rit_*.csv"), key=lambda p: p.stat().st_mtime)
    radius, stops = {}, {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            w = r["regel"].split()
            if w and w[0] == "DOEL":
                radius[w[1]] = float(w[3][2:]) if len(w) > 3 else 0.135
            elif w and w[0] == "STOP":
                stops[w[1]] = abs(float(w[2]))
    fase = args.fase or next(iter(stops), None)
    if fase not in stops:
        raise SystemExit(f"Geen gemeten stop voor fase '{fase}' in {path.name}. Rij met --distance.")
    new_r = radius[fase] * args.gemeten / stops[fase]
    CALIBRATION.write_text(json.dumps({"wielstraal_m": round(new_r, 5), "rit": path.stem, "fase": fase,
                                       "encoder_m": stops[fase], "gemeten_m": args.gemeten}, indent=2), encoding="utf-8")
    print(f"{path.name} · {fase}: encoders {stops[fase]:.4f} m, rolmeter {args.gemeten:.4f} m")
    print(f"Wielstraal {radius[fase]:.5f} -> {new_r:.5f} m ({(new_r / radius[fase] - 1) * 100:+.2f} %), opgeslagen in {CALIBRATION.name}")


if __name__ == "__main__":
    main()
