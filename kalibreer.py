"""Wielstraal kalibreren met een rolmeter, of de afstandsmethodes vergelijken met de rolmeter.

1. Rij een rechte afstand op de mat:  python fase1_rechtdoor_imu.py --power 250 --distance 3
2. Meet met de rolmeter hoe ver hetzelfde punt van de robot (bv. de voorbumper) echt verplaatst is.
3. python kalibreer.py 2.968            (gemeten meter; standaard de nieuwste rit)
   python kalibreer.py 2.968 ritten/rit_....csv --fase heen
   python kalibreer.py 6.41 --vergelijk  (op gras/grind: alleen vergelijken, niets opslaan)

De encoders tellen wielomwentelingen; de effectieve rolstraal van de noppenbanden op de
ondergrond wijkt af van de nominale 0,135 m. Nieuwe straal = gebruikte straal × echt / encoder,
met het gemiddelde van de 4 wielen. Kalibreer op een harde, vlakke ondergrond: op gras zou de
straal de slip mee opnemen. Het resultaat komt in kalibratie.json (andere waarden, zoals de
gyroschaal, blijven staan) en geldt voor de rijscripts en de analyse.
"""
import argparse
import csv
import json
from pathlib import Path

from analyse_rit import CALIBRATION, KALIBRATIE, WHEEL_R, analyse

HERE = Path(__file__).parent


def main():
    ap = argparse.ArgumentParser(description="Wielstraal kalibreren of afstandsmethodes vergelijken met een gemeten afstand")
    ap.add_argument("gemeten", type=float, help="echte verplaatsing in meter (rolmeter)")
    ap.add_argument("rit", nargs="?", help="rit-CSV (standaard de nieuwste)")
    ap.add_argument("--fase", help="segment, bv. heen (standaard het eerste met een doel)")
    ap.add_argument("--vergelijk", action="store_true", help="alleen de methodes vergelijken, geen nieuwe wielstraal opslaan")
    args = ap.parse_args()
    path = Path(args.rit) if args.rit else max((HERE / "ritten").glob("rit_*.csv"), key=lambda p: p.stat().st_mtime)
    radius = {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            w = r["regel"].split()
            if w and w[0] == "DOEL":
                radius[w[1]] = float(w[3][2:]) if len(w) > 3 else 0.135
    segments = {d["fase"]: d for d in analyse(path)["summary"]["doelen"] if d.get("wielen4_m") is not None}
    fase = args.fase or next(iter(segments), None)
    if fase not in segments:
        raise SystemExit(f"Geen gemeten stop voor fase '{fase}' in {path.name}. Rij met --distance.")
    seg = segments[fase]
    echt = args.gemeten * (1 if seg["wielen4_m"] >= 0 else -1)

    print(f"{path.name} · {fase} · rolmeter {echt:+.4f} m (wielstraal nu {WHEEL_R:.5f} m)")
    for naam, key in (("rijscript (stopwaarde)", "gemeten_m"), ("gemiddelde 4 wielen", "wielen4_m"),
                      ("slipbewust (odometrie.py)", "slipbewust_m"), ("voorwielen", "voorwielen_m"), ("achterwielen", "achterwielen_m")):
        v = seg.get(key)
        if v is not None:
            print(f"  {naam:27} {v:+.4f} m   fout {(v - echt) * 1000:+7.1f} mm  ({(v / echt - 1) * 100:+.2f} %)")
    if args.vergelijk:
        return

    # De straal waarmee de analyse rekende (kalibratie.json), niet die tijdens het rijden (DOEL-regel).
    new_r = WHEEL_R * abs(echt / seg["wielen4_m"])
    data = dict(KALIBRATIE, wielstraal_m=round(new_r, 5), rit=path.stem, fase=fase,
                encoder_m=abs(seg["wielen4_m"]), gemeten_m=args.gemeten)
    CALIBRATION.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"Wielstraal {WHEEL_R:.5f} -> {new_r:.5f} m ({(new_r / WHEEL_R - 1) * 100:+.2f} %), opgeslagen in {CALIBRATION.name}"
          + (f" (rijscript reed met {radius[fase]:.5f} m)" if fase in radius else ""))


if __name__ == "__main__":
    main()
