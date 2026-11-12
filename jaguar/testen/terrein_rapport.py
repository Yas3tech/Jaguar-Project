"""Een rit over stenen en door plassen, in de gesimuleerde robot: rijden, analyseren en tekenen waar het robot reed.

Alles in één keer (het demo-parcours uit testdata/terrein_demo.json, ongeveer twee minuten):
  python -m jaguar.testen.terrein_rapport --demo

Zelf een route tekenen (drie terminals):
  1. python -m jaguar.testen.sim_robot --terrein testdata/terrein_demo.json --spoor ritten/sim_spoor.csv
  2. python -m jaguar.rijden.bediening --ip 127.0.0.1
     op de pagina: Rijden → Binnen → teken je route vanaf de start (of plak de "route" uit het terreinbestand bij
     Voorbeelden → Route als tekst), kies bogen "Vloeiend", start de rit, en klik daarna op Analyse
  3. python -m jaguar.testen.terrein_rapport          maakt ritten/terrein_rapport.png van de nieuwste rit

De tekening toont: de stenen (grijs) en de plassen (blauw) zoals ze in de simulatie liggen, waar het robot ECHT reed
(rood, uit de simulatie), waar het robot DENKT dat het reed (de odometrie, gekleurd naar de trilling die de IMU
voelde) en wat de analyse onderweg opmerkte (kruisjes). Eronder de cijfers per stuk grond.

Wat je eraan ziet: stenen voelt het robot (trilling, schokken) en het rijdt er gewoon overheen; in een plas slippen
alle wielen samen en dat ziet de odometrie niet of nauwelijks: de wielen tellen meer meters dan het robot aflegt, en
daarna rijdt het robot naast zijn route zonder het te weten. Het robot herkent een plas dus NIET vooraf en ontwijkt
niets: het volgt de lijn die je tekent. Dit is een model (sim_robot.py), geen meting op de echte robot.
"""
import argparse
import csv
import json
import math
import socket
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
KLEUR = {"stenen": "#8a8a8a", "plas": "#4a90d9"}


def demo(terrein_pad, spoor_pad):
    """Het demo-parcours rijden tegen een gesimuleerde robot met dat terrein; geeft het pad van de rit-CSV."""
    from jaguar.kern.robot import prepare, robot_session
    from jaguar.rijden.parcours import rijd
    from jaguar.testen.sim_robot import Sim, serve
    data = json.loads(Path(terrein_pad).read_text(encoding="utf-8"))
    srv = socket.create_server(("127.0.0.1", 0))
    sim = Sim(terrein=data["zones"], spoor=str(spoor_pad), richting=float(data["route"]["b0"]))
    threading.Thread(target=serve, args=(srv, sim, False), daemon=True).start()
    with robot_session(True, "_terrein", "127.0.0.1", srv.getsockname()[1], live=False) as robot:
        prepare(robot, 3.0)
        rijd(robot, [tuple(s) for s in data["stappen"]], data["route"]["power"], pause=0.5)
    sim.running = False
    srv.close()
    return max((ROOT / "ritten").glob("rit_*_terrein.csv"), key=lambda p: p.stat().st_mtime)


def rapport(terrein_pad, spoor_pad, analyse_pad, uit, plan=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from jaguar.navigatie.gps import robot_naar_enu
    from jaguar.rijden.parcours import pad_punten

    data = json.loads(Path(terrein_pad).read_text(encoding="utf-8"))
    b0 = float(data["route"]["b0"])
    zones = data["zones"]
    echt = [(float(r["oost"]), float(r["noord"]), r["grond"]) for r in csv.DictReader(open(spoor_pad, encoding="utf-8"))]
    ana = json.loads(Path(analyse_pad).read_text(encoding="utf-8"))
    se, sm = ana["series"], ana["summary"]
    odo = [robot_naar_enu(x, y / 100, b0) for x, y in zip(se["x"], se["y"])]     # de analyse bewaart y in cm
    # trilling: [tijd, …, g] om de 0,2 s; bij elke waarde de plek volgens de odometrie op dat moment
    import bisect
    tijd = se["t"]
    plek = [odo[min(len(odo) - 1, bisect.bisect_left(tijd, v[0]))] for v in se["vib"]]
    tril = [float(v[-1]) for v in se["vib"]]

    def zone_van(e, n):
        return next((k for k, z in enumerate(zones) if math.hypot(e - z["x"], n - z["y"]) <= z["r"]), None)

    fig, ax = plt.subplots(figsize=(10, 7.5))
    for z in zones:
        ax.add_patch(plt.Circle((z["x"], z["y"]), z["r"], color=KLEUR[z["soort"]], alpha=.35, lw=0))
        ax.text(z["x"], z["y"], z["soort"], ha="center", va="center", fontsize=9, color="#222")
    if plan:
        pad, _ = pad_punten((0.0, 0.0), 0.0, [tuple(s) for s in data["stappen"]])
        pe = [robot_naar_enu(x, y, b0) for x, y in pad]
        ax.plot([p[0] for p in pe], [p[1] for p in pe], "--", color="#444", lw=1.3, label="gevraagde route")
    sc = ax.scatter([p[0] for p in plek], [p[1] for p in plek], c=tril, cmap="viridis", s=14, vmin=0, vmax=0.3,
                    label="waar het robot denkt te rijden (odometrie), kleur = trilling")
    ax.plot([p[0] for p in echt], [p[1] for p in echt], color="#d33", lw=1.8, label="waar het robot echt reed (simulatie)")
    ax.plot(0, 0, "o", color="green", ms=10, label="start")
    ax.plot(echt[-1][0], echt[-1][1], "x", color="black", ms=11, mew=3, label="echt einde")
    for g in sm.get("gebeurtenissen", []):
        ge, gn = robot_naar_enu(g.get("x", 0.0), g.get("y", 0.0), b0)
        ax.plot(ge, gn, "P", color="#e08a00", ms=11, mec="black", zorder=5)
        ax.annotate(g["soort"], (ge, gn), textcoords="offset points", xytext=(8, 8), fontsize=8, color="#7a4a00")
    plt.colorbar(sc, ax=ax, shrink=.6, label="trilling volgens de IMU (g)")
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(loc="upper left", bbox_to_anchor=(0.0, -0.08), fontsize=8, ncol=2)
    ax.set_xlabel("oost (m)"); ax.set_ylabel("noord (m)")

    # --- cijfers
    verschil = math.hypot(echt[-1][0] - odo[-1][0], echt[-1][1] - odo[-1][1])
    regels = [f"Rit {sm['rit']}: {sm['afstand_slipbewust_m']:.1f} m volgens de odometrie.",
              f"Aan het einde staat het robot {verschil * 100:.0f} cm van waar het zelf denkt te staan."]
    if plan:
        doel = pe[-1]
        regels.append(f"Echt einde: {math.hypot(echt[-1][0] - doel[0], echt[-1][1] - doel[1]) * 100:.0f} cm van het einde van de route; "
                      f"volgens het robot zelf: {math.hypot(odo[-1][0] - doel[0], odo[-1][1] - doel[1]) * 100:.0f} cm.")
    per = {}                                           # trilling per soort grond, op de plek waar het robot ECHT was
    # het spoor van de simulatie en de trilling lopen allebei om de 0,2 s; de rit eindigt voor beide op hetzelfde moment
    n = min(len(echt), len(tril))
    for k in range(1, n + 1):
        per.setdefault(echt[-k][2] or "gewone grond", []).append(tril[-k])
    for soort, waarden in sorted(per.items()):
        regels.append(f"  {soort:13} trilling gemiddeld {sum(waarden) / len(waarden):.3f} g ({len(waarden) / 5:.0f} s gereden)")
    gezien = [g["soort"] for g in sm.get("gebeurtenissen", [])]
    regels.append("Opgemerkt door de analyse: " + (", ".join(gezien) if gezien else "niets"))
    ax.set_title("Gesimuleerde rit over stenen en door plassen\n" + regels[1], fontsize=11)
    fig.savefig(uit, dpi=110, bbox_inches="tight")
    print("\n".join(regels))
    print(f"Tekening: {uit}")


def main():
    sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description="Rit over stenen en door plassen: rijden (--demo), analyseren en tekenen")
    ap.add_argument("--demo", action="store_true", help="eerst het demo-parcours rijden tegen de gesimuleerde robot")
    ap.add_argument("--terrein", default=str(ROOT / "testdata" / "terrein_demo.json"))
    ap.add_argument("--spoor", default=str(ROOT / "ritten" / "sim_spoor.csv"), help="waar het robot echt reed (sim_robot --spoor)")
    ap.add_argument("--rit", help="rit-CSV (standaard de nieuwste)")
    ap.add_argument("--uit", default=str(ROOT / "ritten" / "terrein_rapport.png"))
    args = ap.parse_args()
    from jaguar.analyse import rit as analyse
    if args.demo:
        pad = demo(args.terrein, args.spoor)
    else:
        pad = Path(args.rit) if args.rit else max((ROOT / "ritten").glob("rit_*.csv"), key=lambda p: p.stat().st_mtime)
    analyse.run(pad)                                   # schrijft <rit>_analyse.json en het dashboard
    rapport(args.terrein, args.spoor, pad.with_name(pad.stem + "_analyse.json"), args.uit, plan=args.demo or pad.stem.endswith("_terrein"))


if __name__ == "__main__":
    main()
