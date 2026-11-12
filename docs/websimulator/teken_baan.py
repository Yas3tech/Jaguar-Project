"""De baan uit de simulator (middellijn) tegen het echt gereden spoor (api_brug --spoor)."""
import csv, json, math, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D, spoor, uit = sys.argv[1], sys.argv[2], sys.argv[3]
baan = [p for p in json.load(open(D + "/baan.json")) if p[0] <= 43.0]
bx, bz = [p[1] for p in baan], [p[2] for p in baan]
rows = list(csv.DictReader(open(spoor, encoding="utf-8")))
x = [float(r["x"]) for r in rows]; z = [float(r["z"]) for r in rows]

def tot_lijn(px, pz):                      # afstand van een punt tot de middellijn (stukken tussen de monsters)
    best = 9e9
    for (ax, az), (cx, cz) in zip(zip(bx, bz), zip(bx[1:], bz[1:])):
        dx, dz = cx - ax, cz - az
        t = max(0.0, min(1.0, ((px - ax) * dx + (pz - az) * dz) / (dx * dx + dz * dz or 1)))
        best = min(best, math.hypot(px - ax - t * dx, pz - az - t * dz))
    return best

afw = [tot_lijn(a, b) for a, b in zip(x, z)]
gereden = sum(math.hypot(a2 - a1, b2 - b1) for a1, b1, a2, b2 in zip(x, z, x[1:], z[1:]))
eind = math.hypot(x[-1] - bx[0], z[-1] - bz[0])
gem = sum(afw) / len(afw)
fig, ax = plt.subplots(figsize=(8.5, 8))
ax.plot(bx, [-v for v in bz], color="#c9a36a", lw=14, alpha=.55, solid_capstyle="round", label="aarden pad (ca. 1 m breed getekend)")
ax.plot(bx, [-v for v in bz], "--", color="#555", lw=1.2, label="middellijn = de opgegeven GPS-punten")
ax.plot(x, [-v for v in z], color="#d33", lw=1.6, label="echt gereden (positie in de simulator)")
ax.plot(bx[0], -bz[0], "o", color="green", ms=11, label="start en doel")
ax.plot(x[-1], -z[-1], "x", color="black", ms=12, mew=3, label=f"gestopt op {eind * 100:.0f} cm van het doel")
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9)
ax.set_xlabel("oost (m)"); ax.set_ylabel("noord (m)")
ax.set_title(f"Websimulator, park: de ε-baan autonoom gereden uit GPS-coördinaten\n"
             f"{gereden:.1f} m in {float(rows[-1]['t']) - float(rows[0]['t']):.0f} s · naast de middellijn: gemiddeld {gem * 100:.0f} cm, hoogstens {max(afw) * 100:.0f} cm")
fig.savefig(uit, dpi=110, bbox_inches="tight")
print(f"gereden {gereden:.2f} m, einde {eind * 100:.1f} cm van het doel, naast de middellijn gemiddeld {gem * 100:.1f} cm, hoogstens {max(afw) * 100:.1f} cm")
