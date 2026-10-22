"""Controles voor jaguar/navigatie zonder robot:  python -m jaguar.testen.test_navigatie"""
import csv
import glob
import math
import random
from pathlib import Path

from jaguar.navigatie.gps import (GpsAnker, Lokaal, NoordPeiling, afstand_richting, checksum_ok, hoek180, parse_rmc,
                                  robot_naar_enu)
from jaguar.navigatie.route import route_naar_stappen

ROOT = Path(__file__).resolve().parents[2]


def test_nmea():
    zin = "$GPRMC,110644.4,A,5050.50731,N,00419.38150,E,000.39,336.7,221026,000.4,E*53"
    assert checksum_ok(zin)
    assert not checksum_ok(zin.replace("336.7", "336.8"))
    d = parse_rmc(zin)
    assert d["fix"] and abs(d["lat"] - (50 + 50.50731 / 60)) < 1e-9 and abs(d["lon"] - (4 + 19.38150 / 60)) < 1e-9
    assert abs(d["v"] - 0.39 * 0.514444) < 1e-9 and d["cog"] == 336.7
    assert parse_rmc("$GPRMC,1,V,,,,,,,,,*00") is None or not parse_rmc("$GPRMC,1,V,,,,,,,,,*00")["fix"]


def test_omrekenen():
    lok = Lokaal(50.84, 4.32)
    e, n = lok.naar_xy(50.84 + 0.0001, 4.32)
    assert abs(n - 11.1) < 0.1 and abs(e) < 1e-9, (e, n)                        # 0,0001° breedte ≈ 11,1 m
    e, n = lok.naar_xy(50.84, 4.32 + 0.0001)
    assert abs(e - 11.1 * math.cos(math.radians(50.84))) < 0.1, e               # lengte krimpt met cos(breedte)
    for oost, noord in ((0, 0), (3.5, -2.25), (-120.0, 80.0)):
        lat, lon = lok.naar_ll(oost, noord)
        e2, n2 = lok.naar_xy(lat, lon)
        assert abs(e2 - oost) < 1e-6 and abs(n2 - noord) < 1e-6
    af, b = afstand_richting((0, 0), (3, 4))
    assert abs(af - 5) < 1e-9 and abs(b - math.degrees(math.atan2(3, 4))) < 1e-9
    assert abs(hoek180(190) + 170) < 1e-9 and abs(hoek180(-180) - 180) < 1e-9
    e, n = robot_naar_enu(1.0, 0.0, 90.0)                                       # vooruit terwijl het robot oost kijkt
    assert abs(e - 1) < 1e-9 and abs(n) < 1e-9
    e, n = robot_naar_enu(0.0, 1.0, 90.0)                                       # links van oost kijken = noord
    assert abs(e) < 1e-9 and abs(n - 1) < 1e-9


def test_route_vierkant():
    lok = Lokaal(50.84, 4.32)
    hoeken = [lok.naar_ll(*p) for p in ((0, 0), (0, 10), (10, 10), (10, 0), (0, 0))]       # noord, oost, zuid, west
    stappen, lengtes = route_naar_stappen(hoeken, b0=0.0)                                   # robot start naar het noorden
    assert [s for s, _ in stappen] == ["vooruit", "rechts", "vooruit", "rechts", "vooruit", "rechts", "vooruit"], stappen
    assert all(abs(w - 90) < 1e-6 for s, w in stappen if s == "rechts")
    assert all(abs(l - 10) < 1e-6 for l in lengtes)
    # Start naar het oosten kijkend: eerst 90° naar links om naar het noorden te kijken.
    stappen, _ = route_naar_stappen(hoeken, b0=90.0)
    assert stappen[0][0] == "links" and abs(stappen[0][1] - 90) < 1e-6, stappen[:2]


def test_noord():
    random.seed(1)
    b0 = 123.0
    p = NoordPeiling()
    for k in range(40):
        koers = 5.0                                                                  # robot rijdt op koers +5° (links)
        gps = {"fix": True, "cog": (b0 - koers + random.gauss(0, 1.5)) % 360, "v": 1.5, "lat": 0, "lon": 0}
        p.voeg_toe(gps, koers, 0.5)
    b, s = p.resultaat()
    assert abs(hoek180(b - b0)) < 1.0 and s < 3, (b, s)
    p = NoordPeiling()                                                              # te langzaam: geen resultaat
    for k in range(40):
        p.voeg_toe({"fix": True, "cog": random.uniform(0, 360), "v": 0.4, "lat": 0, "lon": 0}, 0.0, 0.0)
    assert p.resultaat() is None


def test_anker_schoon_en_met_uitschieters():
    random.seed(2)
    lok = Lokaal(50.84, 4.32)
    b0 = 40.0
    anker = GpsAnker(lok, b0)
    echte_verschuiving = (7.0, -4.0)              # waar de odometrie t.o.v. de wereld staat
    gebruikt = 0
    for k in range(200):
        x = 0.05 * k                              # robot rijdt 1,5 m/s over 10 m
        oe, on = robot_naar_enu(x, 0.0, b0)
        ruis = 1.5 if k % 40 else 25.0            # 1 op 40 metingen springt 25 m
        lat, lon = lok.naar_ll(oe + echte_verschuiving[0] + random.gauss(0, ruis), on + echte_verschuiving[1] + random.gauss(0, ruis))
        gebruikt += anker.meting({"fix": True, "lat": lat, "lon": lon}, x, 0.0)
    pos = anker.positie(10.0, 0.0)
    oe, on = robot_naar_enu(10.0, 0.0, b0)
    fout = math.hypot(pos[0] - oe - echte_verschuiving[0], pos[1] - on - echte_verschuiving[1])
    assert fout < 2.0, fout
    assert anker.geweigerd >= 1                  # de sprongen van 25 m zijn eruit gefilterd


def test_anker_binnen_ritten():
    """Echte GPS-gegevens van vandaag (binnen: positie dwaalt tientallen meters): het anker moet weinig
    vertrouwen geven. Slaat over als de ritten er niet zijn."""
    gevonden = 0
    for f in sorted(glob.glob(str(ROOT / "ritten" / "rit_20261022_12*_parcours.csv"))):
        rijen = list(csv.reader(open(f, encoding="utf-8")))[1:]
        zinnen = [parse_rmc(r[3]) for r in rijen if r[3].startswith("$GPRMC")]
        zinnen = [z for z in zinnen if z]
        if len(zinnen) < 100:
            continue
        lok = Lokaal(zinnen[0]["lat"], zinnen[0]["lon"])
        anker = GpsAnker(lok, 0.0)
        for z in zinnen:
            anker.meting(z, 0.0, 0.0)                # robot "stilstaand" t.o.v. de odometrie: alles wat de GPS beweegt is fout
        gevonden += 1
        assert not anker.betrouwbaar, (f, anker.geaccepteerd, anker.geweigerd)
    print(f"  ({gevonden} ritten met binnen-GPS gecontroleerd)")


def main():
    for naam, f in sorted(globals().items()):
        if naam.startswith("test_") and callable(f):
            f()
            print("ok ", naam)


if __name__ == "__main__":
    main()
