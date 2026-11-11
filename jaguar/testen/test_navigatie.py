"""Controles voor jaguar/navigatie zonder robot:  python -m jaguar.testen.test_navigatie"""
import csv
import glob
import math
import random
from pathlib import Path

from jaguar.navigatie.gps import (GpsAnker, GpsKwaliteit, Lokaal, NoordPeiling, afstand_richting, checksum_ok, hoek180,
                                  parse_rmc, robot_naar_enu)
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


def test_kwaliteit_stilstaand():
    """Open hemel (0,6 m ruis) = goed; een positie die 20 m per minuut wegloopt = slecht; geen fix = slecht."""
    random.seed(3)
    lok = Lokaal(50.84, 4.32)
    for ruis, wegloop, verwacht in ((0.6, 0.0, "goed"), (2.5, 3.0, "matig"), (2.0, 20.0, "slecht")):
        q = GpsKwaliteit()
        for k in range(5 * 70):                   # 70 s bij 5 Hz
            t = k / 5
            lat, lon = lok.naar_ll(random.gauss(0, ruis) + wegloop * t / 60, random.gauss(0, ruis))
            q.voeg_toe(t, {"fix": True, "lat": lat, "lon": lon, "v": 0.0, "cog": None})
            if t < GpsKwaliteit.MIN_S - 1:
                assert q.rapport(t)["oordeel"] == "wachten"
        r = q.rapport(t)
        assert r["oordeel"] == verwacht, (ruis, wegloop, r)
    q = GpsKwaliteit()
    for k in range(5 * 40):
        q.voeg_toe(k / 5, None)                   # GPRMC zonder positie
    assert q.rapport(40.0)["oordeel"] == "slecht"
    assert q.rapport(45.0)["oordeel"] == "geen"   # al 5 s niets meer ontvangen
    assert GpsKwaliteit().rapport(0.0)["oordeel"] == "geen"


def test_kwaliteit_binnen_ritten():
    """De binnenritten van 22/10 mogen (bijna) nooit "goed" heten. Slaat over als de ritten er niet zijn."""
    goed = totaal = 0
    for f in sorted(glob.glob(str(ROOT / "ritten" / "rit_20261022_12*_parcours.csv"))):
        q = GpsKwaliteit()
        for r in list(csv.reader(open(f, encoding="utf-8")))[1:]:
            if r[3].startswith("$GPRMC") and checksum_ok(r[3]):
                q.voeg_toe(float(r[0]), parse_rmc(r[3]))
                o = q.rapport(float(r[0]))["oordeel"]
                if o != "wachten":
                    totaal += 1
                    goed += o == "goed"
    assert goed <= 0.02 * totaal, (goed, totaal)
    print(f"  ({totaal} oordelen op binnenritten, {goed} keer goed)")


def test_uitlijning():
    """Het gereden spoor op het GPS-spoor leggen: noorden, schaal en positie komen eruit, ook met ruis; een dwalende
    GPS bij een stilstaand robot of een te kort spoor geeft geen bruikbare passing."""
    from jaguar.navigatie.gps import Uitlijning, enu_naar_robot, robot_naar_enu
    random.seed(7)
    lok = Lokaal(50.84, 4.32)
    spoor = [(i * 0.1, 0.0) for i in range(100)] + [(10.0, j * 0.1) for j in range(60)]      # 10 m vooruit, 6 m naar links
    for b0, schaal, ruis in ((0.0, 1.0, 0.02), (73.0, 0.95, 0.3), (200.0, 1.04, 0.3), (331.0, 1.0, 1.5)):
        u = Uitlijning(lok)
        for x, y in spoor:
            e, n = robot_naar_enu(x * schaal, y * schaal, b0)
            lat, lon = lok.naar_ll(e + 3 + random.gauss(0, ruis), n - 2 + random.gauss(0, ruis))
            u.meting({"fix": True, "lat": lat, "lon": lon}, x, y)
        r = u.resultaat()
        tol_hoek, tol_schaal = (0.5, 0.005) if ruis < 0.1 else ((3.0, 0.04) if ruis < 1 else (12.0, 0.2))
        assert abs(hoek180(r["b0"] - b0)) < tol_hoek and abs(r["schaal"] - schaal) < tol_schaal, (b0, schaal, ruis, r)
        assert abs(r["rms"] - ruis * math.sqrt(2)) < 0.35 * ruis * math.sqrt(2) + 0.01, (ruis, r["rms"])
        assert Uitlijning.schaal_bruikbaar(r) == (ruis < 0.1), (ruis, r)
        if ruis < 1:
            e, n = robot_naar_enu(10 * schaal, 6 * schaal, b0)
            pe, pn = u.positie(10.0, 6.0)
            assert math.hypot(pe - e - 3, pn - n + 2) < 4 * ruis + 0.02, (pe, pn)
            ox, oy = u.naar_odo(e + 3, n - 2)
            assert math.hypot(ox - 10, oy - 6) < 4 * ruis + 0.02, (ox, oy)
    u = Uitlijning(lok)                                  # stilstaand robot, dwalende GPS: geen spoor om op te passen
    for k in range(200):
        lat, lon = lok.naar_ll(k * 0.05, random.gauss(0, 1))
        u.meting({"fix": True, "lat": lat, "lon": lon}, 0.0, 0.0)
    assert u.resultaat() is None
    assert Uitlijning(lok).resultaat() is None and Uitlijning(lok).positie(1, 1) is None
    assert not u.meting({"fix": False, "lat": 0, "lon": 0}, 0, 0) and not u.meting(None, 0, 0)
    x, y = enu_naar_robot(*robot_naar_enu(1.5, -2.0, 123.0), 123.0)
    assert abs(x - 1.5) < 1e-9 and abs(y + 2.0) < 1e-9


def test_imu_velden():
    """De IMU-regel van het robot (17 velden), een regel met extra velden (COMP) en beschadigde regels."""
    from jaguar.kern.odometrie import imu_velden
    echt = "#228,YAW,0.03,GYRO,4,-21,11,ACC,-18,0,247,ADC,1270,386,6,2,"
    assert imu_velden(echt.split(",")) == (228, [4, -21, 11], [-18, 0, 247])
    comp = "#7,YAW,1.5,GYRO,1,2,3,ACC,4,5,256,COMP,10,20,30,ADC,0,0,0,0,"
    assert imu_velden(comp.split(",")) == (7, [1, 2, 3], [4, 5, 256])
    assert imu_velden("#7,GYRO,1,2,3,ACC,4,5,256,COMP,10,20,30".split(",")) == (7, [1, 2, 3], [4, 5, 256])
    assert imu_velden("#9,Yaw,0.3,Gyro,1,2,3,Accel,4,5,256,Comp,10,20,30,x".split(",")) == (9, [1, 2, 3], [4, 5, 256])
    for kapot in ("#228,YAW,0.03,GYRO,4,-21,11,ACC,-6,,256,ADC,1,2,3,4,", "#12,YAW,0.1,GYRO,4,-21", "MM0 C=1:2",
                  "$GPRMC,080457.4,V,5050.51796,N,00419.38C,1274,502,7,2,", "#x,YAW,0,GYRO,1,2,3,ACC,1,2,3,ADC,0,0,0,0,"):
        assert imu_velden(kapot.split(",")) is None, kapot


def test_sim_gps_zin():
    """De $GPRMC van de gesimuleerde robot heeft een geldige controlesom en geeft de positie terug (ook zuid/west)."""
    from jaguar.testen.sim_robot import rmc
    for lat, lon in ((50.8419, 4.3229), (-33.8567, 151.2152), (40.7128, -74.0060)):
        g = parse_rmc(rmc(lat, lon, 0.77, 123.4))
        assert g and g["fix"] and abs(g["lat"] - lat) < 2e-7 and abs(g["lon"] - lon) < 2e-7, g
        assert abs(g["v"] - 0.77) < 0.01 and abs(g["cog"] - 123.4) < 0.06, g
    assert parse_rmc(rmc(50.0, 4.0, 0, 0, fix=False))["fix"] is False


def main():
    for naam, f in sorted(globals().items()):
        if naam.startswith("test_") and callable(f):
            f()
            print("ok ", naam)


if __name__ == "__main__":
    main()
