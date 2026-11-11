"""Controles voor het rijden naar GPS-coördinaten, tegen de gesimuleerde robot met GPS (sim_robot.py):
  python -m jaguar.testen.test_gps_route            alles (ongeveer zes minuten)
  python -m jaguar.testen.test_gps_route slip       alleen de tests met "slip" in de naam

Gemeten wordt waar het robot in de simulatie echt staat, tegenover de coördinaat waar het heen moest:
  - een route van coördinaten met een opgegeven richting komt aan op het laatste punt;
  - het noorden wordt gevonden door een stukje te rijden (ook als het robot scheef staat), en --vanaf-hier werkt;
  - met slip (het robot rijdt minder ver dan de wielen tellen) ziet de GPS dat: de schaal klopt, en met
    --gps-correctie komt het robot veel dichter bij het doel dan zonder;
  - een onrustige GPS (meters ruis, dwalen) maakt de rit niet slechter dan rijden op de wielen;
  - zonder GPS rijdt een route met --richting gewoon, en vraagt het zoeken van het noorden om --richting.
De simulatie is een model: geen bewijs voor de echte robot of voor de simulator van robotics.dinno.be.
"""
import math
import sys
import time

from jaguar.kalibratie.gps_afstand import meet
from jaguar.navigatie.gps import robot_naar_enu
from jaguar.rijden.gps_route import afdrukken, rijd_route
from jaguar.testen.test_rijden import hoek, simulatie
from jaguar.kern.robot import robot_session


def punten(sim, *oost_noord):
    """(oost, noord) in meters vanaf de start van de simulatie -> coördinaten."""
    return [sim.lokaal.naar_ll(o, n) for o, n in oost_noord]


def echte_fout(sim, doel):
    """Afstand (m) tussen waar het robot echt staat en de coördinaat `doel`."""
    o, n = robot_naar_enu(sim.x, sim.y, sim.richting)
    do, dn = sim.lokaal.naar_xy(*doel)
    return math.hypot(o - do, n - dn)


def route(poort, pts, **kw):
    with robot_session(False, ip="127.0.0.1", port=poort, live=False) as robot:
        r = rijd_route(robot, pts, rest=1.5, pause=0.3, **kw)
    afdrukken(r)
    return r


def test_route_met_richting():
    with simulatie(richting=90.0) as (sim, poort):                    # kijkt naar het oosten
        pts = punten(sim, (0, 0), (4, 0), (4, 3), (0, 3))
        r = route(poort, pts, richting=90.0)
        fout = echte_fout(sim, pts[-1])
        assert fout < 0.15, f"{fout:.2f} m van het laatste punt"
        assert r["gps_fout"] is not None and abs(r["gps_fout"] - fout) < 0.1, r["gps_fout"]
        p = r["passing"]
        assert p and p["rms"] < 0.1 and abs(hoek(p["b0"] - 90)) < 2 and abs(p["schaal"] - 1) < 0.02, p
    return f"11 m langs 3 punten: {fout * 100:.0f} cm van het doel; spoor past binnen {p['rms'] * 100:.0f} cm, noorden {p['b0']:.1f}°, schaal {p['schaal']:.3f}"


def test_noord_zoeken_en_vanaf_hier():
    with simulatie(richting=137.0, gps_ruis=0.05) as (sim, poort):   # staat scheef, weet zelf niet hoe
        doel = punten(sim, (3, -4))
        r = route(poort, doel, kalibreer_noord=4.0, vanaf_hier=True, gps_wacht=3.0)
        fout = echte_fout(sim, doel[0])
        assert abs(hoek(r["b0"] - 137)) < 3.0, f"noorden {r['b0']:.1f}° i.p.v. 137°"
        assert fout < 0.4, f"{fout:.2f} m van het doel"
    return f"noorden gevonden: {r['b0']:.1f}° (echt 137°, {r['noord'].split(' (')[0]}); naar één coördinaat op 5 m: {fout * 100:.0f} cm ernaast"


def test_slip_gps_correctie():
    """6 % slip: 20 m rijden op de wielen eindigt ruim een meter te kort. De GPS ziet het en stuurt bij."""
    uit = {}
    for correctie in ("uit", "auto"):
        with simulatie(slip=0.06, gps_ruis=0.03) as (sim, poort):
            pts = punten(sim, (0, 0), (0, 10), (6, 10), (6, 6))
            r = route(poort, pts, richting=0.0, gps=correctie)
            uit[correctie] = (echte_fout(sim, pts[-1]), r)
    zonder, met = uit["uit"][0], uit["auto"][0]
    p = uit["uit"][1]["passing"]
    assert p["schaal_bruikbaar"] and abs(p["schaal"] - 0.94) < 0.015, f"schaal {p['schaal']:.3f} i.p.v. 0,94"
    assert zonder > 0.25, f"zonder correctie al {zonder:.2f} m: de test meet niets"
    assert met < 0.4 * zonder and met < 0.25, f"met GPS-correctie {met:.2f} m, zonder {zonder:.2f} m"
    assert uit["auto"][1]["gebruik"]["positie"] > 0 and uit["uit"][1]["gebruik"]["positie"] == 0
    return f"6 % slip over 20 m: zonder correctie {zonder * 100:.0f} cm van het doel, met {met * 100:.0f} cm; schaal volgens de GPS {p['schaal']:.3f}"


def test_onrustige_gps():
    """GPS zoals de fabrikant hem opgeeft (3 m in 95 % van de gevallen: 1,5 m ruis, langzaam dwalen, 0,6 s te laat).
    Het robot staat op een bekend punt: de rit mag door die GPS niet slechter worden dan rijden op de wielen."""
    with simulatie(gps_ruis=1.5, gps_dwaal=0.25, gps_vertraging=0.6) as (sim, poort):
        pts = punten(sim, (0, 0), (0, 6), (5, 6))
        r = route(poort, pts, richting=0.0)
        fout = echte_fout(sim, pts[-1])
        assert fout < 0.6, f"{fout:.2f} m van het doel met een onrustige GPS"
        assert r["gebruik"]["positie"] == 0, r["gebruik"]
    p = r["passing"]
    return f"11 m met 1,5 m GPS-ruis: {fout * 100:.0f} cm van het doel (spoor past binnen {p['rms']:.1f} m, GPS-positie niet gebruikt)"


def test_gps_vertraging():
    """Een nauwkeurige GPS die 0,6 s achterloopt: de vertraging wordt gevonden en het robot komt toch op het punt."""
    with simulatie(gps_vertraging=0.6, slip=0.04) as (sim, poort):
        pts = punten(sim, (0, 0), (0, 8), (5, 8))
        r = route(poort, pts, richting=0.0)
        fout = echte_fout(sim, pts[-1])
        assert abs(r["vertraging"] - 0.6) < 0.25, f"vertraging geschat op {r['vertraging']:.1f} s i.p.v. 0,6 s"
        assert fout < 0.25, f"{fout:.2f} m van het doel"
    return f"GPS 0,6 s te laat en 4 % slip: vertraging geschat op {r['vertraging']:.1f} s, {fout * 100:.0f} cm van het doel"


def test_zonder_gps():
    with simulatie(gps=False) as (sim, poort):
        pts = punten(sim, (0, 0), (0, 3), (2, 3))
        r = route(poort, pts, richting=0.0)
        fout = echte_fout(sim, pts[-1])
        assert r["gps"] is None and r["gps_zinnen"] == 0 and fout < 0.15, (r["gps"], fout)
        try:
            route(poort, pts, kalibreer_noord=2.0)
            raise AssertionError("het noorden zoeken zonder GPS had moeten stoppen")
        except SystemExit as e:
            assert "--richting" in str(e), e
    return f"zonder GPS: route gereden op de wielen ({fout * 100:.0f} cm van het doel); het noorden zoeken vraagt om --richting"


def test_gps_afstand():
    with simulatie(slip=0.05, gps_ruis=0.03) as (sim, poort):
        with robot_session(False, ip="127.0.0.1", port=poort, live=False) as robot:
            res, gereden = meet(robot, 8.0, rest=1.5)
        echt = math.hypot(sim.x, sim.y)
        assert res and abs(res["schaal"] - 0.95) < 0.01, res
        assert abs(gereden * res["schaal"] - echt) < 0.08, (gereden, res["schaal"], echt)
    return f"5 % slip: wielen {gereden:.2f} m, echt {echt:.2f} m, schaal volgens de GPS {res['schaal']:.3f}"


def main():
    sys.stdout.reconfigure(errors="replace")
    kies = sys.argv[1:]
    tests = [f for n, f in sorted(globals().items()) if n.startswith("test_") and (not kies or any(k in n for k in kies))]
    fouten = 0
    for test in tests:
        t0 = time.monotonic()
        try:
            print(f"ok    {test.__name__}: {test()}  ({time.monotonic() - t0:.0f} s)", flush=True)
        except AssertionError as e:
            fouten += 1
            print(f"FOUT  {test.__name__}: {e}", flush=True)
    print("ALLES OK" if not fouten else f"{fouten} test(en) FOUT")
    return 1 if fouten else 0


if __name__ == "__main__":
    sys.exit(main())
