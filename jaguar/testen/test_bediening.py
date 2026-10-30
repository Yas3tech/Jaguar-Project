"""Controles voor bediening.py tegen de gesimuleerde robot (sim_robot.py), zonder Jaguar en zonder browser:
  python -m jaguar.testen.test_bediening

Pauze en hervatten, Terug naar start (tijdens en na de rit), een lege batterij, handmatig rijden met de
dodemansknop, en STOP tijdens een pauze. Duurt ongeveer twee minuten; schrijft geen CSV's.
Gemeten wordt in de wereld van de simulatie (waar het robot echt staat), niet met de odometrie van robot.py.
"""
import argparse
import math
import socket
import sys
import threading
import time

from jaguar.live import view as live_view
from jaguar.rijden.bediening import Control, samenstellen
from jaguar.rijden.parcours import naar_start
from jaguar.testen.sim_robot import Sim, serve

POORT = 10011
TOL_M, TOL_GRADEN = 0.05, 2.5


def hoek(a):
    return (a + 180) % 360 - 180


def wacht(voorwaarde, timeout, wat):
    t0 = time.monotonic()
    while not voorwaarde():
        if time.monotonic() - t0 > timeout:
            raise AssertionError(f"na {timeout} s nog niet: {wat}")
        time.sleep(0.05)


def klaar(c, timeout=90):
    wacht(lambda: not c.state["bezig"], timeout, "rit klaar")
    return c.state


def pose(sim):
    return sim.x, sim.y, sim.h


def afstand(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(hoek(a[2] - b[2]))


def test_naar_start():
    assert naar_start(0, 0, 0) == []
    assert naar_start(2, 0, 0) == [("achteruit", 2.0)]                       # recht vooruit gereden: achteruit terug
    assert naar_start(5, 0, 0) == [("links", 180.0), ("vooruit", 5.0), ("links", 180.0)]   # verder dan 3 m: keren
    st = naar_start(1, 1, 90)                                                  # vierkant half gereden, kijkt naar links
    x, y, k = 1.0, 1.0, 90.0
    for soort, v in st:                                                        # stappen nadoen: moet op de start eindigen
        if soort in ("links", "rechts"):
            k += v if soort == "links" else -v
        else:
            d = v if soort == "vooruit" else -v
            x, y = x + d * math.cos(math.radians(k)), y + d * math.sin(math.radians(k))
    assert math.hypot(x, y) < 1e-3 and abs(hoek(k)) < 1e-6, (st, x, y, k)
    a = samenstellen((1.0, 2.0, 90.0), 1.0, 0.0, 10.0)                         # 1 m vooruit vanaf (1, 2) die naar links kijkt
    assert abs(a[0] - 1) < 1e-9 and abs(a[1] - 3) < 1e-9 and a[2] == 100.0
    print("naar_start en samenstellen: OK")


def test_pauze(c, sim):
    p0 = pose(sim)
    assert c.start({"stappen": [["vooruit", 2.0]], "power": 250})[0] == 200
    wacht(lambda: c.robot is not None and c.robot.phase == "1-vooruit" and math.hypot(sim.x - p0[0], sim.y - p0[1]) > 0.6,
          30, "halverwege de eerste stap")
    assert c.pauze({"aan": True})[0] == 200
    wacht(lambda: c.robot.phase == "gepauzeerd", 5, "fase gepauzeerd")
    time.sleep(1.0)
    p1 = pose(sim)
    time.sleep(2.0)
    stil, _ = afstand(p1, pose(sim))
    assert stil < 0.003, f"robot bewoog {stil * 100:.1f} cm tijdens de pauze"
    assert c.toestand()["gepauzeerd"]
    assert c.pauze({"aan": False})[0] == 200
    st = klaar(c)
    d, _ = afstand(p0, pose(sim))
    assert st["resultaat"] == "klaar" and abs(d - 2.0) < 0.02, (st, d)
    print(f"pauze en hervatten: OK (2 m gevraagd, {d:.3f} m gereden met een pauze halverwege)")


def test_terug_tijdens_rit(c, sim):
    p0 = pose(sim)
    c.start({"stappen": [["vooruit", 1.5], ["links", 90], ["vooruit", 1.0]], "power": 250})
    wacht(lambda: c.robot is not None and c.robot.phase == "3-vooruit" and c.robot.odo.y > 0.4, 60, "in de derde stap")
    assert c.terug()[0] == 200
    st = klaar(c)
    d, h = afstand(p0, pose(sim))
    assert st["resultaat"] == "terug" and d < TOL_M and h < TOL_GRADEN, (st, d, h)
    assert c.toestand()["thuis"] is None, c.toestand()["thuis"]                   # op de start: geen knop meer
    print(f"terug naar start tijdens de rit: OK ({d * 100:.1f} cm en {h:.1f}° van de start)")


def test_terug_na_rit(c, sim):
    p0 = pose(sim)
    c.start({"stappen": [["vooruit", 1.0], ["rechts", 45], ["vooruit", 0.8]], "power": 250})
    assert klaar(c)["resultaat"] == "klaar"
    thuis = c.toestand()["thuis"]
    assert thuis and abs(thuis["afstand"] - 1.66) < 0.05, thuis
    assert c.terug()[0] == 200
    st = klaar(c)
    d, h = afstand(p0, pose(sim))
    assert st["resultaat"] == "klaar" and st["soort"] == "terug" and d < TOL_M and h < TOL_GRADEN, (st, d, h)
    print(f"terug naar start na de rit: OK ({d * 100:.1f} cm en {h:.1f}° van de start, stappen {thuis['stappen']})")


def test_batterij(c, sim):
    p0 = pose(sim)
    # 3 V per minuut is al veel te snel (een echte batterij doet er een uur over), maar de mediaan over 10 s loopt dan
    # nog niet zo ver achter dat ze op de terugweg ook onder BAT_NOOD_V zakt.
    sim.bat, sim.verbruik = 22.6, 3.0
    try:
        c.start({"stappen": [["vooruit", 15.0]], "power": 250})
        wacht(lambda: c.state["soort"] == "terug" or not c.state["bezig"], 60, "batterij laag")
        sim.verbruik = 0.0                 # anders wordt ze op de terugweg leeg (BatterijLeeg)
        st = klaar(c)
    finally:
        sim.bat, sim.verbruik = 24.0, 0.0
    d, h = afstand(p0, pose(sim))
    assert st["resultaat"] == "batterij" and d < 0.01 * 13, (st, d)    # 13 m terug: 1 %
    print(f"batterij laag: OK (parcours afgebroken, {d * 100:.1f} cm van de start; {st['fout']})")


def test_handmatig(c, sim):
    p0 = pose(sim)
    assert c.handmatig({"aan": True, "max": 250})[0] == 200
    wacht(lambda: c.robot is not None and c.robot.phase == "handmatig", 20, "handmatig")

    def sturen(v, w, s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < s:
            c.stuur({"v": v, "w": w})
            time.sleep(0.1)
    sturen(1.0, 0.0, 2.0)
    d, h = afstand(p0, pose(sim))
    assert d > 0.6 and h < 3, f"vooruit: {d:.2f} m, {h:.1f}°"
    time.sleep(0.6)                        # niets meer sturen: dodemansknop
    assert c.robot.power == 0, f"dodemansknop: vermogen nog {c.robot.power} na 0,6 s"
    time.sleep(1.5)                        # uitrollen (0,25 s vertraging in de simulatie)
    p1 = pose(sim)
    time.sleep(1.0)
    stil, _ = afstand(p1, pose(sim))
    assert stil < 0.005 and abs(sim.vl) + abs(sim.vr) < 0.01, f"dodemansknop: nog {stil * 100:.1f} cm"
    h0 = sim.h
    sturen(0.0, 1.0, 1.5)
    assert hoek(sim.h - h0) > 20, f"links draaien: {hoek(sim.h - h0):.1f}°"
    c.handmatig({"aan": False})
    assert klaar(c)["resultaat"] == "klaar"
    c.terug()
    st = klaar(c)
    d, h = afstand(p0, pose(sim))
    assert d < TOL_M and h < TOL_GRADEN, (st, d, h)
    print(f"handmatig rijden en dodemansknop: OK; daarna terug naar de start ({d * 100:.1f} cm, {h:.1f}°)")


def test_stop_in_pauze(c, sim):
    c.start({"stappen": [["vooruit", 1.5]], "power": 250})
    wacht(lambda: c.robot is not None and c.robot.phase == "1-vooruit", 30, "rijden")
    time.sleep(0.8)
    c.pauze({"aan": True})
    wacht(lambda: c.robot.phase == "gepauzeerd", 5, "gepauzeerd")
    c.stop()
    assert klaar(c, 10)["resultaat"] == "gestopt"
    c.stop_requested = False
    print("STOP tijdens een pauze: OK")


def main():
    test_naar_start()
    srv = socket.create_server(("127.0.0.1", POORT))
    sim = Sim(24.0, 0.0)
    threading.Thread(target=serve, args=(srv, sim, False), daemon=True).start()
    args = argparse.Namespace(ip="127.0.0.1", port=POORT, rest=1.0, mqtt=False, mqtt_host="", csv=False)
    hub = live_view.Hub(live_view.LiveView("test"))
    c = Control(hub, args)
    hub.control = c
    fouten = 0
    for test in (test_pauze, test_terug_tijdens_rit, test_terug_na_rit, test_batterij, test_handmatig, test_stop_in_pauze):
        try:
            test(c, sim)
        except AssertionError as e:
            fouten += 1
            print(f"{test.__name__}: FOUT {e}")
            c.stop()
            klaar(c, 15)
            c.stop_requested = False
    print("ALLES OK" if not fouten else f"{fouten} test(en) FOUT")
    return 1 if fouten else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")
    sys.exit(main())
