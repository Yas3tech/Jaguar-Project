"""Controles voor het rijden zelf, tegen de gesimuleerde robot (sim_robot.py), zonder Jaguar:
  python -m jaguar.testen.test_rijden            alles (ongeveer vier minuten)
  python -m jaguar.testen.test_rijden vijf       alleen de tests met "vijf" in de naam

Wat hier bewezen wordt, in de wereld van de simulatie (waar het robot echt staat, niet wat de odometrie denkt):
  - 5 m rijden gebeurt in één beweging: het robot rijdt geen centimeter achteruit en stopt één keer, op het doel;
  - dat blijft zo met tragere motoren, een andere snelheid en meer vermogen (de uitloop wordt bijgeleerd);
  - heen en terug, en een vierkant met draaien: het robot eindigt op de start;
  - alleen MM0 meldt encoders (zoals een simulator kan doen): het robot rijdt toch;
  - met --precies werkt de oude manier (kruipstoten) nog.
De simulatie is een model (vertraging, dode zone, slip bij draaien): geen bewijs voor de echte robot.
"""
import math
import socket
import sys
import threading
import time
from contextlib import contextmanager

from jaguar.kern.robot import prepare, robot_session
from jaguar.rijden.parcours import rijd
from jaguar.testen.sim_robot import Sim, serve


@contextmanager
def simulatie(**opties):
    """Een gesimuleerde robot op een vrije poort; geeft (sim, poort)."""
    srv = socket.create_server(("127.0.0.1", 0))
    sim = Sim(**opties)
    threading.Thread(target=serve, args=(srv, sim, False), daemon=True).start()
    try:
        yield sim, srv.getsockname()[1]
    finally:
        sim.running = False
        srv.close()


@contextmanager
def robot_op(poort, rust=1.0):
    """Verbonden robot na de rustmeting, zonder CSV en zonder live pagina."""
    with robot_session(False, ip="127.0.0.1", port=poort, live=False) as robot:
        prepare(robot, rust)
        yield robot


def hoek(a):
    return (a + 180) % 360 - 180


def rechtuit(sim, robot, meter, power=250, **kw):
    """`meter` rijden en meten wat er in de wereld gebeurde: (echt gereden, achteruit gereden, aantal stops)."""
    x0, y0 = sim.x, sim.y
    sim.tellers_nul()
    robot.phase = "rijden"
    robot.drive_distance(power if meter >= 0 else -power, abs(meter), **kw)
    robot.hold(0.5)
    d = math.hypot(sim.x - x0, sim.y - y0)
    terug = sim.achteruit_m if meter >= 0 else sim.vooruit_m
    return d, terug, sim.stops


def test_vijf_meter_in_een_keer():
    with simulatie() as (sim, poort), robot_op(poort) as robot:
        d, terug, stops = rechtuit(sim, robot, 5.0)
        assert terug < 0.002, f"{terug * 100:.1f} cm achteruit gereden"
        assert stops == 1, f"{stops} keer gestopt in plaats van 1"
        assert abs(d - 5.0) < 0.03, f"5 m gevraagd, {d:.3f} m gereden"
        fout1 = d - 5.0
        d, terug, stops = rechtuit(sim, robot, 5.0)          # tweede keer: de uitloop is bijgeleerd
        assert terug < 0.002 and stops == 1 and abs(d - 5.0) < 0.02, (d, terug, stops)
        zij = abs(sim.y)
        assert zij < 0.05 and abs(hoek(sim.h)) < 2.0, f"{zij * 100:.1f} cm naast de lijn, koers {sim.h:.1f}°"
    return f"fout {fout1 * 1000:+.0f} mm en {(d - 5.0) * 1000:+.0f} mm, 0 cm achteruit, telkens 1 stop, {zij * 100:.1f} cm naast de lijn na 10 m"


def test_vijf_meter_andere_robots():
    """Tragere motoren, andere snelheid, meer vermogen: de eerste keer mag er een stukje bijgereden worden
    (vooruit), maar nooit achteruit; de tweede keer is het één beweging."""
    uit = []
    for naam, opties, power in (("trage motoren", {"vertraging": 0.45}, 250), ("snel", {"snelheid": 1.1}, 250),
                                ("traag", {"snelheid": 0.7}, 250), ("vermogen 450", {}, 450)):
        with simulatie(**opties) as (sim, poort), robot_op(poort) as robot:
            d1, terug1, stops1 = rechtuit(sim, robot, 5.0, power)
            d2, terug2, stops2 = rechtuit(sim, robot, 5.0, power)
            assert terug1 < 0.002 and terug2 < 0.002, f"{naam}: achteruit gereden ({terug1:.3f}, {terug2:.3f} m)"
            assert abs(d1 - 5.0) < 0.08 and abs(d2 - 5.0) < 0.03, f"{naam}: {d1:.3f} en {d2:.3f} m"
            assert stops2 == 1, f"{naam}: tweede keer {stops2} stops"
            uit.append(f"{naam} {(d1 - 5.0) * 1000:+.0f}/{(d2 - 5.0) * 1000:+.0f} mm ({stops1}/{stops2} stops)")
    return "; ".join(uit)


def test_kort_en_achteruit():
    with simulatie() as (sim, poort), robot_op(poort) as robot:
        uit = []
        for meter in (0.3, 1.0, -2.0, 0.05):
            d, terug, stops = rechtuit(sim, robot, meter)
            assert terug < 0.002, f"{meter} m: {terug * 100:.1f} cm de verkeerde kant op"
            assert abs(d - abs(meter)) < 0.03, f"{meter} m gevraagd, {d:.3f} m gereden"
            uit.append(f"{meter:+g} m: {(d - abs(meter)) * 1000:+.0f} mm")
    return ", ".join(uit)


def test_vierkant():
    with simulatie() as (sim, poort), robot_op(poort) as robot:
        sim.tellers_nul()
        rijd(robot, [("vooruit", 2.0), ("links", 90), ("vooruit", 2.0), ("links", 90), ("vooruit", 2.0), ("links", 90),
                     ("vooruit", 2.0), ("links", 90)], pause=0.3)
        d, h = math.hypot(sim.x, sim.y), abs(hoek(sim.h))
        assert d < 0.10 and h < 3.0, f"{d * 100:.1f} cm en {h:.1f}° van de start"
        assert sim.achteruit_m < 0.03, f"{sim.achteruit_m * 100:.1f} cm achteruit gereden in het vierkant"
    return f"8 m vierkant: {d * 100:.1f} cm en {h:.1f}° van de start, {sim.achteruit_m * 100:.1f} cm achteruit"


def test_heen_en_terug():
    with simulatie() as (sim, poort), robot_op(poort) as robot:
        rijd(robot, [("vooruit", 3.0), ("achteruit", 3.0)], pause=0.3)
        d = math.hypot(sim.x, sim.y)
        assert d < 0.04, f"{d * 100:.1f} cm van de start"
    return f"3 m heen en terug: {d * 100:.1f} cm van de start"


def test_alleen_voordriver():
    with simulatie(achterdriver=False) as (sim, poort), robot_op(poort, rust=2.0) as robot:
        d, terug, _ = rechtuit(sim, robot, 2.0)
        assert abs(d - 2.0) < 0.04 and terug < 0.002, (d, terug)
    return f"alleen MM0: 2 m gevraagd, {d:.3f} m gereden"


def test_precies():
    with simulatie() as (sim, poort), robot_op(poort) as robot:
        robot.precies = True
        d, _, stops = rechtuit(sim, robot, 1.5, tolerance=0.003)
        assert abs(d - 1.5) < 0.012, f"{d:.4f} m"
    return f"--precies: 1,5 m gevraagd, {d:.4f} m gereden ({stops} stops)"


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
