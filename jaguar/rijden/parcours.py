"""Parcours rijden: een reeks stappen vooruit, achteruit en ter plaatse draaien.

  python -m jaguar.rijden.parcours vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5
  python -m jaguar.rijden.parcours v:2 l:90 v:2 l:90 v:2 l:90 v:2          (vierkant)

Stappen: vooruit:<m>, achteruit:<m>, links:<graden>, rechts:<graden> (afgekort v, a, l, r).
Rijden gebruikt de slipbewuste odometrie (zoals robot.py), draaien de gyro.
Na een draai volgt de robot de nieuwe koers; opeenvolgende rechte stukken blijven op dezelfde lijn.
Daarna: python -m jaguar.analyse.rit, en het dashboard toont de geplande en de gereden route.
Draaien is zo juist als de gyroschaal: kalibreer eerst met  python -m jaguar.kalibratie.test_sondewiel draai.

Obstakels (jaguar/camera/obstakel.py, als Obstakels aan staat in Opties en de camera gekalibreerd is): de robot
remt af en stopt voor wat hij ziet, en gaat dan verder met het parcours:
  - wacht tot 4 s: loopt iemand weg, dan rijdt hij gewoon door;
  - is er naast het obstakel vrije vloer: omweg (45° opzij, voorbij, terug op de lijn) en verder;
  - anders (muur dwars over de weg): deze stap eindigt hier en de rest van het parcours schuift mee.

Pauze (alleen met bediening.py): elke stap rijdt na Hervatten verder naar hetzelfde geplande hoekpunt of dezelfde koers.
"""
import argparse
import math
import time

from jaguar.kern.robot import (CameraWeg, Gepauzeerd, Obstakel, Vastgelopen, add_drive_args, positive_seconds, prepare,
                               robot_session)
from jaguar.navigatie.gps import hoek180

MAX_CORRECTIE = 0.5   # m: grotere correctie voor de vorige stappen vertrouwen we niet (bv. haperende odometrie)
TERUG_ACHTERUIT_M = 3.0   # naar de start: ligt die achter het robot en dichterbij dan dit, dan achteruit in plaats van keren
OBST_WACHT_S = 4.0    # s wachten of een obstakel (iemand) wegloopt
OBST_VRIJ_M = 1.0     # m: zoveel verder dan de stopafstand moet de weg vrij zijn om door te rijden
OMWEG_TERUG = 0.5     # m eerst achteruit (daar kwam hij net vandaan): ruimte om opzij te draaien
OMWEG_HOEK = 45.0     # graden opzij voor een omweg
OMWEG_ZIJ = 0.9       # m zijwaarts: halve robot + half obstakel + marge
OMWEG_VOORBIJ = 1.2   # m rechtdoor naast het obstakel (de diepte ziet de camera niet)
MAX_OMWEGEN = 2       # per stap; daarna eindigt de stap aan het obstakel
STAPPEN = {"vooruit": "vooruit", "v": "vooruit", "achteruit": "achteruit", "a": "achteruit",
           "links": "links", "l": "links", "rechts": "rechts", "r": "rechts"}


def stap(text):
    """'vooruit:5' -> ('vooruit', 5.0)."""
    soort, _, waarde = text.lower().partition(":")
    if soort not in STAPPEN or not waarde:
        raise argparse.ArgumentTypeError(f"onbekende stap '{text}': gebruik vooruit:<m>, achteruit:<m>, links:<graden> of rechts:<graden>")
    try:
        value = float(waarde.replace(",", "."))
    except ValueError:
        raise argparse.ArgumentTypeError(f"geen getal in '{text}'") from None
    if not 0 < value < 1000:
        raise argparse.ArgumentTypeError(f"waarde in '{text}' moet positief zijn")
    return STAPPEN[soort], value


def main():
    ap = argparse.ArgumentParser(description="Parcours rijden: vooruit, achteruit, links/rechts draaien")
    ap.add_argument("stappen", nargs="+", type=stap, help="bv. vooruit:5 achteruit:4.5 rechts:90 vooruit:1.5")
    ap.add_argument("--power", type=int, default=250, help="maximaal rijvermogen (0..1000)")
    ap.add_argument("--turn-power", type=int, default=380, help="maximaal vermogensverschil bij draaien")
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.0, help="seconden stilstaan tussen de stappen")
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()
    if not 0 < args.power <= 1000 or not 0 < args.turn_power <= 1000:
        ap.error("--power en --turn-power moeten tussen 1 en 1000 liggen")
    print("Parcours: " + " → ".join(f"{s} {v:g} {'m' if s in ('vooruit', 'achteruit') else '°'}" for s, v in args.stappen))
    with robot_session(args.csv, "_parcours", args.ip, args.port, args.live, args.mqtt,
                       args.mqtt_host, args.network) as robot:
        robot.precies = args.precies
        prepare(robot, args.rest)
        rijd(robot, args.stappen, args.power, args.turn_power, args.pause, args.ramp, args.creep, args.tolerance)


def rijd(robot, stappen, power=250, turn_power=380, pause=1.0, ramp=1.5, creep=60, tolerance=10.0, prefix=""):
    """De stappen na elkaar rijden (na prepare). Ook gebruikt door bediening.py (knop op de live pagina).
    prefix: voor de fasenamen (bv. "terug-" voor de terugweg naar de start, zodat de analyse de stappen apart houdt).

    Rechte stukken rijden naar het geplande hoekpunt (gerekend vanaf de start) en volgen de geplande
    lijn, niet telkens de nominale afstand vanaf waar de robot toevallig staat. Zo werkt elke stap de
    afwijking van de vorige (te ver, te kort, naast de lijn) vanzelf weg in plaats van ze op te stapelen
    (rit 122109: 25 cm aan het einde na 15 stappen). Draaien gaat al naar de geplande absolute koers.
    Alles volgens de odometrie: wat die niet ziet (slip zonder signaal, gyroschaal) blijft staan."""
    px, py = robot.odo.x, robot.odo.y                 # gepland hoekpunt

    def vanaf_hier():                                 # na een pauze midden in een draai of boog: de rest vanaf de koers nu
        robot.target_heading = robot.heading

    for i, stap_ in enumerate(stappen, 1):
        soort, value = stap_[0], stap_[1]
        if i > 1:
            robot.phase = "pauze"
            robot.hold(pause)
        if soort == "boog":                               # ("boog", radius m, hoek graden, + = links)
            robot.phase = f"{prefix}{i}-boog"
            h = math.radians(robot.target_heading)
            r, a = float(value), float(stap_[2])
            s_, c_ = math.sin(math.radians(abs(a))), 1 - math.cos(math.radians(abs(a)))
            kant = 1 if a > 0 else -1
            dx, dy = r * s_, kant * r * c_                # eindpunt van de boog in het stelsel van de beginkoers
            px, py = px + dx * math.cos(h) - dy * math.sin(h), py + dx * math.sin(h) + dy * math.cos(h)
            robot.note(f"PLAN {robot.phase} {r:.4f} {a:.2f}")
            eind, gereden = robot.target_heading + a, []

            def boog(r=r, eind=eind, gereden=gereden):
                if not gereden and abs(eind - robot.target_heading) >= 1.0:   # pauze in de eindkoers: geen nieuwe boog
                    robot.arc(r, eind - robot.target_heading, power, ramp)
                gereden.append(True)
                robot.target_heading = eind
                robot.eindkoers(turn_power)
            robot.met_pauze(boog, vanaf_hier)
        elif soort in ("vooruit", "achteruit"):
            robot.phase = f"{prefix}{i}-{soort}"
            planned = value if soort == "vooruit" else -value
            robot.note(f"PLAN {robot.phase} {planned:.4f}")
            px, py = rechtdoor(robot, (px, py), planned, power, turn_power, ramp, creep, tolerance)
        else:
            robot.phase = f"{prefix}{i}-draai"
            eind = robot.target_heading + (value if soort == "links" else -value)
            robot.met_pauze(lambda eind=eind: robot.turn(eind - robot.target_heading, turn_power), vanaf_hier)
    # Eindkoers: bv. een kant die bij het kruipen vastzat en de robot liet zwenken (rit 122109: 20°).
    robot.met_pauze(lambda: robot.eindkoers(turn_power))
    robot.phase = "uitbollen"
    robot.hold(2.0)
    x, y = robot.odo.x, robot.odo.y
    print(f"Einde: {x:+.3f} m vooruit, {y * 100:+.1f} cm links t.o.v. de start, koers {robot.heading:+.1f}° (schatting)")
    return px, py


MAX_BIJSTUREN = 2     # zo vaak mag een punt opnieuw aangereden worden als de GPS zegt dat het robot ernaast staat
MIN_STAP = 0.15       # m: dichter dan dit bij het punt = er al


def rijd_punten(robot, n, doel, power=250, turn_power=380, pause=1.0, ramp=1.5, creep=60, tolerance=10.0):
    """Van punt naar punt rijden met een positie die onderweg beter wordt (GPS): doel(i) geeft voor punt i
    (0 … n−1) waar het NU ligt in het stelsel van de odometrie, als (x, y, marge). Voor elk punt: draaien naar het
    punt, in één keer erheen rijden, en dan opnieuw vragen waar het ligt. Ligt het verder dan `marge` (de GPS heeft
    de schatting onderweg verbeterd), dan rijdt het robot het stuk bij, hoogstens MAX_BIJSTUREN keer.
    Zonder GPS geeft doel() steeds hetzelfde punt en is dit gewoon van punt naar punt rijden op de odometrie."""
    k = 0
    for i in range(n):
        for poging in range(1 + MAX_BIJSTUREN):
            x, y, marge = doel(i)
            dx, dy = x - robot.odo.x, y - robot.odo.y
            afstand = math.hypot(dx, dy)
            if afstand < (MIN_STAP if poging == 0 else max(MIN_STAP, marge)):
                break
            if poging:
                robot.melding(f"punt {i + 1}: volgens de GPS nog {afstand:.2f} m ervandaan, bijsturen")
            if k:
                robot.phase = "pauze"
                robot.hold(pause)
            eind = robot.target_heading + hoek180(math.degrees(math.atan2(dy, dx)) - robot.target_heading)
            if abs(eind - robot.target_heading) >= 1.0:
                k += 1
                robot.phase = f"{k}-draai"
                robot.met_pauze(lambda eind=eind: robot.turn(eind - robot.target_heading, turn_power),
                                lambda: setattr(robot, "target_heading", robot.heading))
            k += 1
            robot.phase = f"{k}-vooruit"
            robot.note(f"PLAN {robot.phase} {afstand:.4f}")
            rechtdoor(robot, (robot.odo.x, robot.odo.y), afstand, power, turn_power, ramp, creep, tolerance)
    robot.phase = "uitbollen"
    robot.hold(2.0)


def naar_start(x, y, koers):
    """Stappen terug naar de start (0, 0) met de beginrichting (koers 0), vanaf (x, y) in m en `koers` in graden,
    alles in het stelsel van de start (+ = links): draaien naar de start, rechtdoor, draaien naar de beginrichting.
    Ligt de start achter het robot en dichterbij dan TERUG_ACHTERUIT_M, dan achteruit: één draai minder, en
    meestal over de weg waarlangs het robot kwam. Stukken onder 2 cm en draaien onder 1° vallen weg."""
    stappen, richting = [], koers
    afstand = math.hypot(x, y)
    if afstand >= 0.02:
        a = hoek180(math.degrees(math.atan2(-y, -x)) - koers)
        achteruit = abs(a) > 90 and afstand <= TERUG_ACHTERUIT_M
        if achteruit:
            a = hoek180(a + 180)
        if abs(a) >= 1:
            stappen.append(("links" if a > 0 else "rechts", round(abs(a), 1)))
        richting = koers + a
        stappen.append(("achteruit" if achteruit else "vooruit", round(afstand, 3)))
    a = hoek180(-richting)
    if abs(a) >= 1:
        stappen.append(("links" if a > 0 else "rechts", round(abs(a), 1)))
    return stappen


def rechtdoor(robot, start, planned, power, turn_power, ramp, creep, tolerance):
    """Eén rechte stap van het geplande hoekpunt `start` over `planned` m (− = achteruit) op de
    gewenste koers. Geeft het hoekpunt terug van waaruit het parcours verdergaat: het geplande,
    of de plek op de lijn waar de stap voor een obstakel eindigde."""
    heading = robot.target_heading
    h = math.radians(heading)
    ux, uy = math.cos(h), math.sin(h)

    def along():                                       # afgelegd op de geplande lijn vanaf start
        return (robot.odo.x - start[0]) * ux + (robot.odo.y - start[1]) * uy

    togo = planned - along()
    if abs(togo - planned) > MAX_CORRECTIE:
        print(f"{robot.phase}: correctie {(togo - planned) * 100:+.0f} cm is te groot (odometrie onbetrouwbaar?): "
              f"de geplande {planned:+.3f} m rijden zonder correctie")
        start = (start[0] + (planned - togo) * ux, start[1] + (planned - togo) * uy)   # lijn verschuift mee
    elif abs(togo - planned) >= 0.005:
        print(f"{robot.phase}: gepland {planned:+.3f} m, te rijden {togo:+.3f} m "
              f"(correctie {(togo - planned) * 100:+.1f} cm voor de vorige stappen)")
    end = (start[0] + planned * ux, start[1] + planned * uy)
    detours = waits = 0
    while True:
        if abs(robot.target_heading - heading) > 0.5:   # na een pauze midden in een omweg: eerst weer op koers
            robot.met_pauze(lambda: robot.turn(heading - robot.target_heading, turn_power, label="OMWEG"))
        robot.target_heading, robot.line = heading, start   # de geplande lijn
        togo = planned - along()
        try:
            robot.drive_distance(power if togo >= 0 else -power, abs(togo), ramp, creep, tolerance=tolerance / 1000)
            if robot.vast:
                raise Vastgelopen(f"vastgelopen na {along():.2f} m van {planned:.2f} m ({robot.vast}): het parcours stopt hier")
            return end
        except Gepauzeerd:
            robot.pauzeren()                            # daarna verder naar hetzelfde hoekpunt
            continue
        except (CameraWeg, Vastgelopen):
            raise                                       # blind of vast: niet verder rijden
        except Obstakel as e:
            robot.hold(0.3)
            robot.melding(f"{robot.phase}: {e}")
        det = robot.obstacle
        if waits < 3 and wait_clear(robot):             # iemand liep weg
            waits += 1
            robot.melding(f"{robot.phase}: weg vrij, verder")
            continue
        side = det.free_side((det.dist or 0) + OBST_VRIJ_M)
        rest = planned - along()
        if side and detours < MAX_OMWEGEN and rest > det.stop + OBST_VRIJ_M:   # anders ligt het doel vlak achter het obstakel
            detours += 1
            try:
                detour(robot, side, heading, power, turn_power, ramp, creep, tolerance)
                continue
            except Gepauzeerd:
                robot.pauzeren()
                continue
            except CameraWeg:
                raise
            except Obstakel as e:
                robot.hold(0.3)
                robot.melding(f"{robot.phase}: omweg lukt niet ({e})")
        # Muur dwars over de weg of geen omweg mogelijk: de stap eindigt hier, het parcours gaat verder.
        if abs(robot.target_heading - heading) > 0.5:
            robot.met_pauze(lambda: robot.turn(heading - robot.target_heading, turn_power, label="OMWEG"))
        a = along()
        robot.melding(f"{robot.phase}: stap ingekort tot {a:.2f} m van {planned:.2f} m (obstakel); "
                      "het parcours gaat verder vanaf hier")
        robot.target_heading = heading
        return start[0] + a * ux, start[1] + a * uy


def wait_clear(robot):
    """Tot OBST_WACHT_S stilstaan; True als de weg vrij komt (bv. iemand loopt weg)."""
    det = robot.obstacle
    robot.melding(f"{robot.phase}: obstakel op {det.dist:.1f} m, even wachten…" if det.dist else f"{robot.phase}: even wachten…")
    t0 = time.monotonic()
    while time.monotonic() - t0 < OBST_WACHT_S:
        robot.hold(0.2)
        d, age = det.latest()
        if age < 1.0 and (d is None or d > det.stop + OBST_VRIJ_M):
            return True
    return False


def detour(robot, side, heading, power, turn_power, ramp, creep, tolerance):
    """Omweg: OMWEG_TERUG achteruit, OMWEG_HOEK opzij, OMWEG_ZIJ zijwaarts, OMWEG_VOORBIJ voorbij, terug
    op de lijn en weer op de gewenste koers. Elk vooruit stuk let zelf op obstakels (Obstakel als daar
    ook iets staat). Zonder die eerste stap achteruit staat het obstakel na het opzij draaien nog
    in de weg (simulatie: tas van 0,4 m op 0,6 m)."""
    s = 1 if side == "links" else -1
    phase = robot.phase
    diag = OMWEG_ZIJ / math.sin(math.radians(OMWEG_HOEK))
    robot.melding(f"{phase}: omweg {side} om het obstakel")
    robot.phase = f"{phase}-omweg"
    try:
        robot.drive_distance(-power, OMWEG_TERUG, ramp, creep, tolerance=tolerance / 1000)
        for angle, dist in ((s * OMWEG_HOEK, diag), (-s * OMWEG_HOEK, OMWEG_VOORBIJ), (-s * OMWEG_HOEK, diag)):
            robot.turn(angle, turn_power, label="OMWEG")
            robot.drive_distance(power, dist, ramp, creep, tolerance=tolerance / 1000)
        robot.turn(heading - robot.target_heading, turn_power, label="OMWEG")
    finally:
        robot.phase = phase


if __name__ == "__main__":
    main()
