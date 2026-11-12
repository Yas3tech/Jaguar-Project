"""Naar GPS-coördinaten rijden: één punt of een hele route.

  python -m jaguar.rijden.gps_route --vanaf-hier 50.84010,4.32000                       ga naar dit punt
  python -m jaguar.rijden.gps_route --vanaf-hier <lat,lon> <lat,lon> ...                langs deze punten
  python -m jaguar.rijden.gps_route <lat,lon> <lat,lon> ...  --richting 90              het eerste punt is de start
  python -m jaguar.rijden.gps_route 50.84000,4.32000 50.84010,4.32000 --richting 0 --droog

Waar het robot begint:
  - standaard is het eerste punt de start: daar staat het robot (coördinaat 1 = nulpunt). Dat is het nauwkeurigst:
    de route hangt dan niet af van waar de GPS denkt dat het robot staat;
  - met --vanaf-hier meet het robot zelf waar het staat (gemiddelde van de GPS in rust, --gps-wacht seconden) en
    zijn alle punten doelen. De fout van de GPS op dat moment (meters) zit dan in de hele route.
Het noorden (de GPS zegt waar het robot is, niet waarheen het kijkt; het kompas is door de motoren onbruikbaar):
  --richting <graden>   waar het robot bij de start naartoe kijkt (kompaskoers, 0 = noord, 90 = oost)
  --kalibreer-noord <m> rijdt eerst <m> rechtdoor (standaard 6 m als er geen --richting is) en bepaalt het noorden
                        uit de koers over de grond en uit het gereden spoor; de nauwkeurigste van de twee telt.
                        Dat stuk rijdt het robot sneller (--noord-power): de koers over de grond klopt pas boven
                        ongeveer 1 m/s. Daarna rijdt het vanaf waar het staat naar het eerste punt.
Hoe het rijdt, met een GPS die volgens de fabrikant 3 m (WAAS/EGNOS) tot 15 m fout heeft:
  De afstanden en hoeken rijdt het robot met de wielen en de gyro, elk recht stuk in één beweging: over tientallen
  meters is dat veel nauwkeuriger dan de GPS. De GPS wordt gebruikt waar hij beter is, en het script kiest dat zelf
  bij elk punt (--gps auto, de standaard):
    1. altijd: het gereden spoor wordt op het GPS-spoor gelegd (Uitlijning in jaguar/navigatie/gps.py). De draaiing
       daarvan verbetert het noorden, de schaal verbetert de afstand (slip). Een vaste fout van de GPS doet daar
       niets aan af, want alleen de vorm van het spoor telt;
    2. de positie van de GPS zelf telt pas als haar geschatte fout kleiner is dan die van de wielen (2 % van de
       gereden afstand + de onzekerheid op het noorden). Met de GPS van het robot is dat pas na tientallen meters;
       met een nauwkeurige GPS (de websimulator, RTK) meteen;
    3. past het GPS-spoor niet op het gereden spoor (binnen, tussen gebouwen), dan telt de GPS niet.
  De GPS loopt achter op de beweging (Dr Robot noemt dat zelf): het script schat die vertraging en rekent ze weg.
  --gps uit   alleen wielen en gyro        --gps aan   de GPS-positie altijd gebruiken zodra het spoor past
  --droog     alleen tonen wat het zou rijden, zonder het robot (heeft --richting nodig, kan niet met --vanaf-hier)
Na de rit: waar het robot staat volgens de wielen en volgens de GPS, hoe ver dat van het laatste punt is, of het
GPS-spoor op het gereden spoor past (rms), het noorden volgens het spoor en de schaal (echte meters per getelde meter).
Wat geen programma kan: dichter bij een coördinaat komen dan de GPS zelf fout is, als alleen de GPS zegt waar het
robot begint. Zet het robot daarvoor op een bekend punt (eerste coördinaat) en geef de richting op.
"""
import argparse
import math
from collections import deque

from jaguar.kern.robot import Vastgelopen, add_drive_args, positive_seconds, prepare, robot_session
from jaguar.navigatie.gps import Lokaal, NoordPeiling, Uitlijning, enu_naar_robot, hoek180, robot_naar_enu
from jaguar.navigatie.route import beschrijf, route_naar_stappen
from jaguar.rijden.parcours import rijd_punten

NOORD_MAX_ONZEKER = 5.0   # graden: nauwkeuriger moet het noorden zijn om ermee te vertrekken
NOORD_POWER = 400         # vermogen voor het stuk dat het noorden zoekt (≈ 1,1 m/s: koers over de grond bruikbaar)
RICHTING_ONZEKER = 5.0    # graden: zo goed is een richting die de gebruiker opgeeft (op het oog, van de kaart)
FIT_MAX_ONZEKER = 5.0     # graden: slechter passend GPS-spoor telt niet voor de positie
FIT_MIN_BASIS = 1.0       # m: zo breed moet het gereden spoor zijn voor de passing iets zegt
ODO_FOUT = 0.02           # deel van de gereden afstand dat de wielen ernaast kunnen zitten (slip, wielstraal)
GPS_VASTE_FOUT = 1.5      # m: fout van een gewone GPS die niet uitmiddelt (WAAS/EGNOS: 3 m in 95 % van de gevallen)
GPS_NAUWKEURIG = 0.15     # m: past het spoor zo goed, dan is het geen gewone GPS (simulator, RTK): geen vaste fout rekenen
VERTRAGINGEN = 6          # de GPS-vertraging wordt gezocht in stappen van één GPS-zin (0,2 s): 0 tot 1 s
START_MIN_FIXES = 10      # --vanaf-hier: zoveel GPS-posities in rust
START_MAX_SPREIDING = 8.0  # m: springt de GPS in rust verder dan dit, dan is hij geen startpunt (GpsKwaliteit: slecht)


def coordinaat(tekst):
    try:
        lat, lon = (float(x) for x in tekst.replace(";", ",").split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{tekst}': gebruik breedte,lengte, bv. 50.84000,4.32000") from None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise argparse.ArgumentTypeError(f"'{tekst}' ligt niet op aarde")
    return lat, lon


def uitgemiddeld(res):
    """Fout (m) van een positie uit de passing: de rms van de GPS, uitgemiddeld over onafhankelijke metingen
    (GPS-fouten hangen in de tijd samen: één per 5 s)."""
    return res["rms"] / math.sqrt(max(1.0, res["n"] / 25))


class GpsVolger:
    """Krijgt elke GPS-zin van het robot (robot.gps_hook) en houdt alles bij wat de route nodig heeft."""

    def __init__(self, robot, lokaal):
        self.robot, self.lokaal = robot, lokaal
        # Het gereden spoor op het GPS-spoor, voor elke mogelijke vertraging van de GPS (0, 0,2, … 1 s): de GPS-zin
        # van nu hoort bij waar het robot zoveel zinnen geleden was. De vertraging die het best past, telt.
        self.passingen = [Uitlijning(lokaal) for _ in range(VERTRAGINGEN)]
        self._odo = deque(maxlen=VERTRAGINGEN)  # odometrie bij de laatste zinnen
        self.peiling = NoordPeiling()           # het noorden uit de koers over de grond (boven 1 m/s)
        self.recent = deque(maxlen=300)         # laatste posities (oost, noord): 1 minuut
        self.zinnen = self.fixes = 0
        robot.gps_hook = self

    def __call__(self, gps, koers, draai, x, y):
        self.zinnen += 1
        self._odo.append((x, y))
        if not gps["fix"]:
            return
        self.fixes += 1
        self.recent.append(self.lokaal.naar_xy(gps["lat"], gps["lon"]))
        for k, uit in enumerate(self.passingen):
            if k < len(self._odo):
                uit.meting(gps, *self._odo[-1 - k])
        if self.robot.odo.v > 0.5:              # alleen vooruit: achteruit is de koers over de grond omgekeerd
            self.peiling.voeg_toe(gps, koers, draai)

    def gemiddelde(self, n=None):
        """(gemiddelde (oost, noord) van de laatste n posities, spreiding: 95 % ligt zo dicht bij het gemiddelde), of None."""
        pts = list(self.recent)[-(n or len(self.recent)):]
        if not pts:
            return None
        gx, gy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        afst = sorted(math.hypot(p[0] - gx, p[1] - gy) for p in pts)
        return (gx, gy), afst[min(len(afst) - 1, int(0.95 * len(afst)))]

    def beste(self):
        """(passing, uitlijning, vertraging in s) met de vertraging die het best past; (None, None, 0) zonder passing.
        De schaal telt alleen mee als het spoor lang genoeg is (Uitlijning.schaal_bruikbaar), anders schaal 1.
        Een andere vertraging dan 0 moet duidelijk beter passen (5 %), anders blijft het 0."""
        kandidaten = []
        for k, uit in enumerate(self.passingen):
            res = uit.resultaat()
            if res is not None:
                if not Uitlijning.schaal_bruikbaar(res):
                    res = uit.resultaat(met_schaal=False)
                kandidaten.append((res["rms"] * (1.0 if k == 0 else 1.05), k, res, uit))
        if not kandidaten:
            return None, None, 0.0
        _, k, res, uit = min(kandidaten, key=lambda c: c[0])
        return res, uit, 0.2 * k


def noord_zoeken(robot, volger, meter, power, ramp, creep, tolerance):
    """`meter` rechtdoor rijden en het noorden bepalen: (b0, onzekerheid in graden, uitleg). SystemExit als het niet lukt."""
    robot.phase = "noord"
    robot.drive_distance(power, meter, ramp, creep, tolerance=tolerance / 1000)
    robot.hold(1.0)
    kandidaten = []
    res, _, _ = volger.beste()
    if res is not None:
        kandidaten.append((res["onzeker"], res["b0"], f"uit het gereden spoor ({res['n']} GPS-posities, past binnen {res['rms']:.2f} m)"))
    cog = volger.peiling.resultaat()
    if cog is not None:
        n = len(volger.peiling.monsters)
        kandidaten.append((cog[1] / math.sqrt(max(1.0, n / 5)), cog[0], f"uit de koers over de grond ({n} metingen, spreiding {cog[1]:.1f}°)"))
    if kandidaten:
        onzeker, b0, uitleg = min(kandidaten)
        if onzeker <= NOORD_MAX_ONZEKER:
            return b0, max(onzeker, 0.5), uitleg
    if volger.fixes == 0:
        raise SystemExit("Het noorden is niet te bepalen: het robot geeft geen GPS-positie (geen GPS, of geen fix). "
                         "Geef --richting op.")
    raise SystemExit(f"Het noorden is niet te bepalen na {meter:g} m rijden: "
                     + (f"het GPS-spoor past maar binnen {res['rms']:.1f} m (onzekerheid ± {res['onzeker']:.0f}°)" if res else "te weinig GPS-posities")
                     + f", en er zijn {len(volger.peiling.monsters)} bruikbare koersen over de grond (nodig: 15 boven 1 m/s, weinig spreiding). "
                     "Rijd verder (--kalibreer-noord groter) of sneller (--noord-power), buiten onder open hemel, of geef --richting op.")


def vloeiend_pad(punten, stap_m=0.02):
    """Een lijn door de punten (x, y) als pad voor Jaguar.volg: de hoeken afgerond (twee keer Chaikin: elk hoekpunt
    wordt vervangen door twee punten op 1/4 en 3/4 van de stukken; begin en einde blijven) en een punt om de stap_m."""
    for _ in range(2):
        rond = [punten[0]]
        for a, b in zip(punten, punten[1:]):
            rond.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            rond.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        rond.append(punten[-1])
        punten = rond
    pad = [punten[0]]
    for a, b in zip(punten, punten[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, round(d / stap_m))
        pad += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return pad


def rijd_route(robot, punten, richting=None, kalibreer_noord=None, vanaf_hier=False, gps="auto",
               power=250, turn_power=380, pause=1.0, ramp=1.5, creep=60, tolerance=10.0, rest=3.0,
               gps_wacht=10.0, noord_power=NOORD_POWER, vloeiend=False):
    """De route rijden met een verbonden robot (vóór prepare). Geeft een rapport (dict) terug; zie afdrukken()."""
    lokaal = Lokaal(*punten[0])
    volger = GpsVolger(robot, lokaal)
    prepare(robot, rest)
    if vanaf_hier:
        robot.phase = "gps-start"
        robot.hold(max(0.0, gps_wacht - rest))
        for _ in range(10):                      # tot 5 s extra wachten op genoeg posities
            if len(volger.recent) >= START_MIN_FIXES:
                break
            robot.hold(0.5)
        meting = volger.gemiddelde()
        if meting is None or len(volger.recent) < START_MIN_FIXES:
            raise SystemExit("--vanaf-hier: het robot geeft geen GPS-positie (geen GPS, of geen fix). "
                             "Geef het startpunt zelf op als eerste coördinaat.")
        start, spreiding = meting
        if spreiding > START_MAX_SPREIDING:
            raise SystemExit(f"--vanaf-hier: de GPS springt {spreiding:.0f} m terwijl het robot stilstaat (binnen, of tussen gebouwen): "
                             "dat is geen startpunt. Zet het robot onder open hemel, of geef het startpunt zelf op als eerste coördinaat.")
        doelen = [lokaal.naar_xy(*p) for p in punten]
        print("Start volgens de GPS: {:.7f}, {:.7f}".format(*lokaal.naar_ll(*start))
              + f" (gemiddelde van {len(volger.recent)} posities, spreiding {spreiding:.1f} m"
              + ("; de GPS kan daar nog enkele meters naast zitten)" if spreiding > GPS_NAUWKEURIG else ")"))
    else:
        start, doelen = (0.0, 0.0), [lokaal.naar_xy(*p) for p in punten[1:]]
    if richting is not None:
        b0, sigma, noord = richting, RICHTING_ONZEKER, "opgegeven"
    else:
        b0, sigma, noord = noord_zoeken(robot, volger, kalibreer_noord, max(power, noord_power), ramp, creep, tolerance)
        print(f"Noord: het robot keek bij de start naar kompaskoers {b0:.1f}° ± {sigma:.1f}° ({noord})")

    route = [lokaal.naar_ll(*start)] + [lokaal.naar_ll(*q) for q in doelen]
    stappen, lengtes = route_naar_stappen(route, b0, lokaal)
    print(f"Route: {len(lengtes)} rechte stukken · {sum(lengtes):.2f} m" + (f"\n{beschrijf(stappen)}" if richting is not None else ""))
    gebruik = {"wielen": 0, "noorden": 0, "positie": 0}

    def doel(i):
        """Punt i in het stelsel van de odometrie, met wat nu het beste bekend is (zie de uitleg bovenaan)."""
        q = doelen[i]
        gereden = robot.odo.afstand
        res, uit, _ = volger.beste() if gps != "uit" else (None, None, 0.0)
        past = res is not None and res["basis"] >= FIT_MIN_BASIS and res["onzeker"] <= FIT_MAX_ONZEKER
        b, s, schaal, niveau = b0, sigma, 1.0, "wielen"
        if past and res["onzeker"] < sigma:      # 1. de vorm van het spoor: noorden en schaal
            b, s, niveau = res["b0"], res["onzeker"], "noorden"
            schaal = res["schaal"]
        x, y = enu_naar_robot(q[0] - start[0], q[1] - start[1], b)
        x, y = x / schaal, y / schaal
        fout = ODO_FOUT * gereden * (0.25 if schaal != 1.0 else 1.0) + gereden * math.sin(math.radians(s))
        if past:                                  # 2. de positie van de GPS, als die beter is dan de wielen
            if res["rms"] < GPS_NAUWKEURIG:      # nauwkeurige GPS (simulator zonder fout, RTK): ruis middelt uit
                fout_gps = uitgemiddeld(res)
            else:
                # Gewone GPS: de fout dwaalt in tientallen seconden (gemeten in de websimulator: 1 m in 20 s, en
                # het robot reed twee keer een stukje bij achter die dwaling aan). Ze middelt dus niet uit over een
                # rit, en is ook niet dezelfde als bij de start. Zonder bekend startpunt komt daar de vaste fout bij.
                fout_gps = res["rms"] + (0.0 if vanaf_hier else GPS_VASTE_FOUT)
            if gps == "aan" or fout_gps < fout:
                (x, y), fout, niveau = uit.naar_odo(q[0], q[1], res), fout_gps, "positie"
                if res["rms"] >= GPS_NAUWKEURIG:
                    fout = max(fout, 0.5)         # Dr Robot rekent een punt met deze GPS binnen 1 m als bereikt
        gebruik[niveau] += 1
        return x, y, max(0.2, 2 * fout)           # bijsturen heeft geen zin binnen de eigen onzekerheid

    if vloeiend:
        # De hele route als één pad, zonder stoppen: voor punten die dicht op elkaar liggen langs een weg of een
        # getekende lijn. Het pad ligt vast bij het vertrek (start en noorden); de GPS kijkt alleen mee.
        pad = vloeiend_pad([(robot.odo.x, robot.odo.y)] + [enu_naar_robot(q[0] - start[0], q[1] - start[1], b0) for q in doelen])
        robot.phase = "1-pad"
        robot.note(f"PLAN {robot.phase} {sum(lengtes):.4f}")
        robot.met_pauze(lambda: robot.volg(pad, power, ramp))
        if robot.vast:
            raise Vastgelopen(f"vastgelopen op het pad ({robot.vast}): de route stopt hier")
        robot.phase = "uitbollen"
        robot.hold(2.0)
    else:
        rijd_punten(robot, len(doelen), doel, power, turn_power, pause, ramp, creep, tolerance)
    robot.gps_hook = None

    # --- rapport: alles in (oost, noord) rond het eerste punt
    doel_enu = doelen[-1]
    oe, on = robot_naar_enu(robot.odo.x, robot.odo.y, b0)
    wielen = (start[0] + oe, start[1] + on)
    r = {"b0": b0, "noord": noord, "doel": lokaal.naar_ll(*doel_enu), "lengte": sum(lengtes), "gereden": robot.odo.afstand,
         "wielen": lokaal.naar_ll(*wielen), "wielen_fout": math.hypot(wielen[0] - doel_enu[0], wielen[1] - doel_enu[1]),
         "gps": None, "gps_fout": None, "gps_zinnen": volger.zinnen, "gps_fixes": volger.fixes, "passing": None,
         "gps_modus": gps, "gebruik": gebruik, "vertraging": 0.0}
    nu = volger.gemiddelde(10)                   # het robot staat al 2 s stil: gemiddelde van de laatste 2 s
    if nu is not None:
        r["gps"], r["gps_fout"] = lokaal.naar_ll(*nu[0]), math.hypot(nu[0][0] - doel_enu[0], nu[0][1] - doel_enu[1])
    res, uit, vertraging = volger.beste()
    if res is not None:
        vol = uit.resultaat()                    # met de schaal, ook als die niet bruikbaar is (voor het rapport)
        r["passing"] = dict(vol, schaal_bruikbaar=Uitlijning.schaal_bruikbaar(vol))
        r["vertraging"] = vertraging
    return r


def afdrukken(r):
    print(f"\nDoel (laatste punt):     {r['doel'][0]:.7f}, {r['doel'][1]:.7f}   route {r['lengte']:.2f} m, gereden {r['gereden']:.2f} m")
    print(f"Volgens wielen en gyro:  {r['wielen'][0]:.7f}, {r['wielen'][1]:.7f}   {r['wielen_fout'] * 100:.0f} cm van het doel"
          + (" (met het noorden van de start; onderweg is bijgestuurd met de GPS)" if r["gebruik"]["noorden"] + r["gebruik"]["positie"] else ""))
    if r["gps"] is None:
        print("Volgens de GPS:          geen positie (geen GPS of geen fix): alleen de wielen en de gyro tellen")
        return
    print(f"Volgens de GPS:          {r['gps'][0]:.7f}, {r['gps'][1]:.7f}   {r['gps_fout']:.2f} m van het doel "
          f"(gemiddelde van de laatste 2 s)")
    p = r["passing"]
    if p is None:
        return
    verschil = hoek180(p["b0"] - r["b0"])
    oordeel = ("nauwkeurige GPS" if p["rms"] <= GPS_NAUWKEURIG else "rustige GPS" if p["rms"] <= 1.0 else
               "bruikbaar voor een ruwe positie" if p["rms"] <= 3.0 else "onrustige GPS: niet op vertrouwen")
    print(f"GPS-spoor op het gereden spoor: past binnen {p['rms']:.2f} m ({oordeel}; {p['n']} posities"
          + (f"; de GPS loopt {r['vertraging']:.1f} s achter" if r["vertraging"] else "") + ")")
    print(f"  noorden volgens het spoor: {p['b0']:.1f}° (bij de start: {r['b0']:.1f}°, verschil {verschil:+.1f}°, onzekerheid ± {p['onzeker']:.1f}°)")
    if p["schaal_bruikbaar"]:
        print(f"  schaal: 1 m volgens de wielen is {p['schaal']:.3f} m volgens de GPS"
              + ("" if abs(p["schaal"] - 1) < 0.02 else
                 f" → het robot rijdt {abs(p['schaal'] - 1) * 100:.0f} % {'minder' if p['schaal'] < 1 else 'meer'} ver dan de wielen tellen "
                 "(slip of wielstraal): kalibreer met  python -m jaguar.kalibratie.gps_afstand  of met de rolmeter"))
    else:
        print("  schaal: het spoor is te kort tegenover de GPS-fout om de afstand te controleren")
    g = r["gebruik"]
    print(f"  gebruikt per punt: {g['wielen']}× alleen wielen en gyro, {g['noorden']}× het noorden/de schaal uit het spoor, "
          f"{g['positie']}× de positie van de GPS" + (" (--gps uit)" if r["gps_modus"] == "uit" else ""))


def main():
    ap = argparse.ArgumentParser(description="Naar GPS-coördinaten rijden (één punt of een route)")
    ap.add_argument("punten", nargs="*", type=coordinaat, help="breedte,lengte; het eerste is het startpunt (behalve met --vanaf-hier)")
    ap.add_argument("--vanaf-hier", action="store_true", help="de start is waar het robot nu staat volgens zijn GPS; alle punten zijn doelen")
    ap.add_argument("--richting", type=float, help="kompaskoers van het robot bij de start (0 = noord)")
    ap.add_argument("--kalibreer-noord", type=float, metavar="METER", help="eerst zoveel meter rechtdoor om het noorden te vinden (standaard 6 zonder --richting)")
    ap.add_argument("--noord-power", type=int, default=NOORD_POWER, help=f"vermogen voor dat stuk (standaard {NOORD_POWER}: boven 1 m/s)")
    ap.add_argument("--gps", choices=("auto", "aan", "uit"), default="auto", help="de GPS onderweg gebruiken: auto = alleen waar hij beter is dan de wielen")
    ap.add_argument("--gps-wacht", type=float, default=10.0, help="--vanaf-hier: zoveel seconden de GPS middelen in rust (standaard 10)")
    ap.add_argument("--vloeiend", action="store_true",
                    help="de hele route als één pad rijden, zonder te stoppen bij de punten (punten dicht op elkaar langs een weg)")
    ap.add_argument("--punten-bestand", help="tekstbestand met op elke regel breedte,lengte (zoals Dr Robot: lat,lon,0), in plaats van de punten op de opdrachtregel")
    ap.add_argument("--droog", action="store_true", help="alleen het plan tonen")
    ap.add_argument("--power", type=int, default=250)
    ap.add_argument("--turn-power", type=int, default=380)
    add_drive_args(ap, distance=1.0)
    ap.add_argument("--pause", type=positive_seconds, default=1.0)
    ap.add_argument("--rest", type=positive_seconds, default=3.0)
    ap.add_argument("--csv", action=argparse.BooleanOptionalAction, default=True)
    args = ap.parse_args()
    if args.punten_bestand:
        with open(args.punten_bestand, encoding="utf-8") as f:
            args.punten += [coordinaat(",".join(regel.replace(";", ",").split(",")[:2])) for regel in f if regel.strip() and not regel.startswith("#")]
    if len(args.punten) < (1 if args.vanaf_hier else 2):
        ap.error("een route heeft minstens twee punten (of één doel met --vanaf-hier)")
    if args.droog and (args.richting is None or args.vanaf_hier):
        ap.error("--droog heeft --richting nodig en kan niet met --vanaf-hier (de start is dan nog niet bekend)")
    if args.richting is None and args.kalibreer_noord is None:
        args.kalibreer_noord = 6.0
    if args.kalibreer_noord is not None and not 2 <= args.kalibreer_noord <= 50:
        ap.error("--kalibreer-noord: tussen 2 en 50 m")
    if not 0 < args.power <= 1000 or not 0 < args.noord_power <= 1000:
        ap.error("--power en --noord-power moeten tussen 1 en 1000 liggen")

    if args.droog:
        stappen, lengtes = route_naar_stappen(args.punten, args.richting)
        print(f"Startkoers {args.richting:.1f}° · {len(lengtes)} rechte stukken · {sum(lengtes):.2f} m")
        print(beschrijf(stappen))
        return
    if args.richting is None:
        print(f"Geen --richting: het robot rijdt eerst {args.kalibreer_noord:g} m RECHTDOOR (vermogen {max(args.power, args.noord_power)}) "
              "om het noorden te vinden. Zorg dat die weg vrij is.")

    with robot_session(args.csv, "_gps_route", args.ip, args.port, args.live, args.mqtt,
                       args.mqtt_host, args.network) as robot:
        robot.precies = args.precies
        afdrukken(rijd_route(robot, args.punten, args.richting, args.kalibreer_noord, args.vanaf_hier, args.gps,
                             args.power, args.turn_power, args.pause, args.ramp, args.creep, args.tolerance, args.rest,
                             args.gps_wacht, args.noord_power, args.vloeiend))


if __name__ == "__main__":
    main()
