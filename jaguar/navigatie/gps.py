"""GPS: NMEA lezen, coördinaten naar meters, noordrichting uit de koers over de grond, voorzichtig anker.

Wat de GPS (Garmin GPS 18x-5Hz) kan en niet kan:
  - 5 posities per seconde, fout < 15 m (95 %), met WAAS < 3 m, alleen onder open hemel.
  - Binnen meldt de ontvanger toch "A" (fix) terwijl de positie tientallen meters dwaalt (gemeten: 50 m
    in 2 minuten met een stilstaand robot). Een fix zegt dus niets over de kwaliteit.
  - De controller van het robot stuurt alleen GPRMC door: positie, snelheid en koers over de grond (COG),
    zonder HDOP of aantal satellieten.
Daarom levert de GPS hier alleen wat de wielen en de IMU niet kunnen: coördinaten in de wereld en de
richting van het noorden. De precisie van de route komt van de encoders en de gyro (zie odometrie.py).

Overal: koers van het robot in graden, 0 = de startrichting, + = links (zoals odometrie.py). Kompaskoers
(bearing) in graden, 0 = noord, 90 = oost, met de klok mee.
"""
import math
from collections import deque

KT = 0.514444             # m/s per knoop
V_MIN_COG = 1.0           # m/s: onder deze snelheid is de koers over de grond onbetrouwbaar (gemeten: 28° ruis in 2 s)
GATE_M = 2.5              # m: verschil tussen GPS-verplaatsing en wielverplaatsing (over 6 tot 10 s) waarboven de GPS wordt genegeerd
DRIFT_MAX = 5.0           # m: zoveel mag het verschil GPS − odometrie in een minuut veranderen voor de GPS betrouwbaar heet
KP_ANKER = 0.1            # deel van het resterende verschil dat per geldige GPS-meting wordt bijgetrokken (Dr Robot: 0,5)


def checksum_ok(zin):
    """NMEA-controlesom ($...*hh); een zin zonder * telt als fout."""
    if not zin.startswith("$") or "*" not in zin:
        return False
    body, _, hh = zin[1:].partition("*")
    c = 0
    for ch in body:
        c ^= ord(ch)
    try:
        return c == int(hh[:2], 16)
    except ValueError:
        return False


def parse_rmc(zin):
    """$GPRMC -> dict(lat, lon, v (m/s), cog (graden of None), fix (bool)) of None bij een kapotte zin."""
    if not zin.startswith("$GPRMC") or not checksum_ok(zin):
        return None
    p = zin.split("*")[0].split(",")
    if len(p) < 10:
        return None
    try:
        fix = p[2] == "A"
        lat = int(p[3][:2]) + float(p[3][2:]) / 60
        lon = int(p[5][:3]) + float(p[5][3:]) / 60
        lat = -lat if p[4] == "S" else lat
        lon = -lon if p[6] == "W" else lon
        v = float(p[7]) * KT if p[7] else 0.0
        cog = float(p[8]) if p[8] else None
    except (ValueError, IndexError):
        return None
    return {"fix": fix, "lat": lat, "lon": lon, "v": v, "cog": cog}


def hoek180(a):
    """Hoek in graden naar (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


class Lokaal:
    """Vlak assenstelsel (oost, noord) in meters rond een referentiepunt (coördinaat 1 = nulpunt).
    Gewone WGS84-formules voor meters per graad; op een paar honderd meter is de fout < 1 cm."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        f = math.radians(lat0)
        self.m_lat = 111132.954 - 559.822 * math.cos(2 * f) + 1.175 * math.cos(4 * f)
        self.m_lon = 111412.84 * math.cos(f) - 93.5 * math.cos(3 * f) + 0.118 * math.cos(5 * f)

    def naar_xy(self, lat, lon):
        """(oost, noord) in meters ten opzichte van het referentiepunt."""
        return (lon - self.lon0) * self.m_lon, (lat - self.lat0) * self.m_lat

    def naar_ll(self, oost, noord):
        return self.lat0 + noord / self.m_lat, self.lon0 + oost / self.m_lon


def afstand_richting(van, naar):
    """Afstand (m) en kompaskoers (graden, 0 = noord) tussen twee (oost, noord)-punten."""
    de, dn = naar[0] - van[0], naar[1] - van[1]
    return math.hypot(de, dn), math.degrees(math.atan2(de, dn)) % 360.0


def robot_naar_enu(x, y, b0):
    """Positie (x vooruit, y links) in het robotstelsel van de start -> (oost, noord), als de startrichting
    kompaskoers b0 had."""
    s, c = math.sin(math.radians(b0)), math.cos(math.radians(b0))
    return x * s - y * c, x * c + y * s


class NoordPeiling:
    """Kompaskoers van de startrichting (b0) uit de koers over de grond (COG).
    Tijdens een rechte rit op koers θ (gyro) geldt COG = b0 − θ, dus b0 = COG + θ.
    Alleen monsters bij genoeg snelheid en een rustige gyro tellen mee; het resultaat is een cirkelgemiddelde
    en komt pas als er genoeg monsters zijn met een kleine spreiding."""

    def __init__(self, v_min=V_MIN_COG, max_draai=3.0, min_monsters=15, max_spreiding=5.0):
        self.v_min, self.max_draai, self.min_monsters, self.max_spreiding = v_min, max_draai, min_monsters, max_spreiding
        self.monsters = []

    def voeg_toe(self, gps, koers, draaisnelheid):
        """gps: dict van parse_rmc; koers/draaisnelheid van de odometrie (graden, graden/s)."""
        if gps and gps["fix"] and gps["cog"] is not None and gps["v"] >= self.v_min and abs(draaisnelheid) <= self.max_draai:
            self.monsters.append(math.radians(gps["cog"] + koers))

    def resultaat(self):
        """(b0 in graden, spreiding in graden) of None als er te weinig of te wisselende monsters zijn."""
        if len(self.monsters) < self.min_monsters:
            return None
        s = sum(math.sin(a) for a in self.monsters) / len(self.monsters)
        c = sum(math.cos(a) for a in self.monsters) / len(self.monsters)
        r = math.hypot(s, c)
        spreiding = math.degrees(math.sqrt(max(0.0, -2 * math.log(max(r, 1e-9)))))
        if spreiding > self.max_spreiding:
            return None
        return math.degrees(math.atan2(s, c)) % 360.0, spreiding


class GpsAnker:
    """Houdt bij hoe de odometrie (nauwkeurig maar langzaam driftend) zich verhoudt tot de GPS (ruw maar
    zonder drift). Absolute positie = odometrie in het wereldstelsel + `verschuiving`; die verschuiving
    wordt bijgetrokken met een klein deel van het GPS-verschil.

    Een GPS-meting telt alleen mee als de GPS-verplaatsing over de laatste 6 tot 10 seconden past bij die van
    de wielen (verschil < GATE_M, elk gemiddeld over 5 metingen). Een controle met alleen de vorige meting
    helpt niet: de GPS dwaalt langzaam, in stapjes van centimeters (gemeten binnen: 20 tot 30 m in twee
    minuten bij een stilstaand robot, terwijl de receiver "A" meldt). Over zo'n basis is dat verschil groot
    en wordt de meting geweigerd. Elk geweigerd monster verhoogt `geweigerd`."""

    VENSTER = 50              # metingen (10 s bij 5 Hz)
    MIN_VENSTER = 30          # zoveel geschiedenis nodig voor een vergelijking
    GEMIDDELD = 5

    def __init__(self, lokaal, b0, kp=KP_ANKER, gate=GATE_M):
        self.lokaal, self.b0, self.kp, self.gate = lokaal, b0, kp, gate
        self.verschuiving = None         # (oost, noord)
        self.venster = deque(maxlen=self.VENSTER)    # (gps_oost, gps_noord, odo_oost, odo_noord)
        self.geaccepteerd = 0
        self.geweigerd = 0
        self.historie = deque(maxlen=300)            # verschuiving na elke geaccepteerde meting (1 minuut)

    @staticmethod
    def _gem(punten):
        n = len(punten)
        return tuple(sum(p[k] for p in punten) / n for k in range(4))

    def meting(self, gps, odo_x, odo_y):
        """Eén GPS-zin (dict) met de odometriepositie op hetzelfde moment. Geeft True als hij is gebruikt."""
        if not gps or not gps["fix"]:
            return False
        ge, gn = self.lokaal.naar_xy(gps["lat"], gps["lon"])
        oe, on = robot_naar_enu(odo_x, odo_y, self.b0)
        self.venster.append((ge, gn, oe, on))
        if len(self.venster) < self.MIN_VENSTER:
            return False                 # nog geen basis om mee te vergelijken
        oud = self._gem(list(self.venster)[:self.GEMIDDELD])
        nieuw = self._gem(list(self.venster)[-self.GEMIDDELD:])
        verschil = math.hypot((nieuw[0] - oud[0]) - (nieuw[2] - oud[2]), (nieuw[1] - oud[1]) - (nieuw[3] - oud[3]))
        if verschil > self.gate:
            self.geweigerd += 1
            return False
        meting = (nieuw[0] - nieuw[2], nieuw[1] - nieuw[3])
        if self.verschuiving is None:
            self.verschuiving = meting
        else:
            self.verschuiving = (self.verschuiving[0] + self.kp * (meting[0] - self.verschuiving[0]),
                                 self.verschuiving[1] + self.kp * (meting[1] - self.verschuiving[1]))
        self.geaccepteerd += 1
        self.historie.append(self.verschuiving)
        return True

    def positie(self, odo_x, odo_y):
        """Geschatte absolute (oost, noord); None zolang er geen geldige GPS-meting is geweest."""
        if self.verschuiving is None:
            return None
        oe, on = robot_naar_enu(odo_x, odo_y, self.b0)
        return oe + self.verschuiving[0], on + self.verschuiving[1]

    @property
    def drift(self):
        """Hoe ver (m) het verschil GPS − odometrie de laatste minuut is weggelopen. Klopt de GPS, dan blijft dat
        verschil vast; dwaalt hij (binnen, slechte satellieten), dan loopt het weg."""
        if len(self.historie) < 2:
            return 0.0
        return math.hypot(max(h[0] for h in self.historie) - min(h[0] for h in self.historie),
                          max(h[1] for h in self.historie) - min(h[1] for h in self.historie))

    @property
    def betrouwbaar(self):
        """Genoeg geaccepteerde metingen (minstens 20 en 60 % van alle beoordeelde) en een verschil GPS − odometrie
        dat de laatste minuut niet meer dan DRIFT_MAX is weggelopen."""
        totaal = self.geaccepteerd + self.geweigerd
        return self.geaccepteerd >= 20 and self.geaccepteerd >= 0.6 * totaal and self.drift <= DRIFT_MAX
