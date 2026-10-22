"""Obstakels voor de robot zien met de camera, om op tijd te stoppen (niet om afstand te meten: dat doen de wielen).

Idee: de camera staat vast op de robot, dus elke beeldrij hoort bij een vaste afstand op de vloer.
In de middelste strook van het beeld (de weg van de robot) zoeken we van onder naar boven waar
de vloer ophoudt: de voet van een muur, deur, stoel, been … Dat is het dichtste obstakel.
Elke rij wordt vergeleken met de vloer net eronder: een plotse kleursprong is de rand van de vloer
(de vloer zelf wordt met de afstand hoogstens geleidelijk lichter of donkerder). Grenzen: een muur in dezelfde kleur als de vloer, glas, een vloer met een sterk
patroon of een rand (tapijt) en donker. Alleen vooruit: achteruit is de robot blind.

Rij → afstand: d = k / (y − y_horizon), met y de rij (0 = boven, 1 = onder). k en y_horizon komen uit
de kalibratie (camera_kalibratie.json): minstens twee keer de voet van een muur aanklikken op een
gekende afstand (Opties → Obstakels op de live pagina).
"""
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

KALIBRATIE = Path(__file__).parent / "camera_kalibratie.json"
RESOLUTIE, FPS = "320x240", 10   # klein beeld: weinig extra wifi naast de robotgegevens
BAND = (0.38, 0.62)       # middelste strook: de weg van de robot (≈ 0,6-0,8 m breed op 1-2 m)
START = 0.97              # onderste rij die we bekijken (daaronder kan de robot zelf in beeld komen)
BELOW = (3, 11)           # vergelijk elke rij met de vloer 3 … 10 rijen eronder (geleidelijk lichter worden mag)
JUMP = 15.0               # Lab-kleursprong t.o.v. die vloer = rand (vloer zelf: ≤ 7 per 5 rijen, muurvoet ≈ 30)
PERCENTIEL = 85           # van de kolommen telt de dichtste rand, op enkele ruisende kolommen na
MAX_BEREIK = 6.0          # m: verder is de rij te dicht bij de horizon om iets te betekenen
LAAGSTE_RIJ = 0.95        # onderste rij waar edge_columns een rand kan vinden (START en BELOW)
BLIND_MARGE = 0.05        # m: stoppen minstens zo ver vóór de dode zone onder de camera
# Ter plaatse draaien: de hoeken (61,5 × 57,3 cm, handleiding) beschrijven een cirkel van 42 cm rond het
# midden, 11 cm voorbij de voorkant. De stopafstand (standaard 0,4 m) is die 11 cm plus marge voor de
# camerafout dichtbij, de vertraging en het niet perfect rond het midden draaien van skid-steer.
VAKKEN = 32               # vloerrand over de hele breedte in zoveel vakken (voor de pagina en de omweg)
ZIJ_LINKS, ZIJ_RECHTS = (0.0, BAND[0]), (BAND[1], 1.0)   # beelddelen naast de weg van de robot


def edge_columns(img):
    """Per kolom de rij (0 = boven, 1 = onder) waar de vloer ophoudt; 0 = vloer tot bovenaan.
    Van onder naar boven: de eerste rij die plots verschilt van de vloer net eronder.
    Zo mag de vloer met de afstand geleidelijk lichter of donkerder worden (reflectie)."""
    h, w = img.shape[:2]
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (7, 7), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    # gemiddelde van de rijen r+3 … r+10 per kolom, via cumulatieve som
    a, b = BELOW
    cs = np.concatenate([np.zeros((1, w, 3), np.float32), np.cumsum(lab, 0)])
    n = h - b
    below = (cs[b:b + n] - cs[a:a + n]) / (b - a)
    jump = np.sqrt(((lab[:n] - below) ** 2).sum(2)) > JUMP           # (n, w)
    start = min(n, int(START * h))
    sub = jump[:start][::-1]                                          # van onder naar boven
    edge = np.where(sub.any(0), start - 1 - sub.argmax(0), 0)
    return edge / h


def part(edges, lo, hi, pct=PERCENTIEL):
    """Dichtste rand (rij) in het beelddeel lo … hi (fractie van de breedte), op ruisende kolommen na."""
    w = len(edges)
    return float(np.percentile(edges[int(lo * w):max(int(lo * w) + 1, int(hi * w))], pct))


def floor_edge(img):
    """Rij waar de vloer ophoudt in de middelste strook (de weg van de robot)."""
    return part(edge_columns(img), *BAND)


def profile(edges):
    """De vloerrand in VAKKEN vakken over de hele breedte (voor de pagina)."""
    return [round(part(edges, i / VAKKEN, (i + 1) / VAKKEN, 50), 4) for i in range(VAKKEN)]


def load_calibration():
    try:
        return json.loads(KALIBRATIE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"punten": []}


def fit(points):
    """y = y_h + k / d (rechte in 1/d) door de punten [rij, afstand_m]; None bij te weinig of onzinnige punten."""
    pts = [(float(y), float(d)) for y, d in points if 0 < float(y) <= 1 and 0.1 <= float(d) <= 20]
    if len(pts) < 2 or len({d for _, d in pts}) < 2:
        return None
    x = np.array([1 / d for _, d in pts])
    y = np.array([y for y, _ in pts])
    k, yh = np.polyfit(x, y, 1)
    if k <= 0 or not 0 <= yh < LAAGSTE_RIJ:
        return None
    # Dichter dan blind_m ziet de camera de vloer niet meer: daar kan hij geen muurvoet vinden.
    return {"k": float(k), "y_horizon": float(yh), "blind_m": round(float(k / (LAAGSTE_RIJ - yh)), 3)}


def save_point(row, dist):
    """Kalibratiepunt toevoegen (rij van de muurvoet, afstand in m); geeft de nieuwe kalibratie terug."""
    cal = load_calibration()
    cal["punten"] = [p for p in cal.get("punten", []) if abs(p[1] - dist) > 0.01] + [[round(row, 4), round(dist, 3)]]
    cal["model"] = fit(cal["punten"])
    KALIBRATIE.write_text(json.dumps(cal, indent=2), encoding="utf-8")
    return cal


def clear_calibration():
    KALIBRATIE.unlink(missing_ok=True)
    return {"punten": [], "model": None}


def distance(row, model):
    """Afstand (m) op de vloer tot beeldrij `row`; None = verder dan MAX_BEREIK of boven de horizon."""
    dy = row - model["y_horizon"]
    if dy <= 0:
        return None
    d = model["k"] / dy
    return d if d <= MAX_BEREIK else None


class Detector:
    """Leest de camera in een achtergronddraad en houdt de laatste meting bij: rij, afstand en tijdstip."""

    def __init__(self, stop_m=0.6):
        self.stop_m = stop_m
        self.model = load_calibration().get("model")
        self.row = None
        self.dist = None
        self.profile = None      # vloerrand per vak over de hele breedte
        self.sides = {}          # "links"/"rechts" -> vrije afstand (m) naast de weg; None = verder dan het bereik
        self.t = None            # time.monotonic() van de laatste meting
        self.error = None
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    @property
    def stop(self):
        """Stopafstand (m): de gekozen, maar nooit binnen de dode zone onder de camera."""
        blind = (self.model or {}).get("blind_m") or 0
        return max(self.stop_m, blind + BLIND_MARGE)

    def latest(self):
        """(afstand_m of None, leeftijd in s)."""
        return self.dist, (time.monotonic() - self.t) if self.t else float("inf")

    def info(self):
        """Voor de live pagina."""
        _, age = self.latest()
        return {"rij": self.row, "m": self.dist, "leeftijd": round(age, 2) if age < 1e6 else None,
                "profiel": self.profile, "band": BAND, "zijden": self.sides,
                "gekalibreerd": self.model is not None, "stop_m": self.stop, "blind_m": (self.model or {}).get("blind_m"),
                "fout": self.error}

    def free_side(self, beyond):
        """Kant om een obstakel heen: "links" of "rechts" waar de vloer verder doorloopt dan `beyond` m,
        de vrijste eerst; None = beide kanten even dicht (muur dwars over de weg)."""
        free = {k: (MAX_BEREIK if v is None else v) for k, v in self.sides.items()}
        ok = [k for k, v in sorted(free.items(), key=lambda kv: -kv[1]) if v > beyond]
        return ok[0] if ok else None

    def _loop(self):
        import live_view   # zelfde camera-login als de live pagina
        while self.running:
            try:
                resp = live_view.open_camera(RESOLUTIE, FPS)
            except OSError as e:
                self.error = f"camera niet bereikbaar: {e}"
                time.sleep(2)
                continue
            self.error = None
            buf = b""
            try:
                while self.running:
                    chunk = resp.read1(65536)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b"\xff\xd8")
                    b = buf.find(b"\xff\xd9", a + 2)
                    while a >= 0 and b >= 0:          # alle volledige JPEG's in de buffer
                        jpg, buf = buf[a:b + 2], buf[b + 2:]
                        self._frame(jpg)
                        a = buf.find(b"\xff\xd8")
                        b = buf.find(b"\xff\xd9", a + 2)
                    if len(buf) > 2_000_000:
                        buf = b""
            except OSError as e:
                self.error = f"camerabeeld onderbroken: {e}"
            finally:
                resp.close()

    def _frame(self, jpg):
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        edges = edge_columns(img)
        row = part(edges, *BAND)
        self.row = round(row, 4)
        self.profile = profile(edges)
        if self.model:
            self.dist = distance(row, self.model)
            self.sides = {"links": distance(part(edges, *ZIJ_LINKS, 50), self.model),
                          "rechts": distance(part(edges, *ZIJ_RECHTS, 50), self.model)}
        else:
            self.dist, self.sides = None, {}
        self.t = time.monotonic()

    def close(self):
        self.running = False


_detector = None


def get(stop_m=0.6):
    """Eén detector per proces (rijscript of bediening.py), gestart bij het eerste gebruik."""
    global _detector
    if _detector is None or not _detector.running:
        _detector = Detector(stop_m)
    _detector.stop_m = stop_m
    _detector.model = load_calibration().get("model")
    return _detector


def current():
    return _detector if _detector is not None and _detector.running else None
