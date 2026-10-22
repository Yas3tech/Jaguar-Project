"""Maakt dashboards/jaguar.json (Grafana leest het binnen ~10 s opnieuw in).

Gebruik:  python grafana/maak_dashboard.py
Panelen aanpassen gebeurt hier, niet in de JSON: zo blijven alle Flux-queries gelijk van vorm.

Indeling = het analysedashboard (dashboard_template.html), maar live:
  kerncijfers · Ritverkenner · Pad in 3D: hoogte en gebeurtenissen · Tracking · Niveau ·
  Trillingen en snelheid · Ruwe IMU en motorstatus
De 3D-weergaven van het analysedashboard bestaan niet in Grafana; op die plek staat de camera.
"""
import json
from pathlib import Path

OUT = Path(__file__).parent / "dashboards" / "jaguar.json"
# Camerabeeld (MJPEG) via een lokale doorgeefserver: de Axis-camera (192.168.0.65) vraagt een login,
# en die kan een browser niet in een <img> meesturen. Die server komt er later bij.
CAMERA_URL = "http://localhost:8766/camera.mjpg"

DS = {"type": "influxdb", "uid": "jaguar-influx"}
RIT = 'r.rit =~ /^${rit:regex}$/'
BLUE, ORANGE, GREEN, RED, GREY, YELLOW = "#2a78d6", "#eb6834", "#1f9e6a", "#d93b3b", "#8a9a94", "#fab219"
COL = [BLUE, ORANGE, GREEN]


DASH = {"type": "datasource", "uid": "-- Dashboard --"}
# Vier gedeelde query's i.p.v. één per paneel: 25 query's per seconde haalde de verversing niet
# (≈ 0,9 s). Panelen lezen het resultaat van een "bronpaneel" via de Dashboard-databron.
IMU_FIELDS = ["afstand", "afstand_totaal", "snelheid", "opdracht", "y_cm", "koers", "koers_enc", "pitch", "roll",
              "pitch_hoogte", "hoogte_cm", "trilling_z", "gx", "gy", "gz", "ax", "ay", "az"]


def wide_q(meas, fields, agg="mean"):
    """Eén brede tabel: tijd + één kolom per veld (zelfde tijdvensters voor alle velden)."""
    fs = ", ".join(f'"{f}"' for f in fields)
    return f'''from(bucket: "ritten")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r._measurement == "{meas}" and {RIT})
  |> filter(fn: (r) => contains(value: r._field, set: [{fs}]))
  |> group(columns: ["_field"])
  |> aggregateWindow(every: v.windowPeriod, fn: {agg}, createEmpty: false)
  |> group()
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> sort(columns: ["_time"])'''


XY_Q = f'''from(bucket: "ritten")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r._measurement == "imu" and {RIT})
  |> filter(fn: (r) => contains(value: r._field, set: ["x", "y_cm", "afstand_totaal", "hoogte_cm"]))
  |> group(columns: ["_field"])
  |> aggregateWindow(every: v.windowPeriod, fn: mean, createEmpty: false)
  |> group()
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> sort(columns: ["_time"])
  |> map(fn: (r) => ({{x: r.x, y: r.y_cm / 100.0, s: r.afstand_totaal, h: r.hoogte_cm}}))'''

SRC = {}   # naam gedeelde query -> id van het bronpaneel


def source(name, query):
    """Eerste paneel dat deze query gebruikt voert ze uit; volgende panelen lezen mee."""
    if name in SRC:
        return DASH, [{"refId": "A", "datasource": DASH, "panelId": SRC[name], "withTransforms": False}]
    SRC[name] = len(panels) + 1
    return DS, [{"refId": "A", "datasource": DS, "query": query}]


def only(*names):
    return [{"id": "filterFieldsByName", "options": {"include": {"pattern": "^(Time|_time|" + "|".join(names) + ")$"}}}]


panels = []
y_cursor = [0]


def add(p, x, y, w, h):
    p.update(id=len(panels) + 1, gridPos={"x": x, "y": y, "w": w, "h": h})
    if p["type"] not in ("row", "text"):
        p.setdefault("datasource", DS)
    panels.append(p)
    return p


def row(title):
    add({"type": "row", "title": title, "collapsed": False, "panels": []}, 0, y_cursor[0], 24, 1)
    y_cursor[0] += 1


def stat(title, src, field, unit, x, y, w=4, h=3, decimals=2, desc="", mappings=None, steps=None, calc="lastNotNull"):
    """Tegel = laatste (of max) waarde van één veld uit een gedeelde query."""
    ds, targets = source(*src)
    fc = {"unit": unit, "decimals": decimals, "color": {"mode": "thresholds"},
          "thresholds": {"mode": "absolute", "steps": steps or [{"color": "text", "value": None}]}}
    if mappings:
        fc["mappings"] = mappings
    add({"type": "stat", "title": title, "description": desc, "datasource": ds, "targets": targets,
         "options": {"reduceOptions": {"calcs": [calc], "fields": f"/^{field}$/", "values": False},
                     "textMode": "value", "colorMode": "value", "graphMode": "none", "justifyMode": "center"},
         "fieldConfig": {"defaults": fc, "overrides": []}}, x, y, w, h)


def ts(title, src, fields, unit, x, y, w=12, h=8, names=None, right=None, minspan=None, desc="", colors=None):
    ds, targets = source(*src)
    overrides = []
    for i, f in enumerate(fields):
        props = [{"id": "displayName", "value": (names or fields)[i]},
                 {"id": "color", "value": {"mode": "fixed", "fixedColor": (colors or COL)[i]}}]
        if right and f in right:
            props += [{"id": "custom.axisPlacement", "value": "right"}, {"id": "unit", "value": right[f]}]
        overrides.append({"matcher": {"id": "byName", "options": f}, "properties": props})
    custom = {"drawStyle": "line", "lineWidth": 2, "fillOpacity": 0, "showPoints": "never", "spanNulls": False,
              "insertNulls": 2000}   # lijn onderbreken bij > 2 s zonder data (tussen ritten)
    if minspan:
        custom.update(axisSoftMin=-minspan, axisSoftMax=minspan)
    add({"type": "timeseries", "title": title, "description": desc, "datasource": ds, "targets": targets,
         "transformations": only(*fields),
         "options": {"legend": {"displayMode": "list", "placement": "bottom"}, "tooltip": {"mode": "multi"}},
         "fieldConfig": {"defaults": {"unit": unit, "custom": custom}, "overrides": overrides}}, x, y, w, h)


def xy(title, xf, yf, xunit, yunit, x, y, w, h, desc="", soft=None):
    """xy-grafiek met vaste assen (automatisch wisselde Grafana x en y soms om)."""
    ds, targets = source("xy", XY_Q)
    custom = {"show": "lines", "lineWidth": 2, "pointSize": {"fixed": 3}}
    y_props = [{"id": "custom.axisSoftMin", "value": -soft}, {"id": "custom.axisSoftMax", "value": soft}] if soft else []
    # pluginVersion ≥ 11.1, anders "migreert" Grafana deze instellingen naar het oude formaat (Err).
    add({"type": "xychart", "pluginVersion": "11.4.0", "title": title, "description": desc, "datasource": ds, "targets": targets,
         "options": {"mapping": "manual",
                     "series": [{"frame": {"matcher": {"id": "byIndex", "options": 0}},
                                 "x": {"matcher": {"id": "byName", "options": xf}},
                                 "y": {"matcher": {"id": "byName", "options": yf}}}],
                     "legend": {"showLegend": False}, "tooltip": {"mode": "single"}},
         "fieldConfig": {"defaults": {"unit": yunit, "color": {"mode": "fixed", "fixedColor": BLUE}, "custom": custom},
                         "overrides": [{"matcher": {"id": "byName", "options": xf},
                                        "properties": [{"id": "unit", "value": xunit}]},
                                       {"matcher": {"id": "byName", "options": yf}, "properties": y_props}]}}, x, y, w, h)


def timeline(title, field, colors, x, y, w, h, desc=""):
    ds, targets = source("tekst", wide_q("imu", ["fase", "gebeurtenis"], agg="last"))
    add({"type": "state-timeline", "title": title, "description": desc, "datasource": ds, "targets": targets,
         "transformations": only(field),
         "options": {"showValue": "auto", "mergeValues": True, "rowHeight": 0.8, "legend": {"showLegend": False}},
         "fieldConfig": {"defaults": {"color": {"mode": "fixed", "fixedColor": GREY},
                                      "custom": {"insertNulls": 2000, "fillOpacity": 80, "lineWidth": 0},
                                      "mappings": [{"type": "value", "options": {
                                          k: (c if isinstance(c, dict) else {"color": c}) for k, c in colors.items()}}]},
                         "overrides": [{"matcher": {"id": "byName", "options": field},
                                        "properties": [{"id": "displayName", "value": title.lower()}]}]}}, x, y, w, h)


IMU = ("imu", wide_q("imu", IMU_FIELDS))
STATUS = ("status", wide_q("status", ["batterij_v", "stroom_max_a", "temp_max_c", "estop"], agg="last"))


# --- Kerncijfers (zelfde tegels als het analysedashboard, live) ------------------------------------
tiles = [
    ("Verplaatsing", "afstand", "lengthm", 2, "getekende encoderafstand; terugrijden trekt af", "lastNotNull"),
    ("Totale afstand", "afstand_totaal", "lengthm", 2, "heen en terug opgeteld", "lastNotNull"),
    ("Snelheid", "snelheid", "velocityms", 2, "nu, encoders over ~0,6 s", "lastNotNull"),
    ("Hoogste snelheid", "snelheid", "velocityms", 2, "in de gekozen periode", "max"),
    ("Zijdelings", "y_cm", "suffix: cm", 1, "afstand tot de startlijn; + = links", "lastNotNull"),
    ("Koersdrift", "koers", "degree", 1, "gyro (Z) t.o.v. de start", "lastNotNull"),
    ("Koers encoders", "koers_enc", "degree", 1, "(rechts − links) / spoorbreedte", "lastNotNull"),
    ("Langshelling", "pitch", "degree", 1, "pitch nu, accelerometer gefilterd", "lastNotNull"),
    ("Dwarshelling", "roll", "degree", 1, "roll nu, accelerometer gefilterd", "lastNotNull"),
    ("Hoogte", "hoogte_cm", "suffix: cm", 1, "t.o.v. het startvlak", "lastNotNull"),
]
for i, (title, field, unit, dec, desc, calc) in enumerate(tiles):
    stat(title, IMU, field, unit, 4 * (i % 6), 3 * (i // 6), decimals=dec, desc=desc, calc=calc)
stat("Batterij", STATUS, "batterij_v", "volt", 16, 3, decimals=1, desc="stop onder 22,2 V (6S LiPo)",
     steps=[{"color": "red", "value": None}, {"color": "text", "value": 22.2}])
stat("E-Stop", STATUS, "estop", "none", 20, 3, decimals=0,
     mappings=[{"type": "value", "options": {"0": {"text": "vrij", "color": "green"},
                                             "1": {"text": "actief", "color": "orange"}}}])
y_cursor[0] = 6

# --- Ritverkenner ----------------------------------------------------------------------------------
row("Ritverkenner")
y = y_cursor[0]
timeline("Fase", "fase", {"rust": GREY, "vrijgeven": YELLOW, "rijden": BLUE, "heen": BLUE,
                          "pauze": GREY, "terug": ORANGE, "uitbollen": GREEN}, 0, y, 24, 3)
y += 3
CAMERA_HTML = f"""<div style="height:100%;display:flex;align-items:center;justify-content:center;background:#0b0c0e;border-radius:4px">
<img src="{CAMERA_URL}" alt="" style="max-width:100%;max-height:100%;object-fit:contain"
 onerror="this.style.display='none';this.nextElementSibling.style.display='block'">
<div style="display:none;text-align:center;color:#8e9192;line-height:1.6">
<div style="font-size:42px">&#128247;</div>
<div style="font-size:15px">Camera nog niet gekoppeld</div>
<div style="font-size:12px">beeld van de robotcamera (192.168.0.65) komt hier</div></div></div>"""
add({"type": "text", "title": "Camera", "description": "Op deze plek staat in het analysedashboard de 3D-robotstand.",
     "options": {"mode": "html", "content": CAMERA_HTML}}, 0, y, 8, 10)
xy("Geschatte baan · bovenaanzicht", "x", "y", "lengthm", "lengthm", 8, y, 8, 10,
   desc="Encoderafstand + gyrokoers. y = zijdelings, positief = links. Assen niet op gelijke schaal.", soft=0.25)
ts("Motoropdracht en gemeten snelheid", IMU, ["snelheid", "opdracht"], "velocityms", 16, y, w=8, h=10,
   names=["snelheid", "opdracht (van ±1000)"], right={"opdracht": "none"},
   desc="Motoropdracht is geen snelheid. Negatief = achteruit.")
y_cursor[0] = y + 10

# --- Pad in 3D: hoogte en gebeurtenissen ------------------------------------------------------------
row("Pad in 3D: hoogte en gebeurtenissen")
y = y_cursor[0]
xy("Hoogteprofiel langs de afgelegde weg", "s", "h", "lengthm", "suffix: cm", 0, y, 14, 8,
   desc="Hoogte = som van sin(pitch) × afgelegde weg; pitch uit de gyro tijdens het rijden, "
        "uit de accelerometer in stilstand. Heen en terug liggen na elkaar op de as.", soft=2)
timeline("Gebeurtenissen", "gebeurtenis", {"geen": {"color": "transparent", "text": " "}, "vastgelopen": RED, "schuin": YELLOW,
                                           "vrije val": "purple", "wielen vrij": ORANGE, "één kant": BLUE},
         14, y, 10, 8,
         desc="Vastgelopen: vermogen aan maar < 15 % van de normale snelheid (≥ 1 s). Schuin: pitch of roll "
              "> 7° t.o.v. rust (≥ 1 s). Vrije val: totale versnelling < 0,35 g (≥ 3 samples). Wielen vrij: "
              "> 160 % van de normale snelheid (≥ 0,6 s). Eén kant: links en rechts verschillen > 60 % (≥ 1 s).")
y_cursor[0] = y + 8

# --- Tracking ---------------------------------------------------------------------------------------
row("Tracking: rijdt de robot recht?")
y = y_cursor[0]
ts("Zijdelingse afwijking t.o.v. de startlijn", IMU, ["y_cm"], "suffix: cm", 0, y, names=["zijdelings"], minspan=5,
   desc="Encoderafstand gecombineerd met de gyrokoers. Positief = links.")
ts("Koersverandering", IMU, ["koers", "koers_enc"], "degree", 12, y, names=["gyro (Z)", "encoders (rechts − links)"],
   minspan=2, desc="Verschil tussen gyro en encoders = slip, kalibratie of meetfout.")
y_cursor[0] = y + 8

# --- Niveau -----------------------------------------------------------------------------------------
row("Niveau: helling en dwarshelling")
y = y_cursor[0]
ts("Pitch en roll", IMU, ["pitch", "roll", "pitch_hoogte"], "degree", 0, y, w=24, minspan=2,
   names=["pitch (neus omhoog +)", "roll (linkerkant hoger +)", "pitch voor de hoogte (gyro tijdens rijden)"],
   desc="Pitch en roll uit de zwaartekrachtrichting van de accelerometer (gefilterd ~0,2 s). Tijdens optrekken, "
        "remmen en trillen meet de accelerometer ook versnelling; daarom gebruikt de hoogte de gyro-pitch.")
y_cursor[0] = y + 8

# --- Trillingen en snelheid -------------------------------------------------------------------------
row("Trillingen en snelheid")
y = y_cursor[0]
ts("Verticale trilling", IMU, ["trilling_z"], "accG", 0, y, names=["trilling Z (RMS, 0,2 s)"],
   desc="RMS van de Z-versnelling min het lokale gemiddelde, per 0,2 s. In rust ca. 0,01 g.")
ts("Snelheid (encoders)", IMU, ["snelheid"], "velocityms", 12, y, names=["snelheid"])
y_cursor[0] = y + 8

# --- Ruwe IMU en motorstatus ------------------------------------------------------------------------
row("Ruwe IMU en motorstatus")
y = y_cursor[0]
ts("Gyro", IMU, ["gx", "gy", "gz"], "rotdegs", 0, y, names=["X", "Y", "Z"])
ts("Versnelling", IMU, ["ax", "ay", "az"], "accG", 12, y, names=["X", "Y", "Z"])
ts("Batterij en stroom", STATUS, ["batterij_v", "stroom_max_a", "temp_max_c"], "volt", 0, y + 8, w=24, h=6,
   names=["batterij", "stroom max", "temperatuur max"], right={"stroom_max_a": "amp", "temp_max_c": "celsius"})

dash = {
    "uid": "jaguar-live", "title": "Jaguar live", "tags": ["jaguar"], "timezone": "browser",
    "description": "Live versie van het analysedashboard (dashboard.html). Definitieve cijfers: analyse_rit.py op de CSV.",
    "editable": True, "liveNow": True, "refresh": "1s", "schemaVersion": 39,
    "time": {"from": "now-2m", "to": "now"},
    "timepicker": {"refresh_intervals": ["1s", "2s", "5s", "10s", "30s"]},
    "templating": {"list": [{
        "name": "rit", "label": "Rit", "type": "query", "datasource": DS,
        "query": 'import "influxdata/influxdb/schema"\nschema.tagValues(bucket: "ritten", tag: "rit", start: -365d)',
        "definition": "rit-tags", "refresh": 1, "sort": 2, "includeAll": True, "allValue": ".*", "multi": False,
        "current": {"text": "All", "value": "$__all"}}]},
    "panels": panels,
}
OUT.write_text(json.dumps(dash, indent=2, ensure_ascii=False), encoding="utf-8")
print(f"{len(panels)} panelen -> {OUT}")
