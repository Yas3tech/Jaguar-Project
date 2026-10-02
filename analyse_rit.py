"""Analyse van een rit-log (fase1_rechtdoor_imu.py): rechtlijnigheid en niveau.

Gebruik:  python analyse_rit.py [ritten/rit_....csv]   (standaard de nieuwste rit)
Schrijft ritten/<rit>_analyse.json en één dashboard.html met een ritselector.

Aannames (Jaguar-handleiding + config OutDoorRobotConfig_4x4.xml):
  - ADXL345 full-res: 256 LSB/g  (in rust leest Z ~254)
  - ITG3205: 14.375 LSB/(deg/s)  (datasheet, nog niet geverifieerd met een draai)
  - IMU-assen: X vooruit (bevestigd: +0.1 g bij optrekken, -0.1 g bij remmen), Z omhoog, Y links
  - wiel: 300 encoderpulsen per omwenteling, straal 0.135 m, spoorbreedte 0.52 m
  - kanaal 1 = linkerwielen (vooruit = pulsen omhoog), kanaal 2 = rechterwielen (vooruit = pulsen omlaag)
  - MM0 = voorste motordriver, MM1 = achterste
  - hoogte = integraal van sin(pitch) over de encoderafstand, t.o.v. het vlak waarop de robot in rust stond
"""
import csv
import json
import math
import statistics as st
import sys
from pathlib import Path

ACC_LSB_G = 256.0
GYRO_LSB_DPS = 14.375
WHEEL_CNT = 300
WHEEL_R = 0.135           # nominaal; kalibreer.py schrijft de gemeten effectieve straal naar kalibratie.json
WHEEL_DIS = 0.52
CALIBRATION = Path(__file__).parent / "kalibratie.json"
if CALIBRATION.exists():
    WHEEL_R = json.loads(CALIBRATION.read_text(encoding="utf-8"))["wielstraal_m"]
M_PER_CNT = 2 * math.pi * WHEEL_R / WHEEL_CNT
SMOOTH_S = 0.5            # venster voor hellingshoeken (laagdoorlaat)
VIB_WINDOW_S = 0.2        # venster voor trillings-RMS
G = 9.81
REST_ANCHOR_S = 0.3       # stilstand die minstens zo lang duurt geeft een betrouwbare rusthoek (accelerometer)
FREEFALL_G = 0.35         # |a| hieronder = (bijna) vrije val
FREEFALL_MIN_S = 0.035    # ≥ 3 opeenvolgende samples bij 50 Hz
TILT_DEG = 7.0            # pitch/roll t.o.v. rust boven deze grens …
TILT_MIN_S = 1.0          # … minstens zo lang = robot hangt schuin (obstakel, helling)
STALL_FRAC = 0.15         # snelheid < 15 % van wat het vermogen normaal geeft …
STALL_MIN_S = 1.0         # … minstens zo lang = vastgelopen
SPIN_FRAC = 1.6           # snelheid > 160 % van normaal = wielen draaien vrij (opgetild / in de lucht)
SPIN_MIN_S = 0.6
SIDE_FRAC = 0.6           # links en rechts verschillen meer dan 60 % = één kant blokkeert of slipt
SIDE_MIN_S = 1.0

HERE = Path(__file__).parent
TEMPLATE = HERE / "dashboard_template.html"


def load(path):
    imu, enc, motor, notes = [], {"MM0": [], "MM1": []}, [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            t, fase, line = float(r["t"]), r["fase"], r["regel"]
            p = line.split(",")
            if line.split(" ", 1)[0] in ("DOEL", "STOP", "VAST"):   # eigen regels van het rijscript
                notes.append((t, line.split()))
            elif p[0].startswith("#") and len(p) == 17 and p[3] == "GYRO" and p[7] == "ACC":
                try:
                    imu.append({"t": t, "fase": fase, "pwr": int(r["vermogen"]), "yaw": float(p[2]),
                                "g": [int(v) for v in p[4:7]], "a": [int(v) for v in p[8:11]]})
                except ValueError:
                    pass                # beschadigd wifipakket (bv. "ACC,-6,,256"): sample overslaan
            elif line[:4] in ("MM0 ", "MM1 ") and "=" in line:
                board, kv = line.split(" ", 1)
                key, val = kv.split("=", 1)
                if not all(v.lstrip("-").isdigit() for v in val.split(":")):
                    continue            # beschadigd pakket
                if key == "C":
                    l, rr = (int(v) for v in val.split(":"))
                    enc[board].append((t, l, -rr))
                elif key in ("A", "T", "V"):
                    motor.append((t, fase, board, key, [int(v) for v in val.split(":")]))
    dejitter(imu, 0.02, get=lambda s: s["t"], put=lambda s, t: s.__setitem__("t", t))
    for board, rows in enc.items():
        gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if b[0] - a[0] > 0.1]
        cycle = st.median(gaps) if gaps else 0.2
        dejitter(rows, 0.9 * cycle, get=lambda r: r[0], put=None)
    return imu, enc, motor, notes


def dejitter(items, dt_min, get, put):
    """Wifi bundelt pakketten: ze komen tegelijk binnen terwijl ze na elkaar gemeten zijn.
    Achterwaarts: elk sample ligt minstens dt_min voor het volgende, maar nooit na zijn
    ontvangsttijd. Echte gaten (de robot stuurde niets) blijven zo behouden."""
    t_next = math.inf
    for i in range(len(items) - 1, -1, -1):
        t = min(get(items[i]), t_next - dt_min)
        if put:
            put(items[i], t)
        else:
            items[i] = (t,) + items[i][1:]
        t_next = t


def interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    f = (x - xs[lo]) / (xs[hi] - xs[lo]) if xs[hi] > xs[lo] else 0
    return ys[lo] + f * (ys[hi] - ys[lo])


def moving_avg(vals, n):
    half = n // 2
    out = []
    for i in range(len(vals)):
        w = vals[max(0, i - half): i + half + 1]
        out.append(sum(w) / len(w))
    return out


def rnd(v, d=3):
    return round(v, d)


def analyse(path):
    imu, enc, motor, notes = load(path)
    drive_phases = {"rijden", "heen", "terug"}
    if not imu or not any(s["fase"] in drive_phases for s in imu) or not any(s["fase"] == "rust" for s in imu) or any(len(v) < 2 for v in enc.values()):
        raise ValueError("Log mist IMU, rustfase, rijfase of encoders van beide drivers.")
    t_drive = next(s["t"] for s in imu if s["fase"] in drive_phases)
    rest = [s for s in imu if s["fase"] == "rust"]
    bias = [st.mean(s["g"][i] for s in rest) for i in range(3)]
    acc_rest = [st.mean(s["a"][i] for s in rest) for i in range(3)]

    # Encoders: afstand links/rechts (gemiddelde voor+achter), t.o.v. start van de rit
    def wheel_series(board):
        rows = enc[board]
        base = next((r for r in reversed(rows) if r[0] <= t_drive), rows[0])
        return [(r[0], (r[1] - base[1]) * M_PER_CNT, (r[2] - base[2]) * M_PER_CNT) for r in rows]

    front, rear = wheel_series("MM0"), wheel_series("MM1")
    et = [r[0] for r in front]
    left = [(a[1] + interp([q[0] for q in rear], [q[1] for q in rear], a[0])) / 2 for a in front]
    right = [(a[2] + interp([q[0] for q in rear], [q[2] for q in rear], a[0])) / 2 for a in front]
    dist = [(l + r) / 2 for l, r in zip(left, right)]

    # Snelheid uit encoders
    speed = []
    for i in range(1, len(et)):
        dt = et[i] - et[i - 1]
        if dt > 0.05:
            speed.append(((et[i] + et[i - 1]) / 2 - t_drive, (dist[i] - dist[i - 1]) / dt))
    v_max = max(abs(v) for _, v in speed)
    v_ref = st.median(abs(v) for _, v in speed if abs(v) > 0.05) if v_max > 0.05 else v_max
    p_max = max(abs(s["pwr"]) for s in imu)
    def sample_at(t):
        return min(imu, key=lambda s: abs(s["t"] - (t + t_drive)))
    steady = [(t, v) for t, v in speed if abs(v) > 0.85 * v_ref
              and sample_at(t)["fase"] in drive_phases
              and p_max > 0 and abs(sample_at(t)["pwr"]) >= 0.95 * p_max]
    if not steady:
        steady = [(t, v) for t, v in speed if sample_at(t)["fase"] in drive_phases]
    t_steady = (steady[0][0], steady[-1][0])

    # IMU in fysische eenheden, koers integreren, pad reconstrueren
    ts = [s["t"] - t_drive for s in imu]
    ax = [s["a"][0] / ACC_LSB_G for s in imu]
    ay = [s["a"][1] / ACC_LSB_G for s in imu]
    az = [s["a"][2] / ACC_LSB_G for s in imu]
    gx = [(s["g"][0] - bias[0]) / GYRO_LSB_DPS for s in imu]
    gy = [(s["g"][1] - bias[1]) / GYRO_LSB_DPS for s in imu]
    gz = [(s["g"][2] - bias[2]) / GYRO_LSB_DPS for s in imu]

    n_smooth = max(1, round(SMOOTH_S * 50))
    pitch = moving_avg([math.degrees(math.atan2(a, c)) for a, c in zip(ax, az)], n_smooth)
    roll = moving_avg([math.degrees(math.atan2(b, c)) for b, c in zip(ay, az)], n_smooth)

    heading, x, y, s_at = [], [], [], []
    psi = px = py = 0.0
    prev_s = 0.0
    for i, t in enumerate(ts):
        if i and t > 0:
            psi += gz[i] * (t - max(ts[i - 1], 0))
        s = interp([e - t_drive for e in et], dist, t) if t > 0 else 0.0
        ds = s - prev_s
        px += ds * math.cos(math.radians(psi))
        py += ds * math.sin(math.radians(psi))
        prev_s = s
        heading.append(psi)
        x.append(px)
        y.append(py)
        s_at.append(s)

    enc_heading = [math.degrees((r - l) / WHEEL_DIS) for l, r in zip(left, right)]

    # Pitch voor de hoogte: optrekken/remmen (uit de encoders) aftrekken van de X-versnelling,
    # dan fuseren met de gyro. Het teken van de gyro-Y-as volgt uit de data zelf.
    sp_t, sp_v = [t for t, _ in speed], [v for _, v in speed]
    v_imu = [interp(sp_t, sp_v, t) for t in ts]
    n = len(ts)
    lo_hi = [(max(0, i - 10), min(n - 1, i + 10)) for i in range(n)]
    a_fwd = [(v_imu[b] - v_imu[a]) / (ts[b] - ts[a]) if ts[b] > ts[a] else 0.0 for a, b in lo_hi]
    pitch_acc = [math.degrees(math.atan2(a - f / G, c)) for a, f, c in zip(ax, a_fwd, az)]
    smooth = moving_avg(pitch_acc, 10)
    rate = [(smooth[b] - smooth[a]) / (ts[b] - ts[a]) if ts[b] > ts[a] else 0.0 for a, b in lo_hi]
    gyro_sign = 1 if sum(r * g for r, g in zip(rate, gy)) >= 0 else -1
    # Tijdens het rijden geeft de accelerometer door trillingen een schijnbare helling (gemeten:
    # +0,4° in beide richtingen op een vlakke mat, terwijl de rustwaarden voor en na gelijk zijn).
    # Daarom: in stilstand de accelerometer (betrouwbaar), tijdens het rijden alleen de gyro,
    # vertrekkend van de rusthoek ervoor en lineair bijgesteld naar de rusthoek erna.
    still = [s["pwr"] == 0 and abs(v) < 0.02 for s, v in zip(imu, v_imu)]
    blocks, i = [], 0                                   # (begin, eind, stilstand?)
    while i < n:
        j = i
        while j + 1 < n and still[j + 1] == still[i]:
            j += 1
        blocks.append([i, j, still[i] and ts[j] - ts[i] >= REST_ANCHOR_S])
        i = j + 1
    pitch_f = moving_avg(pitch_acc, 25)
    anchor = {}
    for b0, b1, is_rest in blocks:
        if is_rest:
            anchor[(b0, b1)] = st.mean(pitch_acc[b0:b1 + 1])
            pitch_f[b0:b1 + 1] = [anchor[(b0, b1)]] * (b1 - b0 + 1)
    for k, (b0, b1, is_rest) in enumerate(blocks):
        if is_rest:
            continue
        prev = next((anchor[(a0, a1)] for a0, a1, r in reversed(blocks[:k]) if r), None)
        nxt = next((anchor[(a0, a1)] for a0, a1, r in blocks[k + 1:] if r), None)
        theta = prev if prev is not None else pitch_f[b0]
        seg = []
        for i in range(b0, b1 + 1):
            if i:
                theta += gyro_sign * gy[i] * max(0.0, min(0.1, ts[i] - ts[i - 1]))
            seg.append(theta)
        if nxt is not None:
            drift = seg[-1] - nxt
            seg = [v - drift * (q + 1) / len(seg) for q, v in enumerate(seg)]
        pitch_f[b0:b1 + 1] = seg
    pitch0 = math.degrees(math.atan2(acc_rest[0], acc_rest[2]))
    roll0 = math.degrees(math.atan2(acc_rest[1], acc_rest[2]))
    def integrate(tilt):
        out = [0.0]
        for i in range(1, n):
            out.append(out[-1] + math.sin(math.radians(pitch_f[i] - pitch0 - tilt * moving[i])) * (s_at[i] - s_at[i - 1]))
        return out
    # Het chassis kantelt onder het motorkoppel (vooruit neus omhoog, achteruit staart omhoog);
    # dat is geen helling. Bij heen en terug rijdt de robot over hetzelfde pad terug, dus moet de
    # hoogte op nul eindigen: daaruit volgt de koppelkanteling, die we aftrekken.
    moving = [(v > 0.05) - (v < -0.05) for v in v_imu]
    z = integrate(0.0)
    heen_terug = any(s["fase"] == "terug" for s in imu)
    abs_ds = sum(abs(s_at[i] - s_at[i - 1]) for i in range(1, n) if moving[i])
    torque_tilt = math.degrees(math.asin(max(-1, min(1, z[-1] / abs_ds)))) if heen_terug and abs_ds > 0.2 else 0.0
    if torque_tilt:
        z = integrate(torque_tilt)

    # Trillingen: verticale versnelling min lokaal gemiddelde, RMS per venster
    az_lp = moving_avg(az, n_smooth)
    vib, i = [], 0
    n_vib = max(2, round(VIB_WINDOW_S * 50))
    while i + n_vib <= len(ts):
        w = range(i, i + n_vib)
        rms = math.sqrt(sum((az[k] - az_lp[k]) ** 2 for k in w) / n_vib)
        vib.append((ts[i + n_vib // 2], s_at[i + n_vib // 2], rms))
        i += n_vib

    def in_steady(t):
        return any(abs(t - q) <= 0.12 for q, _ in steady)

    def rms_dev(vals, idx):
        m = st.mean(vals[k] for k in idx)
        return math.sqrt(st.mean((vals[k] - m) ** 2 for k in idx))

    idx_steady = [k for k, t in enumerate(ts) if in_steady(t)]
    idx_rest = [k for k, s in enumerate(imu) if s["fase"] == "rust"]
    end = len(ts) - 1
    after = [k for k, s in enumerate(imu) if s["fase"] == "uitbollen" and ts[k] >= ts[-1] - 0.5]
    def final_angle(vals):
        return rnd(st.mean(vals[k] for k in after), 2) if after else None

    def motor_vals(key, fase, idx, scale):
        v = [m[4][idx] for m in motor if m[3] == key and m[1] == fase]
        return v and [scale * x for x in v]

    amps = [abs(a) / 10 for m in motor if m[3] == "A" and m[1] in drive_phases for a in m[4]]
    batt_start = motor_vals("V", "rust", 1, 0.1)
    batt_drive = [m[4][1] * 0.1 for m in motor if m[3] == "V" and m[1] in drive_phases]
    temps = [t for m in motor if m[3] == "T" for t in m[4]]

    start_e = max(0, next((i for i, t in enumerate(et) if t >= t_drive), 1) - 1)
    def travelled(vals):
        out = [0.0] * len(vals)
        for i in range(start_e + 1, len(vals)):
            out[i] = out[i-1] + abs(vals[i] - vals[i-1])
        return out
    cumulative = travelled(dist)
    total = cumulative[-1]
    afstand = [interp(et, cumulative, t + t_drive) if t >= 0 else 0 for t in ts]

    # Gebeurtenissen: periodes waarin een voorwaarde lang genoeg geldt.
    def runs(flags, min_s):
        out, i = [], 0
        while i < n:
            if flags[i]:
                j = i
                while j + 1 < n and flags[j + 1]:
                    j += 1
                if ts[j] - ts[i] >= min_s:
                    out.append((i, j))
                i = j + 1
            else:
                i += 1
        return out

    events = []
    def event(kind, i, j, detail):
        events.append({"soort": kind, "t0": rnd(ts[i], 2), "t1": rnd(ts[j], 2), "s": rnd(afstand[i], 2),
                       "x": rnd(x[i], 3), "y": rnd(y[i], 3), "z": rnd(z[i], 3), "detail": detail})

    mag = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(ax, ay, az)]
    for i, j in runs([m < FREEFALL_G for m in mag], FREEFALL_MIN_S):
        dur = ts[j] - ts[i] + 0.02
        land = max(mag[j: j + 15])
        event("vrije val", i, j, f"{dur * 1000:.0f} ms onder {FREEFALL_G} g · val ca. {0.5 * G * dur * dur * 100:.1f} cm · landing {land:.1f} g")
    for i, j in runs([abs(p - pitch0) > TILT_DEG or abs(r - roll0) > TILT_DEG for p, r in zip(pitch_f, roll)], TILT_MIN_S):
        pk = max(range(i, j + 1), key=lambda k: abs(pitch_f[k] - pitch0))
        rk = max(range(i, j + 1), key=lambda k: abs(roll[k] - roll0))
        event("schuin", i, j, f"{ts[j] - ts[i]:.1f} s · pitch tot {pitch_f[pk] - pitch0:+.1f}° · roll tot {roll[rk] - roll0:+.1f}° t.o.v. rust")

    k_v = v_ref / p_max if p_max else 0
    def side_speed(vals):
        out = [(et[i] - t_drive, (vals[i] - vals[i - 1]) / (et[i] - et[i - 1])) for i in range(1, len(et)) if et[i] - et[i - 1] > 0.05]
        return [interp([q[0] for q in out], [q[1] for q in out], t) for t in ts] if out else [0.0] * n
    vl, vr = side_speed(left), side_speed(right)
    driving = [s["fase"] in drive_phases and abs(s["pwr"]) >= 0.5 * p_max for s in imu]
    expect = [k_v * abs(s["pwr"]) for s in imu]
    for i, j in runs([d and abs(v) < STALL_FRAC * e for d, v, e in zip(driving, v_imu, expect)], STALL_MIN_S):
        amps_in = [abs(a) / 10 for m in motor if m[3] == "A" and ts[i] <= m[0] - t_drive <= ts[j] for a in m[4]]
        event("vastgelopen", i, j, f"{ts[j] - ts[i]:.1f} s · {abs(st.mean(v_imu[i:j + 1])):.2f} m/s bij opdracht {imu[i]['pwr']}"
              + (f" · stroom tot {max(amps_in):.1f} A" if amps_in else ""))
    for i, j in runs([d and abs(v) > SPIN_FRAC * e for d, v, e in zip(driving, v_imu, expect)], SPIN_MIN_S):
        event("wielen vrij", i, j, f"{ts[j] - ts[i]:.1f} s · {max(abs(v) for v in v_imu[i:j + 1]):.2f} m/s, normaal {expect[i]:.2f} m/s: opgetild of in de lucht?")
    for i, j in runs([d and max(abs(a), abs(b)) > 0.05 and abs(a - b) > SIDE_FRAC * max(abs(a), abs(b))
                      for d, a, b in zip(driving, vl, vr)], SIDE_MIN_S):
        side = "links" if abs(st.mean(vl[i:j + 1])) < abs(st.mean(vr[i:j + 1])) else "rechts"
        event("één kant", i, j, f"{ts[j] - ts[i]:.1f} s · {side} trager: links {st.mean(vl[i:j + 1]):.2f}, rechts {st.mean(vr[i:j + 1]):.2f} m/s")
    for t, words in notes:
        if words[0] == "VAST":
            i = min(range(n), key=lambda k: abs(ts[k] - (t - t_drive)))
            event("afgebroken", i, i, f"rijscript stopte het segment '{words[1]}': geen voortgang")
    events.sort(key=lambda e: e["t0"])

    doelen = {}
    for t, words in notes:
        if words[0] == "DOEL":
            doelen[words[1]] = {"fase": words[1], "doel_m": float(words[2]), "gemeten_m": None, "fout_mm": None}
        elif words[0] == "STOP" and words[1] in doelen:
            d = doelen[words[1]]
            d["gemeten_m"] = float(words[2])
            d["fout_mm"] = rnd((d["gemeten_m"] - d["doel_m"]) * 1000, 1)
    summary = {
        "rit": Path(path).stem,
        "rittype": "heen en terug" if any(s["fase"] == "terug" for s in imu) else "rechtdoor",
        "verplaatsing_m": rnd(dist[-1], 2),
        "snelheid_max_ms": rnd(v_max, 2),
        "pitch_eind_deg": final_angle(pitch),
        "roll_eind_deg": final_angle(roll),
        "afstand_m": rnd(total, 2),
        "afstand_links_m": rnd(travelled(left)[-1], 2),
        "afstand_rechts_m": rnd(travelled(right)[-1], 2),
        "snelheid_ms": rnd(st.mean(abs(v) for _, v in steady), 2),
        "rijtijd_s": rnd(sum(imu[i+1]["t"] - s["t"] for i, s in enumerate(imu[:-1]) if s["fase"] in drive_phases), 1),
        "koersdrift_gyro_deg": rnd(heading[end], 2),
        "koersdrift_encoder_deg": rnd(enc_heading[-1], 2),
        "zijdelings_eind_cm": rnd(y[end] * 100, 1),
        "zijdelings_max_cm": rnd(max(y[:end + 1], key=abs) * 100, 1),
        "pitch_rust_deg": rnd(math.degrees(math.atan2(acc_rest[0], acc_rest[2])), 2),
        "roll_rust_deg": rnd(math.degrees(math.atan2(acc_rest[1], acc_rest[2])), 2),
        "pitch_rit_gem_deg": rnd(st.mean(pitch[k] for k in idx_steady), 2),
        "pitch_rit_min_deg": rnd(min(pitch[k] for k in idx_steady), 2),
        "pitch_rit_max_deg": rnd(max(pitch[k] for k in idx_steady), 2),
        "roll_rit_gem_deg": rnd(st.mean(roll[k] for k in idx_steady), 2),
        "roll_rit_min_deg": rnd(min(roll[k] for k in idx_steady), 2),
        "roll_rit_max_deg": rnd(max(roll[k] for k in idx_steady), 2),
        "trilling_z_rms_g": rnd(rms_dev(az, idx_steady), 3),
        "trilling_x_rms_g": rnd(rms_dev(ax, idx_steady), 3),
        "trilling_y_rms_g": rnd(rms_dev(ay, idx_steady), 3),
        "trilling_z_rust_g": rnd(rms_dev(az, idx_rest), 3),
        "piek_z_g": rnd(max(abs(az[k] - 1) for k in idx_steady), 2),
        "optrekken_g": rnd(max(moving_avg(ax, 10)[k] for k, t in enumerate(ts) if 0 <= t <= 1.0) - acc_rest[0] / ACC_LSB_G, 2),
        "remmen_g": rnd(min(moving_avg(ax, 10)[k] for k, t in enumerate(ts) if t >= t_steady[1]) - acc_rest[0] / ACC_LSB_G, 2),
        "gyro_bias_counts": [rnd(b, 2) for b in bias],
        "stroom_max_a": rnd(max(amps), 1) if amps else None,
        "batterij_start_v": rnd(st.mean(batt_start), 1) if batt_start else None,
        "batterij_rit_min_v": rnd(min(batt_drive), 1) if batt_drive else None,
        "temp_max_c": max(temps) if temps else None,
        "imu_samples": len(imu),
        "imu_hz": rnd((len(imu) - 1) / (imu[-1]["t"] - imu[0]["t"]), 1),
        "steady_venster_s": [rnd(t_steady[0], 2), rnd(t_steady[1], 2)],
        "hoogte_min_cm": rnd(min(z) * 100, 1),
        "hoogte_max_cm": rnd(max(z) * 100, 1),
        "hoogte_eind_cm": rnd(z[-1] * 100, 1),
        "gyro_pitch_teken": gyro_sign,
        "wielstraal_m": WHEEL_R,
        "koppelkanteling_deg": rnd(torque_tilt, 2),
        "gebeurtenissen": events,
        "doelen": list(doelen.values()),
    }
    series = {
        "t": [rnd(t, 3) for t in ts],
        "s": [rnd(v, 3) for v in s_at],
        "fase": [s["fase"] for s in imu],
        "afstand": [rnd(v, 3) for v in afstand],
        "pwr": [s["pwr"] for s in imu],
        "ax": [rnd(v, 3) for v in ax], "ay": [rnd(v, 3) for v in ay], "az": [rnd(v, 3) for v in az],
        "gz": [rnd(v, 2) for v in gz],
        "pitch": [rnd(v, 3) for v in pitch], "roll": [rnd(v, 3) for v in roll],
        "heading": [rnd(v, 3) for v in heading],
        "pitch_f": [rnd(v - pitch0 - torque_tilt * m, 2) for v, m in zip(pitch_f, moving)],
        "z": [rnd(v * 100, 2) for v in z],
        "x": [rnd(v, 3) for v in x], "y": [rnd(v * 100, 2) for v in y],
        "speed": [[rnd(t, 2), rnd(v, 3)] for t, v in speed],
        "enc_heading": [[rnd(e - t_drive, 2), rnd(s, 3), rnd(h, 2)] for e, s, h in zip(et, dist, enc_heading)],
        "vib": [[rnd(t, 2), rnd(s, 3), rnd(r, 4)] for t, s, r in vib],
    }
    return {"summary": summary, "series": series}


def write_dashboard(analysis_dir, selected_rit):
    """Bundel alle analyses in één HTML, ook rechtstreeks te openen via file://."""
    rides = [json.loads(p.read_text(encoding="utf-8"))
             for p in sorted(analysis_dir.glob("rit_*_analyse.json"), reverse=True)]
    payload = {"rides": rides, "selected": selected_rit}
    # Een bestandsnaam mag het JSON-scriptblok niet afsluiten.
    data = json.dumps(payload, separators=(",", ":")).replace("<", "\\u003c")
    html = TEMPLATE.read_text(encoding="utf-8").replace("__RIT_DATA__", data)
    out_html = HERE / "dashboard.html"
    out_html.write_text(html, encoding="utf-8")
    return out_html


def main():
    paths = list((HERE / "ritten").glob("rit_*.csv"))
    if len(sys.argv) <= 1 and not paths:
        raise SystemExit("Geen rit-CSV gevonden.")
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else max(paths, key=lambda p: p.stat().st_mtime)
    result = analyse(path)
    out_json = path.with_name(path.stem + "_analyse.json")
    out_json.write_text(json.dumps(result, separators=(",", ":")), encoding="utf-8")
    for k, v in result["summary"].items():
        print(f"{k:26} {v}")
    if TEMPLATE.exists():
        out_html = write_dashboard(path.parent, result["summary"]["rit"])
        print(f"Dashboard: {out_html}")


if __name__ == "__main__":
    main()
