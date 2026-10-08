"""Slipbewuste odometrie: vier wielencoders + gyro + accelerometer.

Dezelfde klasse draait live in het rijscript en achteraf in analyse_rit.py (herspeel()).

Wat de ritten lieten zien (zie README, "Afstand op elke ondergrond"):
  - De encoders komen op een vaste klok van 0,2 s; de wifi verschuift de ontvangsttijd ±20 ms.
    Snelheid = pulsen / (n × 0,2 s) is daarom 3,5× minder ruizig dan pulsen / ontvangsttijd.
  - Voor- en achterwiel aan dezelfde kant hebben dezelfde grondsnelheid (star chassis); in
    schone ritten verschillen ze < 0,04 m/s (p90). Een groter verschil = één wiel slipt.
  - Links − rechts moet kloppen met de gyro: (vR − vL) ≈ ω × spoorbreedte. Zo niet: één kant slipt.
  - De IMU alleen kan maar ≈ 0,5 s overbruggen (p90-fout 1,5 cm na 0,2 s, 7 cm na 0,6 s,
    17 cm na 1 s). Daarna is de afstand onzeker en moet de robot stoppen.
  - Wielen die snel draaien zonder trillingen (< 0,04 g) raken de grond niet (opgetild, gekanteld).

Werking per encoderpakket (5 Hz):
  1. per kant: voor en achter gelijk → gemiddelde; anders het wiel dat het dichtst bij de
     IMU-voorspelling ligt (het andere slipt);
  2. links/rechts tegen de gyro: klopt het niet, dan de kant die het best bij de IMU past;
  3. Kalman-filter op [snelheid, versnellingsbias]: de IMU voorspelt, de wielen meten. Wijkt de
     wielsnelheid te ver af (alle wielen slippen, vastgelopen, in de lucht), dan telt de IMU-
     integratie in plaats van de wielen, en groeit de onzekerheid;
  4. stilstand (geen pulsen, geen trilling) zet de snelheid op nul en leert de biases bij.
Een glijden van alle vier de wielen samen aan constante snelheid blijft onzichtbaar: daarvoor
dient de proef met vrijlopende meetwielen (test_sondewiel.py).
"""
import math
from collections import deque

G = 9.81
PERIODE = 0.2             # encoderklok van de motion controller (s); wordt bijgeleerd
TOL_WIEL = 0.10           # m/s: voor/achter of links/rechts mogen zoveel verschillen …
TOL_WIEL_REL = 0.05       # … plus dit deel van de snelheid
POORT = 0.25              # m/s: wielsnelheid verder dan dit (+ 3σ) van de IMU-voorspelling = verworpen
MAX_OVERBRUG = 0.6        # s: langer dan dit alleen op de IMU = onzeker
LUCHT_V = 0.4             # m/s wielsnelheid …
LUCHT_TRIL = 0.04         # … met minder trilling dan dit (g) = wielen raken de grond niet
LUCHT_KANTEL = 60.0       # graden t.o.v. de rusthoek: zo schuin draaien de wielen vrij (robot op zijn neus/kant)
STIL_TRIL = 0.03          # g: onder deze trilling en zonder pulsen staat de robot stil
Q_V = 0.006               # procesruis snelheid (m²/s³), uit de overbruggingsmeting
Q_B = 0.004               # procesruis versnellingsbias + helling (m²/s⁵)
R_WIEL = 0.02 ** 2        # meetruis wielsnelheid met vaste klok (m²/s²)
PITCH_TEKEN = -1          # neus omhoog = negatieve gyro Y (gecontroleerd: kantelen naar -85° gaf +81° gyro)


class Odometrie:
    def __init__(self, m_per_cnt, gyro_lsb_dps, acc_lsb_g, spoor=0.52):
        self.m_per_cnt, self.gyro_lsb, self.acc_lsb, self.spoor = m_per_cnt, gyro_lsb_dps, acc_lsb_g, spoor
        self.periode = PERIODE
        self.meetwielen = None         # "achter": achterwielen lopen vrij en meten (test_sondewiel.py)
        self.gbias = None              # gyrobias X, Y, Z (counts); None = nog niet op nul gezet
        self.acc0 = None               # accelerometer in rust (counts): zwaartekracht + bias
        self._wielen = None            # laatste pulsen FL, FR, RL, RR (vooruit positief)
        self._t_enc = None
        self._imu_buf = deque(maxlen=15)   # 0,3 s (acc, gyro) voor trilling en stilstand
        self._hist = deque(maxlen=40)      # (t, v) om de snelheid midden in een interval op te zoeken
        self.gebeurtenissen = []       # (t0, t1, soort, detail, correctie in m)
        self._episode = None
        self.nul(None, None)

    # --- toestand -----------------------------------------------------------------
    def nul(self, gyro_bias, acc_rust):
        """In rust: gyrobias (x, y, z) en zwaartekracht vastleggen; afstand, koers en positie op nul.
        Live roept de leesdraad imu()/encoders() aan terwijl dit loopt: eerst `klaar` uitzetten,
        als laatste weer aan, zodat die nooit een half nulgezette toestand gebruiken."""
        self.gbias = None
        self.acc0 = None
        # Pitch (rad): de zwaartekracht op de X-as is g·sin(pitch). In rust uit de accelerometer,
        # tijdens het rijden uit de gyro (de accelerometer ziet dan ook optrekken en trillingen).
        self.pitch = math.atan2(acc_rust[0], acc_rust[2]) if acc_rust else 0.0
        self.g_norm = math.sqrt(sum(x * x for x in acc_rust)) / self.acc_lsb if acc_rust else 1.0
        self.s = self.x = self.y = 0.0
        self.koers = 0.0               # graden, + = links
        self.draaisnelheid = 0.0       # graden/s, gefilterd
        self.v, self.b = 0.0, 0.0      # m/s en m/s² (bias + zwaartekrachtlek door helling)
        self.P = [[0.01, 0.0], [0.0, 0.05]]
        self.ds_imu = 0.0              # IMU-weg sinds het laatste encoderpakket
        self._dkoers = 0.0
        self._tilt = list(acc_rust) if acc_rust else None
        self.modus = "normaal"         # normaal · slip · overbrugd · onzeker · lucht · stil
        self.overbrug_t = 0.0
        self.onzekerheid = 0.0         # m, opgeteld over alle correcties (conservatief)
        self.afstand = 0.0             # som |ds|
        self.afstand_wielen = 0.0      # som |gemiddelde van 4 wielen|, ter vergelijking
        self.gecorrigeerd = 0.0        # som |ds| waarbij niet het gemiddelde van 4 wielen telde
        self._ok_reeks = 0
        self.n_enc = 0                 # aantal verwerkte encoderpakketten
        self.acc0 = list(acc_rust) if acc_rust else None
        self.gbias = list(gyro_bias) if gyro_bias else None

    @property
    def klaar(self):
        return self.gbias is not None and self.acc0 is not None

    def positie(self):
        """Afstand (m, met teken) nu: laatste encoderpakket + IMU-integratie sindsdien."""
        return self.s + self.ds_imu

    @property
    def trilling(self):
        """Standaardafwijking van de versnelling over 0,3 s (g)."""
        if len(self._imu_buf) < 5:
            return 0.0
        n = len(self._imu_buf)
        tot = 0.0
        for k in range(3):
            m = sum(a[k] for a, _ in self._imu_buf) / n
            tot += sum((a[k] - m) ** 2 for a, _ in self._imu_buf) / n
        return math.sqrt(tot) / self.acc_lsb

    def _rust_bijleren(self):
        """In stilstand: pitch uit de accelerometer, gyrobias langzaam volgen."""
        if len(self._imu_buf) < 10:
            return
        n = len(self._imu_buf)
        am = [sum(a[k] for a, _ in self._imu_buf) / n for k in range(3)]
        self.pitch = math.atan2(am[0], am[2])
        gm = [[g[k] for _, g in self._imu_buf] for k in range(3)]
        if all(max(x) - min(x) < 4 for x in gm):      # gyro rustig (ruis ≈ ±2 counts)
            self.gbias = [b + 0.05 * (sum(x) / n - b) for b, x in zip(self.gbias, gm)]

    @property
    def kanteling(self):
        """Hoek (graden) tussen de huidige zwaartekrachtrichting (0,5 s gefilterd) en die in rust."""
        if not self._tilt or not self.acc0:
            return 0.0
        a, b = self._tilt, self.acc0
        na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
        if na == 0 or nb == 0:
            return 0.0
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (na * nb)))))

    # --- IMU (50 Hz) ---------------------------------------------------------------
    def imu(self, t, dt, g, a):
        """Eén IMU-sample: g en a zijn ruwe counts (x, y, z); dt in seconden."""
        self._imu_buf.append((a, g))
        if not self.klaar:
            return
        dt = max(0.0, min(0.1, dt))
        rate = (g[2] - self.gbias[2]) / self.gyro_lsb
        self.koers += rate * dt
        self._dkoers += rate * dt
        self.draaisnelheid = 0.7 * self.draaisnelheid + 0.3 * rate
        self.pitch += math.radians(PITCH_TEKEN * (g[1] - self.gbias[1]) / self.gyro_lsb) * dt
        k = min(1.0, dt / 0.5)
        self._tilt = [p + k * (q - p) for p, q in zip(self._tilt, a)]
        # Kalman-voorspelling: v += (a − zwaartekracht − b)·dt
        acc = (a[0] / self.acc_lsb - self.g_norm * math.sin(self.pitch)) * G
        self.v += (acc - self.b) * dt
        P = self.P
        p00 = P[0][0] - dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_V * dt
        p01 = P[0][1] - dt * P[1][1]
        self.P = [[p00, p01], [p01, P[1][1] + Q_B * dt]]
        self.ds_imu += self.v * dt
        self._hist.append((t, self.v))

    def _v_op(self, t):
        for th, v in reversed(self._hist):
            if th <= t:
                return v
        return self._hist[0][1] if self._hist else self.v

    def _meet(self, z, r, t_meet):
        """Kalman-update met een gemeten snelheid z (midden van het interval)."""
        y = z - self._v_op(t_meet)
        P = self.P
        S = P[0][0] + r
        k0, k1 = P[0][0] / S, P[1][0] / S
        self.v += k0 * y
        self.b += k1 * y
        self.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
                  [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]

    # --- encoders (5 Hz) -----------------------------------------------------------
    def encoders(self, t, wielen):
        """Pulsen (FL, FR, RL, RR), vooruit positief. Geeft de toegevoegde afstand terug (m)."""
        if self._wielen is None:
            self._wielen, self._t_enc, self.ds_imu, self._dkoers = wielen, t, 0.0, 0.0
            return 0.0
        ruw = t - self._t_enc
        if ruw <= 0:
            return 0.0
        n = max(1, round(ruw / self.periode))
        if n == 1 and 0.5 * PERIODE < ruw < 1.5 * PERIODE:
            self.periode += 0.02 * (ruw - self.periode)
        dt = n * self.periode
        d = [(w - p) * self.m_per_cnt for w, p in zip(wielen, self._wielen)]
        self._wielen, self._t_enc = wielen, t
        w = [x / dt for x in d]
        gem = sum(d) / 4
        om = math.radians(self._dkoers) / dt          # rad/s volgens de gyro
        ds_imu, self.ds_imu, self._dkoers = self.ds_imu, 0.0, 0.0
        if not self.klaar:
            self.v = gem / dt                            # zonder IMU: gemiddelde van 4 wielen
            return self._stap(gem, gem, "normaal")
        t_mid = t - dt / 2
        v_pred = self._v_op(t_mid)
        tril = self.trilling

        # Stilstand: geen pulsen (≤ 1 per wiel) en geen trilling → snelheid nul, biases bijleren.
        if all(abs(x) <= 1.01 * self.m_per_cnt for x in d) and tril < STIL_TRIL:
            self._meet(0.0, 0.01 ** 2, t_mid)
            self.v = 0.0
            self._rust_bijleren()
            self._einde_episode()
            self.overbrug_t, self._ok_reeks = 0.0, 0
            return self._stap(gem, gem, "stil")

        tol = TOL_WIEL + TOL_WIEL_REL * abs(v_pred)
        half = om * self.spoor / 2

        def kant(voor, achter, verwacht):
            if abs(voor - achter) <= tol:
                return (voor + achter) / 2, None
            return (voor, "achter") if abs(voor - verwacht) <= abs(achter - verwacht) else (achter, "voor")

        if self.meetwielen == "achter":
            # Achterwielen zonder vermogen: geen koppel, dus geen aandrijfslip; zij meten de grond.
            # De slip van de voorwielen is dan geen fout maar het verschil met de referentie.
            z, slip = (w[2] + w[3]) / 2, []
        else:
            vl, slip_l = kant(w[0], w[2], v_pred - half)
            vr, slip_r = kant(w[1], w[3], v_pred + half)
            slip = [f"{wiel} {zijde}" for wiel, zijde in ((slip_l, "links"), (slip_r, "rechts")) if wiel]
            z = (vl + vr) / 2
            if abs((vr - vl) - 2 * half) > tol + abs(half):
                cl, cr = vl + half, vr - half            # midden volgens elke kant apart
                if abs(cl - v_pred) <= abs(cr - v_pred):
                    z, slip = cl, slip + ["kant rechts"]
                else:
                    z, slip = cr, slip + ["kant links"]

        S = self.P[0][0] + R_WIEL
        lucht = (abs(z) > LUCHT_V and tril < LUCHT_TRIL) or self.kanteling > LUCHT_KANTEL
        past = abs(z - v_pred) <= POORT + 3 * math.sqrt(S)
        if self.overbrug_t > MAX_OVERBRUG:
            # Te lang op de IMU: terug naar de wielen zodra ze onderling kloppen en de robot trilt
            # zoals bij echt rijden (anders draaien ze in de lucht of slippen ze allemaal).
            self._ok_reeks = self._ok_reeks + 1 if (not slip and not lucht) else 0
            past = self._ok_reeks >= 2
            if past:
                self.v, self.P[0][0] = z, 0.05
        if past and not lucht:
            self._meet(z, R_WIEL if not slip else 4 * R_WIEL, t_mid)
            ds = z * dt
            if slip:
                self.onzekerheid += abs(ds - gem) / 2
            self.overbrug_t = 0.0
            if slip:
                e = self._episode_start(t - dt, "slip", slip)
                e["corr"] += ds - gem
            else:
                self._einde_episode()
            return self._stap(ds, gem, "slip" if slip else "normaal")
        # Wielen verworpen: IMU-integratie telt, maar fysisch begrensd: slippende wielen draaien
        # sneller dan de grond, dus de echte snelheid ligt tussen 0 en het snelste wiel.
        if lucht:
            self._meet(0.0, 0.05 ** 2, t_mid)     # opgetild of gekanteld: de robot rijdt niet
        lo, hi = min(0.0, *w) - 0.1, max(0.0, *w) + 0.1
        self.v = max(lo, min(hi, self.v))
        ds_imu = max(lo * dt, min(hi * dt, ds_imu))
        self.overbrug_t += dt
        soort = "lucht" if lucht else ("onzeker" if self.overbrug_t > MAX_OVERBRUG else "overbrugd")
        e = self._episode_start(t - dt, soort, [f"wielen {z:+.2f} m/s, IMU {v_pred:+.2f} m/s"])
        e["imu"] += ds_imu
        e["wiel"] += z * dt
        return self._stap(ds_imu, gem, soort)

    def _stap(self, ds, gem, modus):
        self.n_enc += 1
        k = math.radians(self.koers)
        self.s += ds
        self.x += ds * math.cos(k)
        self.y += ds * math.sin(k)
        self.afstand += abs(ds)
        self.afstand_wielen += abs(gem)
        if modus not in ("normaal", "stil"):
            self.gecorrigeerd += abs(ds)
        self.modus = modus
        return ds

    # --- gebeurtenissen ------------------------------------------------------------
    # Opeenvolgende pakketten van dezelfde soort vormen één gebeurtenis; overbrugd, onzeker en
    # lucht horen bij elkaar (de zwaarste soort geeft de naam).
    ERNST = {"slip": 0, "overbrugd": 1, "onzeker": 2, "lucht": 3}

    def _episode_start(self, t, soort, wat):
        e = self._episode
        if e and (e["soort"] == "slip") == (soort == "slip"):
            e["t1"] = t + self.periode
            e["wat"].update(wat if soort == "slip" else [])
            if self.ERNST[soort] > self.ERNST[e["soort"]]:
                e["soort"] = soort
            return e
        self._einde_episode()
        self._episode = {"t0": t, "t1": t + self.periode, "soort": soort, "wat": set(wat), "imu": 0.0, "wiel": 0.0, "corr": 0.0}
        return self._episode

    def _einde_episode(self):
        e, self._episode = self._episode, None
        if not e:
            return
        duur = e["t1"] - e["t0"]
        if e["soort"] == "slip":
            detail = f"{', '.join(sorted(e['wat']))} slipt · correctie {e['corr'] * 100:+.1f} cm t.o.v. het gemiddelde van 4 wielen"
        else:
            if e["soort"] == "lucht":
                self.onzekerheid += abs(e["imu"])
            else:
                # Fout van de IMU-overbrugging (p90 uit de ritten), hoogstens het verschil met de wielen.
                self.onzekerheid += min(0.005 + 0.16 * duur * duur, abs(e["imu"] - e["wiel"]))
            oorzaak = "wielen draaien zonder dat de robot rijdt (opgetild, gekanteld of tegen een obstakel)" if e["soort"] == "lucht" else \
                "wielen wijken af van de IMU (alle wielen slippen of vastgelopen)"
            detail = f"{oorzaak} · IMU {e['imu'] * 100:+.0f} cm i.p.v. wielen {e['wiel'] * 100:+.0f} cm"
        corr = e["corr"] if e["soort"] == "slip" else e["imu"] - e["wiel"]
        self.gebeurtenissen.append((e["t0"], e["t1"], e["soort"], detail, corr))

    def afsluiten(self):
        self._einde_episode()


def herspeel(imu, enc, gyro_bias, acc_rust, t_nul, m_per_cnt, gyro_lsb_dps, acc_lsb_g, spoor=0.52):
    """Odometrie achteraf over een rit (formaat van analyse_rit.load).
    Geeft (odo, reeks) met per IMU-sample (t, s, v, modus, x, y, koers, afgelegd); s loopt tussen
    encoderpakketten door met de IMU, afgelegd (som |ds|) springt per pakket."""
    odo = Odometrie(m_per_cnt, gyro_lsb_dps, acc_lsb_g, spoor)
    events = [(s["t"], 0, s) for s in imu]
    for board, rows in enc.items():
        events += [(r[0], 1, (board, r[1], r[2])) for r in rows]
    events.sort(key=lambda e: (e[0], e[1]))
    laatste, nieuw = {}, set()
    reeks, t_vorig, genuld = [], None, False
    for t, soort, data in events:
        if not genuld and t >= t_nul:
            odo.nul(gyro_bias, acc_rust)
            genuld = True
        if soort == 0:
            odo.imu(t, 0.02 if t_vorig is None else t - t_vorig, data["g"], data["a"])
            t_vorig = t
            reeks.append((t, odo.positie(), odo.v, odo.modus, odo.x, odo.y, odo.koers, odo.afstand))
        else:
            board, links, rechts = data
            laatste[board] = (links, rechts)
            nieuw.add(board)
            if {"MM0", "MM1"} <= nieuw:
                f, r = laatste["MM0"], laatste["MM1"]
                odo.encoders(t, (f[0], f[1], r[0], r[1]))
                nieuw.clear()
    odo.afsluiten()
    return odo, reeks
