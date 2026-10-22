"""Simulatie van een skid-steer robot om de boogregeling (Jaguar.arc) te controleren, zonder robot:
  python -m jaguar.testen.sim_boog

Model: elke kant volgt het vermogen met een vertraging van 0,25 s, onder een vermogen van 60 gebeurt niets (stilstaande
wrijving), de grond slipt (effectieve spoorbreedte = SPOOR_ECHT, anders dan wat de code aanneemt) en de snelheid is
10 % lager dan het vermogen voorspelt. De gyro meet de koers zonder fout. Het is geen bewijs dat het robot hetzelfde doet,
maar het laat zien of de regeling stabiel is en wat een verkeerde aanname over de slip kost.
Na de boog zet robot.eindkoers() de koers ter plaatse recht als die meer dan 1° afwijkt, dus een resthoek van enkele graden is geen eindresultaat."""
import math
import sys
import time

from jaguar.kern import robot as R


class Plant(R.Jaguar):
    def __init__(self, spoor_echt=1.4, ratio=None):
        self.phase = "boog"
        self.power = 0
        self.stop_requested = False
        self.target_heading = 0.0
        self.line = (0.0, 0.0)
        self.heading_pid = R.PID(R.KP_HEADING, R.KI_HEADING, R.KD_HEADING)
        self.turn_ratio = ratio
        self.k_v = R.K_V
        self.writer = None
        self.spoor_echt = spoor_echt
        self.h = 0.0                       # graden
        self.x = self.y = 0.0
        self.s = 0.0
        self.vl = self.vr = 0.0
        self.pl = self.pr = 0.0
        self.rate = 0.0
        self.last = time.perf_counter()

        class Odo:
            klaar = True
            kanteling = 0.0
            modus = "normaal"
            v = 0.0
            n_enc = 0
        self.odo = Odo()
        self.odo.positie = lambda: self.s
        self.odo.s = 0.0

    def note(self, line):
        pass

    # --- de "wereld"
    def _step(self):
        now = time.perf_counter()
        dt = min(0.2, now - self.last)
        self.last = now
        tl = 0.0 if abs(self.pl) < 60 else R.K_V * 0.9 * self.pl
        tr = 0.0 if abs(self.pr) < 60 else R.K_V * 0.9 * self.pr
        a = min(1.0, dt / 0.25)
        self.vl += (tl - self.vl) * a
        self.vr += (tr - self.vr) * a
        v = (self.vl + self.vr) / 2
        self.rate = math.degrees((self.vr - self.vl) / self.spoor_echt)
        self.h += self.rate * dt
        self.x += v * math.cos(math.radians(self.h)) * dt
        self.y += v * math.sin(math.radians(self.h)) * dt
        self.s += v * dt
        self.odo.v = v
        self.odo.x, self.odo.y = self.x, self.y

    heading = property(lambda self: (self._step(), self.h)[1])
    yaw_rate = property(lambda self: (self._step(), self.rate)[1])

    def _motor(self, p, steer=0):
        self._step()
        self.power = p
        self.pl, self.pr = p - steer, p + steer

    def _guard(self, p):
        pass

    def _settled(self, wait=2.0):
        self._motor(0)
        for _ in range(60):
            time.sleep(0.02)
            self._step()
        return self.s


def proef(radius, hoek, spoor_echt=1.4, ratio=None):
    r = Plant(spoor_echt, ratio)
    t0 = time.perf_counter()
    r.arc(radius, hoek)
    dt = time.perf_counter() - t0
    a = math.radians(abs(hoek))
    ideaal = (radius * math.sin(a), (1 if hoek > 0 else -1) * radius * (1 - math.cos(a)))
    # eindpunt volgens het model in het stelsel van de beginkoers (beginkoers 0)
    fout = math.hypot(r.x - ideaal[0], r.y - ideaal[1])
    dh = r.h - hoek
    print(f"R={radius:4.1f} m {hoek:+5.0f}°  spoor echt {spoor_echt:.2f} m  →  koers {r.h:+7.1f}° (fout {dh:+5.1f}°), "
          f"eindpunt {fout * 100:5.1f} cm van het ideale punt, {dt:4.1f} s")
    return dh, fout


def main():
    gevallen = [(2.0, 90, 1.4), (1.5, -90, 1.4), (1.0, 180, 1.4), (3.0, 45, 1.4),
                (2.0, 90, 0.9), (2.0, 90, 2.0), (0.8, 90, 1.4)]
    slecht = 0
    for radius, hoek, spoor in gevallen:
        dh, fout = proef(radius, hoek, spoor)
        if abs(dh) > 8 or fout > 0.25 * radius:
            slecht += 1
    print("ALLES BINNEN DE MARGES" if not slecht else f"{slecht} gevallen buiten de marges (hoek 8°, positie 25 % van de straal)")
    return 0 if not slecht else 1


if __name__ == "__main__":
    sys.exit(main())
