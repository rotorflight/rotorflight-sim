"""
Wind for the heli model: a steady wind plus Dryden turbulence, as the air velocity in
earth axes (NED, m/s). The model flies relative to it (heli_model: rotor inflow,
translational lift, drag and the speed derivatives all take the air-relative velocity).

Turbulence follows the Dryden model of MIL-F-8785C at low altitude, with first-order
shaping filters on all three components (the standard has second order on the lateral
and vertical ones - the same variance and length, a slightly different spectrum shape):

  sigma_w = gust (the vertical intensity, m/s; MIL-F-8785C: 0.1 x the wind at 20 ft)
  sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4         h in ft, 10-1000 ft
  L_w = h,  L_u = L_v = h / (0.177 + 0.000823 h)^1.2              in ft
  x' = -(V / L) x + sigma sqrt(2 V / L) white noise               V: airspeed, at least 1 m/s

u is along the mean wind, v across it, w vertical. Seeded, so checks stay repeatable.
Only the Python standard library.
"""

import math
import random

FT = 0.3048


class Wind:
    def __init__(self, speed=0.0, from_deg=0.0, gust=0.0, seed=2):
        """speed (m/s) blowing from from_deg (0 = from north, 90 = from east), gust =
        vertical turbulence intensity sigma_w (m/s)."""
        a = math.radians(from_deg)
        # blowing from the north means the air moves south
        self.dir = (-math.cos(a), -math.sin(a))
        self.speed = speed
        self.gust = gust
        self.rng = random.Random(seed)
        self.x = [0.0, 0.0, 0.0]

    def describe(self):
        return 'wind %.1f m/s, Dryden sigma_w %.2f m/s' % (self.speed, self.gust)

    def scales(self, alt):
        """(sigma_u, sigma_w), (L_u, L_w) in m at height alt (m)."""
        h = min(1000.0, max(10.0, alt / FT))
        k = 0.177 + 0.000823 * h
        su = self.gust / k ** 0.4
        return (su, self.gust), (h / k ** 1.2 * FT, h * FT)

    def step(self, dt, alt, air_speed):
        """Air velocity in earth axes after dt, at height alt with the heli moving at
        air_speed through the air."""
        dx, dy = self.dir
        mean = (self.speed * dx, self.speed * dy, 0.0)
        if self.gust <= 0.0:
            return mean
        (su, sw), (lu, lw) = self.scales(alt)
        V = max(1.0, air_speed)
        for i, (s, L) in enumerate(((su, lu), (su, lu), (sw, lw))):
            a = V / L
            self.x[i] += (-a * self.x[i]) * dt + s * math.sqrt(2 * a * dt) * self.rng.gauss(0.0, 1.0)
        u, v, w = self.x
        # u along the mean wind, v across it (90 deg right of it), w down
        return (mean[0] + u * dx - v * dy, mean[1] + u * dy + v * dx, w)
