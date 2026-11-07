"""
Unit tests of the flight model alone, without the firmware: trim, rotor law, thrust law,
rate response, the flap/body mode and its rpm scaling, the integrator, and the sensor
model. About a minute, standard library only:

    python3 -m unittest discover -s tests -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'heli'))
from heli_model import G, MODELS, Heli, M7R, Stratos200, euler, induced_velocity, ground_effect  # noqa: E402
from sensors import Sensors, available  # noqa: E402

DT = 250e-6


def airborne(heli, rpm_frac=1.0):
    """A heli in free air at a fraction of its nominal rotor speed, level and at rest."""
    heli.reset()
    heli.ground = False
    heli.on_ground = False
    heli.pos = [0.0, 0.0, -100.0]
    heli.w = heli.nominal_w * rpm_frac
    return heli


def hold_rpm(heli, n, dt, ctrl, rpm_frac=1.0):
    """Step n times with the rotor speed held (a perfect governor)."""
    for _ in range(n):
        heli.step(dt, *ctrl)
        heli.w = heli.nominal_w * rpm_frac


def hover_ctrl(heli):
    # throttle irrelevant with the rpm held; collective at the model's hover point
    return (0.5, 0.0, 0.0, 0.0, heli.coll_hover / heli.coll_scale)


class Trim(unittest.TestCase):

    def test_hover_collective_carries_the_weight(self):
        for name, cls in MODELS.items():
            with self.subTest(model=name):
                h = airborne(cls())
                if h.thrust_inflow:
                    # the inflow law's hover point: thrust at the preset's hover collective
                    c = -G / h.Zcol
                    self.assertAlmostEqual(h.thrust_g(c), 1.0, delta=0.02)
                hold_rpm(h, 2000, DT, (0.5, 0, 0, 0, (-G / h.Zcol)))
                # half a second at the hover collective: hardly any vertical speed
                self.assertLess(abs(h.vel[2]), 0.3, h.vel)

    def test_attitude_stays_level_without_input(self):
        for name, cls in MODELS.items():
            with self.subTest(model=name):
                h = airborne(cls())
                hold_rpm(h, 8000, DT, (0.5, 0, 0, 0, -G / h.Zcol))
                r, p, _ = euler(h.q)
                self.assertLess(max(abs(r), abs(p)), math.radians(0.5))


class RotorLaw(unittest.TestCase):

    def test_spool_up_settles(self):
        """Constant throttle, half the hover collective (stays on the ground): the rotor
        speeds up monotonically to a steady speed in the right range."""
        for name, cls in MODELS.items():
            with self.subTest(model=name):
                h = cls()
                c = -0.5 * G / h.Zcol
                u = 0.65 if not h.motor_fit else 0.6
                last, t = 0.0, 0.0
                while t < 20.0:
                    h.step(DT, u, 0, 0, 0, c)
                    # monotonic, apart from the pack sagging as it drains
                    self.assertGreaterEqual(h.w, last * (1 - 1e-6))
                    last = h.w
                    t += DT
                w1 = h.w
                for _ in range(4000):
                    h.step(DT, u, 0, 0, 0, c)
                self.assertLess(abs(h.w - w1) / w1, 0.002, 'not settled')
                self.assertGreater(h.w, 0.5 * h.nominal_w)
                self.assertLess(h.w, 1.5 * h.nominal_w)

    def test_blade360_hover_point(self):
        """The Blade 360's motor is anchored at 65 % throttle for the nominal speed in hover."""
        h = Heli()
        for _ in range(int(15 / DT)):
            h.step(DT, 0.65, 0, 0, 0, h.coll_hover)
        self.assertAlmostEqual(h.rotor_rpm, 2100, delta=21)

    def test_collective_loads_the_rotor(self):
        for name, cls in MODELS.items():
            with self.subTest(model=name):
                speeds = []
                for c in (0.0, 0.5):
                    h = cls()
                    for _ in range(int(15 / DT)):
                        h.step(DT, 0.6, 0, 0, 0, c)
                    speeds.append(h.w)
                self.assertGreater(speeds[0], speeds[1])


class ThrustLaw(unittest.TestCase):

    def test_stratos_hover_collective_per_headspeed(self):
        """Fitted from the logs: hover at 0.636 at 3600 rpm, 0.70-0.73 at 3300."""
        h = Stratos200()
        for rpm, lo, hi in ((3600, 0.62, 0.65), (3300, 0.69, 0.74)):
            w = rpm * 2 * math.pi / 60
            c = self._hover_coll(h, w)
            self.assertTrue(lo <= c <= hi, '%d rpm: %.3f' % (rpm, c))

    def test_m7r_hover_collective_per_headspeed(self):
        """0.346 at 1500 rpm (logged cruise), 0.273 at 1800 (the preset's law)."""
        h = M7R()
        for rpm, want in ((1500, 0.346), (1800, 0.273)):
            c = self._hover_coll(h, rpm * 2 * math.pi / 60)
            self.assertAlmostEqual(c, want, delta=0.02, msg='%d rpm' % rpm)

    def test_thrust_monotonic_and_odd(self):
        for h in (Stratos200(), M7R()):
            prev = -1e9
            for k in range(-20, 21):
                n = h.thrust_g(k / 20, h.nominal_w)
                if k:
                    self.assertGreater(n, prev)
                prev = n
            self.assertAlmostEqual(h.thrust_g(0.4, h.nominal_w), -h.thrust_g(-0.4, h.nominal_w))

    @staticmethod
    def _hover_coll(h, w):
        lo, hi = 0.0, 1.5
        for _ in range(60):
            mid = (lo + hi) / 2
            if h.thrust_g(mid, w) < 1.0:
                lo = mid
            else:
                hi = mid
        return lo


class RateResponse(unittest.TestCase):

    def test_steady_rate_per_cyclic(self):
        """Flapping at equilibrium: rate = Lflt / tf per unit of cyclic (the fitted gain)."""
        for cls, roll_gain, pitch_gain in ((Stratos200, 5.07, 4.74), (M7R, 6.8, 5.7)):
            with self.subTest(model=cls.__name__):
                h = airborne(cls())
                ctrl = (0.5, 0.1, 0.0, 0.0, -G / h.Zcol)
                hold_rpm(h, int(0.5 / DT), DT, ctrl)
                self.assertAlmostEqual(h.rates[0], 0.1 * roll_gain, delta=0.02 * roll_gain)
                h = airborne(cls())
                hold_rpm(h, int(0.5 / DT), DT, (0.5, 0.0, 0.1, 0.0, -G / h.Zcol))
                self.assertAlmostEqual(h.rates[1], 0.1 * pitch_gain, delta=0.02 * pitch_gain)

    def test_m7r_rate_scales_with_rpm(self):
        """ctrl_rpm_exp 0, stiffness ~rpm^2, tf ~1/rpm: the rate per cyclic goes ~rpm."""
        rates = []
        for frac in (1.0, 1.2):
            h = airborne(M7R(), frac)
            hold_rpm(h, int(0.5 / DT), DT, (0.5, 0.1, 0, 0, -G / h.Zcol), frac)
            rates.append(h.rates[0])
        self.assertAlmostEqual(rates[1] / rates[0], 1.2, delta=0.03)


def mode_freq(h, frac=1.0, axis=0, dt=DT, seconds=0.5):
    """Free response of the flap/body mode to a rate kick: frequency from zero crossings
    of p (roll) or q (pitch), in Hz."""
    h = airborne(h, frac)
    h.rates[axis] = 0.5
    xs = []
    for _ in range(int(seconds / dt)):
        h.step(dt, 0.5, 0, 0, 0, -G / h.Zcol)
        h.w = h.nominal_w * frac
        xs.append(h.rates[axis])
    cross = [i for i in range(1, len(xs)) if xs[i - 1] > 0 >= xs[i]]
    if len(cross) < 2:
        return None
    return (len(cross) - 1) / ((cross[-1] - cross[0]) * dt)


class FlapBodyMode(unittest.TestCase):
    """Open loop, the rate and the flapping form a second-order mode: p' = L b1s,
    b1s' = -p - b1s / tf, so s^2 + s / tf + L = 0 and f = sqrt(L - 1 / (4 tf^2)) / 2 pi -
    5.74 Hz for the Stratos (L 1400, tf 0.05), the real heli's ~5.9 Hz whirl."""

    def analytic(self, L, tf):
        return math.sqrt(L - 1 / (4 * tf * tf)) / (2 * math.pi)

    def test_stratos_mode(self):
        h = Stratos200()
        self.assertAlmostEqual(mode_freq(h), self.analytic(1400, 0.05), delta=0.3)
        self.assertAlmostEqual(mode_freq(Stratos200(), axis=1), self.analytic(2000, 0.05), delta=0.3)

    def test_m7r_mode(self):
        self.assertAlmostEqual(mode_freq(M7R()), self.analytic(700, 0.05), delta=0.3)

    def test_mode_scales_with_rpm(self):
        """Stiffness ~rpm^2, tf ~1/rpm: both terms ~rpm^2, so the mode frequency ~rpm."""
        f1 = mode_freq(Stratos200(), 1.0)
        f2 = mode_freq(Stratos200(), 3300 / 3600)
        want = self.analytic(1400 * (3300 / 3600) ** 2, 0.05 * 3600 / 3300)
        self.assertAlmostEqual(f2, want, delta=0.3)
        self.assertAlmostEqual(f2 / f1, 3300 / 3600, delta=0.01)


class Integrator(unittest.TestCase):
    """The model steps with fixed-step Euler at the SITL's 4 kHz. Against the same model at
    40 kHz (a reference for the stiffest flap/body dynamics), a cyclic step response must
    agree within 2 % of its peak, and the mode frequency within 1 %."""

    def response(self, cls, dt):
        h = airborne(cls())
        out = []
        t = 0.0
        while t < 0.4:
            hold_rpm(h, 1, dt, (0.5, 0.1, 0.05, 0.0, -G / h.Zcol))
            t += dt
            out.append((t, h.rates[0], h.rates[1]))
        return out

    def test_euler_at_4khz(self):
        for cls in (Heli, Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                ref = self.response(cls, DT / 10)
                run = self.response(cls, DT)
                peak = max(abs(r[1]) for r in ref)
                for k in range(1, len(run), 40):
                    t, p, q = run[k]
                    rr = ref[min(len(ref) - 1, int(round(t / (DT / 10))) - 1)]
                    self.assertLess(abs(p - rr[1]), 0.02 * peak, 't %.3f' % t)
                f_ref = mode_freq(cls(), dt=DT / 10)
                f_run = mode_freq(cls(), dt=DT)
                if f_ref:
                    self.assertAlmostEqual(f_run / f_ref, 1.0, delta=0.01)


class Inflow(unittest.TestCase):
    """Momentum theory: induced velocity, translational lift, ground effect, vortex ring,
    and the autorotation it gives."""

    def test_induced_velocity(self):
        self.assertAlmostEqual(induced_velocity(3.0, 0.0, 0.0), 3.0)
        # climb lowers it, the vortex ring raises it, the windmill brake brings it back down
        self.assertLess(induced_velocity(3.0, 3.0, 0.0), 3.0)
        self.assertGreater(induced_velocity(3.0, -3.0, 0.0), 1.5 * 3.0)
        self.assertLess(induced_velocity(3.0, -9.0, 0.0), 3.0)
        # forward flight: Glauert, vi -> vh^2 / V at speed
        self.assertAlmostEqual(induced_velocity(3.0, 0.0, 15.0), 9.0 / 15.0, delta=0.02)
        prev = 3.0
        for vx in (1, 2, 4, 8):
            vi = induced_velocity(3.0, 0.0, vx)
            self.assertLess(vi, prev)
            prev = vi

    def test_ground_effect(self):
        self.assertAlmostEqual(ground_effect(0.5, 100.0), 1.0, places=4)
        self.assertLess(ground_effect(0.5, 0.3), 0.9)
        self.assertGreaterEqual(ground_effect(0.5, 0.01), 0.5)

    def steady(self, h, seconds=3.0, vel=(0.0, 0.0, 0.0), alt=100.0, coll=None):
        """Thrust after a few seconds at the nominal rotor speed, with the velocity held."""
        h.reset()
        h.pos = [0.0, 0.0, -alt]
        h.on_ground = False
        h.ground = alt < 50
        h.w = h.nominal_w
        h.n = 1.0
        c = h.coll_hover if coll is None else coll
        for _ in range(int(seconds / DT)):
            h.vel = list(vel)
            h.rates = [0.0, 0.0, 0.0]
            h.step(DT, 0.5, 0, 0, 0, c)
            h.w = h.nominal_w
        return h.n

    def test_translational_lift(self):
        for cls in (Heli, Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                hover = self.steady(cls())
                fwd = self.steady(cls(), vel=(10.0, 0.0, 0.0))
                self.assertAlmostEqual(hover, 1.0, delta=0.03)
                self.assertGreater(fwd, 1.15 * hover)

    def test_ground_effect_thrust(self):
        for cls in (Heli, Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                oge = self.steady(cls())
                ige = self.steady(cls(), alt=0.05)
                self.assertGreater(ige, 1.05 * oge)

    def test_vortex_ring_gives_no_power_relief(self):
        """Sinking at about v_h the rotor sits in its own wake: the total inflow (v_c + v_i),
        and with it the induced power, stays near hover - the descent buys nothing. Only
        beyond ~2 v_h (windmill brake) does the inflow reverse and drive the rotor."""
        for vh in (2.5, 3.3, 3.8):
            hover = induced_velocity(vh, 0.0, 0.0)
            ring = -vh + induced_velocity(vh, -vh, 0.0)
            brake = -3 * vh + induced_velocity(vh, -3 * vh, 0.0)
            self.assertAlmostEqual(ring / hover, 1.0, delta=0.1)
            self.assertLess(brake, 0.0)

    def test_autorotation(self):
        """Motor off, free rotor and free fall at altitude: the airflow keeps the rotor
        turning and the heli settles into a steady descent at 1 g of thrust, 1.5-3 v_h."""
        for cls in (Heli, Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                h = cls()
                h.ground = False
                h.on_ground = False
                h.pos = [0.0, 0.0, -500.0]
                h.w = h.nominal_w
                h.n = 1.0
                for _ in range(int(15 / DT)):
                    h.step(DT, 0.0, 0, 0, 0, 0.3 * h.coll_hover)
                    h.rates = [0.0, 0.0, 0.0]
                self.assertAlmostEqual(h.n, 1.0, delta=0.05)
                self.assertGreater(h.w, 0.7 * h.nominal_w)
                self.assertTrue(1.5 * h.v_hover < h.vel[2] < 3.0 * h.v_hover, h.vel[2])


class SensorModel(unittest.TestCase):

    def rms(self, s, w, n=20000):
        acc = [0.0, 0.0, 0.0]
        for _ in range(n):
            g, _ = s.step(DT, [0.0, 0.0, 0.0], [0.0, 0.0, -G], w)
            for i in range(3):
                acc[i] += g[i] * g[i]
        return [math.degrees(math.sqrt(a / n)) for a in acc]

    def test_ideal_without_data(self):
        s = Sensors()
        g, f = s.step(DT, [0.1, 0.2, 0.3], [0.0, 0.0, -G], 200.0)
        self.assertEqual(g, [0.1, 0.2, 0.3])

    def test_seeded(self):
        if not available('helis') or not available('sensors'):
            self.skipTest('no fitted sensor data')
        a = Sensors(available('sensors')[0], available('helis')[0], seed=7)
        b = Sensors(available('sensors')[0], available('helis')[0], seed=7)
        for _ in range(100):
            self.assertEqual(a.step(DT, [0, 0, 0], [0, 0, -G], 190.0), b.step(DT, [0, 0, 0], [0, 0, -G], 190.0))

    def test_chip_white_noise_level(self):
        """The white noise comes out at the fitted density: std = N sqrt(fs)."""
        for chip in available('sensors'):
            with self.subTest(chip=chip):
                s = Sensors(chip)
                s.g_bias = [0.0] * 3            # isolate the white part
                s.gyro_bias = [0.0] * 3
                s.g_walk = [0.0] * 3
                got = self.rms(s, 0.0)
                want = [n / math.sqrt(DT) for n in s.chip['gyro']['white_dps_rthz']]
                for g_, w_ in zip(got, want):
                    self.assertAlmostEqual(g_ / w_, 1.0, delta=0.05)

    def test_vibration_scales_with_rpm_squared(self):
        for heli in available('helis'):
            with self.subTest(heli=heli):
                s = Sensors(None, heli)
                w = s.rpm_ref * 2 * math.pi / 60
                a = self.rms(Sensors(None, heli), w)
                b = self.rms(Sensors(None, heli), 0.8 * w)
                for x, y in zip(a, b):
                    if x > 0.1:
                        self.assertAlmostEqual(y / x, 0.64, delta=0.05)
                self.assertEqual(self.rms(Sensors(None, heli), 0.0, 2000), [0.0, 0.0, 0.0])


if __name__ == '__main__':
    unittest.main()
