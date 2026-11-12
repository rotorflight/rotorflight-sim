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
from wind import Wind  # noqa: E402
from servos import Servo, Servos, TYPES  # noqa: E402
from heli_model import LIPO_OCV, Fitted, load_model, resolve, trained_models  # noqa: E402

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
                    # monotonic, apart from the pack sagging as it drains and the heli settling on its skids
                    self.assertGreaterEqual(h.w, last * (1 - 1e-4))
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
        h.set_altitude(alt)
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


class Skids(unittest.TestCase):
    """Skid contact: four points with spring, damper and friction. Rotor stopped, so only
    gravity and the ground act."""

    def run_ground(self, h, seconds):
        for _ in range(int(seconds / DT)):
            h.step(DT, 0.0, 0, 0, 0, 0.0)
        return h

    def tilted(self, roll_deg):
        """Resting on one skid's edge, rolled by roll_deg."""
        h = Heli()
        a = math.radians(roll_deg) / 2
        h.q = (math.cos(a), math.sin(a), 0.0, 0.0)
        # lowest point just touching
        h.pos[2] = -(h.cg_height * math.cos(2 * a) + h.skid_y * math.sin(2 * a)) + 0.002
        return h

    def test_rests_level(self):
        for cls in (Heli, Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                h = self.run_ground(cls(), 2.0)
                r, p, _ = euler(h.q)
                self.assertLess(abs(h.altitude), 0.002)
                self.assertLess(max(abs(r), abs(p)), math.radians(0.1))
                self.assertLess(max(abs(v) for v in h.vel), 0.01)
                self.assertTrue(h.on_ground)
                # the accelerometer reads 1 g up on the ground
                self.assertAlmostEqual(h.specific_force[2], -G, delta=0.05)

    def test_drop(self):
        h = Heli()
        h.set_altitude(1.0)
        h.on_ground = False
        h.air_time = 1.0
        self.run_ground(h, 3.0)
        self.assertAlmostEqual(h.touchdown_speed, math.sqrt(2 * G * 1.0), delta=0.1)
        r, p, _ = euler(h.q)
        self.assertLess(max(abs(r), abs(p)), math.radians(0.5))
        self.assertLess(abs(h.altitude), 0.005)

    def test_slide_and_stop(self):
        """Touching down at 2 m/s sideways: friction stops it within about v^2 / 2 mu g,
        upright (the tipping friction is skid_y / cg_height = 0.73 > mu)."""
        h = Heli()
        h.set_altitude(0.05)
        h.vel = [0.0, 2.0, 0.5]
        h.on_ground = False
        self.run_ground(h, 3.0)
        slide = h.pos[1]
        self.assertLess(abs(h.vel[1]), 0.01)
        self.assertTrue(0.5 * 4 / (2 * h.skid_mu * G) < slide < 2.0 * 4 / (2 * h.skid_mu * G), slide)
        r, p, _ = euler(h.q)
        self.assertLess(abs(r), math.radians(2))

    def test_tip_over_angle(self):
        """Statics: tipping angle atan(skid_y / cg_height) = 36 deg. From 30 deg it falls back
        on its skids, from 45 deg it falls on its side."""
        tip = math.degrees(math.atan2(Heli().skid_y, Heli().cg_height))
        self.assertAlmostEqual(tip, 36.3, delta=0.5)
        h = self.run_ground(self.tilted(30.0), 4.0)
        self.assertLess(abs(math.degrees(euler(h.q)[0])), 1.0)
        h = self.run_ground(self.tilted(45.0), 4.0)
        self.assertGreater(abs(math.degrees(euler(h.q)[0])), 60.0)

    def test_stiction(self):
        """A steady sideways push below mu g is held without creeping (the friction sticks);
        above it the heli slides."""
        for push, slides in ((0.3 * G, False), (0.8 * G, True)):
            h = Heli()
            for _ in range(int(3.0 / DT)):
                h.step(DT, 0.0, 0, 0, 0, 0.0)
                h.vel[0] += push * DT
            if slides:
                self.assertGreater(h.pos[0], 1.0)
            else:
                self.assertLess(abs(h.pos[0]), 0.01)

    def test_yaw_friction(self):
        h = Heli()
        h.rates = [0.0, 0.0, 3.0]
        self.run_ground(h, 2.0)
        self.assertLess(abs(h.rates[2]), 0.01)

    def on_slope(self, slope, uphill=0.0, yaw=0.0):
        h = Heli()
        h.q = (math.cos(math.radians(yaw) / 2), 0.0, 0.0, math.sin(math.radians(yaw) / 2))
        h.set_ground_slope(slope, uphill)
        h.place_on_ground()
        return h

    def test_rests_on_a_slope(self):
        """Placed on a 15 deg slope (tan 0.27 < mu), facing uphill and across it: it stays
        put, tilted with the ground - a little more: the downhill skid carries more of the
        weight and the springs are soft (1 cm static deflection), about 3 deg here."""
        for yaw, axis in ((0.0, 1), (90.0, 0)):
            with self.subTest(yaw=yaw):
                h = self.on_slope(15.0, 0.0, yaw)
                x0 = list(h.pos)
                self.run_ground(h, 3.0)
                self.assertLess(math.dist(h.pos, x0), 0.01)
                self.assertTrue(15.0 <= abs(math.degrees(euler(h.q)[axis])) <= 19.0, euler(h.q))
                self.assertTrue(h.on_ground)

    def test_slides_down_a_steep_slope(self):
        """35 deg (tan 0.70 > mu 0.6): it slides downhill at about g (sin - mu cos)."""
        h = self.on_slope(35.0)
        self.run_ground(h, 1.0)
        s, c = math.sin(math.radians(35)), math.cos(math.radians(35))
        expect = 0.5 * G * (s - h.skid_mu * c)               # m along the slope in 1 s
        self.assertAlmostEqual(-h.pos[0] / c, expect, delta=0.3 * expect)
        self.assertAlmostEqual(math.degrees(euler(h.q)[1]), 35.0, delta=1.0)

    def test_lands_level_onto_a_slope(self):
        """Dropped level from 0.2 m onto a 10 deg slope across the skids, it settles tilted
        with the ground (plus the soft skids' share), upright, without sliding off."""
        h = Heli()
        h.set_ground_slope(10.0, 90.0)
        h.set_altitude(0.2)
        h.on_ground, h.air_time = False, 1.0
        self.run_ground(h, 3.0)
        r, p, _ = euler(h.q)
        self.assertTrue(10.0 <= abs(math.degrees(r)) <= 13.0, r)
        self.assertLess(abs(math.degrees(p)), 1.0)
        self.assertLess(max(abs(v) for v in h.vel), 0.01)
        self.assertLess(math.hypot(h.pos[0], h.pos[1]), 0.1)


class WindModel(unittest.TestCase):

    def test_dryden_intensity(self):
        """Each component's std comes out at the MIL-F-8785C intensity for the height."""
        w = Wind(0.0, 0.0, 1.0, seed=3)
        xs = [w.step(DT, 10.0, 5.0) for _ in range(int(300 / DT))]
        (su, sw), _ = w.scales(10.0)
        for i, want in ((0, su), (1, su), (2, sw)):
            m = sum(x[i] for x in xs) / len(xs)
            std = math.sqrt(sum((x[i] - m) ** 2 for x in xs) / len(xs))
            self.assertAlmostEqual(std / want, 1.0, delta=0.15)

    def test_steady_wind_direction(self):
        self.assertEqual(tuple(round(c, 6) for c in Wind(5.0, 90.0).step(DT, 10.0, 0.0)), (-0.0, -5.0, 0.0))

    def test_air_relative(self):
        """Hovering in a 5 m/s wind is flying 5 m/s through still air: the same thrust."""
        def thrust(vel, wind):
            h = M7R()
            h.ground, h.on_ground = False, False
            h.pos = [0.0, 0.0, -100.0]
            h.w, h.n = h.nominal_w, 1.0
            h.wind_e = wind
            for _ in range(int(2 / DT)):
                h.vel, h.rates = list(vel), [0.0, 0.0, 0.0]
                h.step(DT, 0.6, 0, 0, 0, h.coll_hover)
                h.w = h.nominal_w
            return h.n
        self.assertAlmostEqual(thrust((5.0, 0, 0), (0, 0, 0)), thrust((0, 0, 0), (-5.0, 0, 0)), places=6)

    def test_drag_slows(self):
        h = Heli()
        h.ground, h.on_ground = False, False
        h.pos = [0.0, 0.0, -100.0]
        h.w, h.n = h.nominal_w, 1.0
        h.vel = [15.0, 0.0, 0.0]
        for _ in range(int(3 / DT)):
            h.step(DT, 0.65, 0, 0, 0, h.coll_hover)
            h.rates, h.q = [0.0, 0.0, 0.0], (1.0, 0.0, 0.0, 0.0)
        self.assertLess(h.vel[0], 13.0)


class Actuators(unittest.TestCase):

    def test_servo_slew_time(self):
        """A 300 us step on a 0.06 s/60 deg servo (10 us/deg) takes 30 ms at full speed,
        plus its lag."""
        sv = Servo(*TYPES['digital'])
        sv.step(DT, 1500.0)
        t = 0.0
        while sv.step(DT, 1800.0) < 1799.0:
            t += DT
        self.assertTrue(0.030 <= t <= 0.060, t)

    def test_servo_deadband(self):
        sv = Servo(*TYPES['analog'])          # 4 us deadband
        sv.step(DT, 1500.0)
        for _ in range(400):
            pos = sv.step(DT, 1503.0)
        self.assertAlmostEqual(pos, 1500.0)
        for _ in range(2000):
            pos = sv.step(DT, 1510.0)
        self.assertAlmostEqual(pos, 1510.0, delta=0.01)

    def test_ideal_and_unused(self):
        self.assertEqual(Servos('ideal').step(DT, (1500.0, 0.0)), (1500.0, 0.0))
        out = Servos('digital').step(DT, (1500.0,) + (0.0,) * 7)
        self.assertEqual(out[1], 0.0)

    def test_tail_servo_type(self):
        """--tail-servo: output 3 gets its own type, the cyclic servos keep theirs."""
        def after(servos, ms):
            servos.step(DT, (1500.0,) * 8)
            for _ in range(int(ms / 1000 / DT)):
                out = servos.step(DT, (1800.0,) * 8)
            return out
        out = after(Servos('ideal', tail='analog'), 20)
        self.assertEqual(out[0], 1800.0)                 # cyclic: ideal
        self.assertLess(out[3], 1700.0)                  # tail: analog, still on its way
        out = after(Servos('fast', tail='analog'), 20)
        self.assertGreater(out[0], out[3])               # the fast cyclic servo is ahead
        self.assertEqual(after(Servos('digital'), 20), after(Servos('digital', tail='digital'), 20))

    def test_pack_curve(self):
        self.assertEqual(LIPO_OCV[-1], 4.20)
        for a, b in zip(LIPO_OCV, LIPO_OCV[1:]):
            self.assertGreater(b, a)
        h = M7R()
        self.assertAlmostEqual(h.pack_voltage(0.0), 12 * 4.20)
        h.used_ah = 0.5 * h.pack_ah
        self.assertAlmostEqual(h.pack_voltage(0.0) / 12, 3.84, delta=0.01)
        # the sag under load grows towards empty
        sag_half = h.pack_voltage(0.0) - h.pack_voltage(40.0)
        h.used_ah = 0.95 * h.pack_ah
        self.assertGreater(h.pack_voltage(0.0) - h.pack_voltage(40.0), sag_half)


class TailDynamics(unittest.TestCase):
    """The servo tail's thrust lag and yaw mode (Heli.tail_moment): off on the base model,
    unit gain at rest, and the gain and phase of first order x second order at 12 Hz -
    the lag that lets the M7R's 1500 rpm yaw gains wag the tail at 2000 rpm."""

    @staticmethod
    def response(heli, f, seconds=2.0):
        """Gain and phase (deg) of tail_moment for a sine at f Hz, over the last second."""
        heli.reset()
        n = int(seconds / DT)
        last = int(1.0 / DT)
        c = s_ = 0.0
        for i in range(n):
            t = i * DT
            y = heli.tail_moment(DT, math.sin(2 * math.pi * f * t))
            if i >= n - last:
                c += y * math.cos(2 * math.pi * f * t)
                s_ += y * math.sin(2 * math.pi * f * t)
        k = 2.0 / last
        return math.hypot(c * k, s_ * k), math.degrees(math.atan2(c * k, s_ * k))

    def test_off_on_the_base_model(self):
        heli = Heli()
        heli.reset()
        self.assertEqual(heli.tail_moment(DT, 0.7), 0.7)

    def test_unit_gain_at_rest(self):
        heli = M7R()
        heli.reset()
        for _ in range(int(1.0 / DT)):
            y = heli.tail_moment(DT, 1.0)
        self.assertAlmostEqual(y, 1.0, places=3)

    def test_m7r_at_12hz(self):
        heli = M7R()
        gain, phase = self.response(heli, 12.0)
        s = 2j * math.pi * 12.0
        wn = 2 * math.pi * heli.yaw_mode_hz
        h = 1 / (1 + s * heli.tail_lag) * wn ** 2 / (s * s + 2 * heli.yaw_mode_zeta * wn * s + wn * wn)
        self.assertAlmostEqual(gain, abs(h), delta=0.03 * abs(h))
        self.assertAlmostEqual(phase, math.degrees(math.atan2(h.imag, h.real)), delta=3.0)   # Euler steps
        self.assertLess(phase, -45.0)          # the lag the first-order yaw axis was missing


class PowerAndActuators(unittest.TestCase):
    """ESC lag, pack temperature, servo load and the BLHeli32 telemetry frame."""

    def test_esc_lag(self):
        heli = M7R()
        heli.reset()
        self.assertAlmostEqual(heli.esc_tau, 0.065)
        heli.step(DT, 0.0, 0, 0, 0, 0)
        n = int(heli.esc_tau / DT)
        for _ in range(n):
            heli.step(DT, 1.0, 0, 0, 0, 0)
        self.assertAlmostEqual(heli.esc_u, 1 - math.exp(-1), delta=0.01)   # one time constant
        base = Heli()
        base.reset()
        base.step(DT, 0.5, 0, 0, 0, 0)
        self.assertEqual(base.esc_u, 0.0)                                   # off by default

    def test_pack_temperature(self):
        warm, cold = M7R(), M7R(pack_temp=0.0)
        self.assertAlmostEqual(warm.pack_voltage(40.0), M7R(pack_temp=25.0).pack_voltage(40.0))
        self.assertLess(cold.pack_voltage(40.0), warm.pack_voltage(40.0) - 1.0)    # sags in the cold
        self.assertLess(cold.pack_capacity(), warm.pack_capacity())
        t0 = cold.pack_temp
        cold.reset()
        cold.w = cold.nominal_w
        for _ in range(int(5.0 / DT)):
            cold.step(DT, 0.8, 0, 0, 0, 0.3)
        self.assertGreater(cold.pack_temp, t0)                                     # I^2 R warms it
        self.assertEqual(warm.pack_temp, 25.0)                                     # no model: fixed

    def test_servo_load(self):
        def reach(load, rr=1.0):
            s = Servo(*TYPES['digital'], load=load)
            s.step(DT, 1500.0, rr)
            for _ in range(int(0.3 / DT)):
                p = s.step(DT, 2000.0, rr)
            return p
        self.assertAlmostEqual(reach(0.0), 2000.0, delta=1.0)
        self.assertAlmostEqual(reach(0.9), 2000.0, delta=1.0)        # slower, still gets there
        self.assertAlmostEqual(reach(1.5), 1500 + 500 / 1.5, delta=5.0)   # stalls where L = 1
        self.assertAlmostEqual(reach(1.5, rr=0.5), 2000.0, delta=1.0)     # half the load at lower rpm
        with self.assertRaises(SystemExit):
            Servos('ideal', load=0.5)                                # an ideal servo has no motor

    def test_blheli32_frame(self):
        from esc import Blh32Esc, crc8_kiss
        self.assertEqual(crc8_kiss(b'123456789'), 0xF4)              # CRC-8 (poly 0x07) check value
        f = Blh32Esc(None).frame(123400, 48.5, 31.25)
        self.assertEqual(len(f), 10)
        self.assertEqual(int.from_bytes(f[1:3], 'big'), 4850)        # 10 mV
        self.assertEqual(int.from_bytes(f[3:5], 'big'), 3125)        # 10 mA
        self.assertEqual(int.from_bytes(f[7:9], 'big'), 1234)        # eRPM / 100
        self.assertEqual(f[9], crc8_kiss(f[:9]))


class GroundResonance(unittest.TestCase):
    """The lag mode (lag_nu) on the skids: a four-blade rotor whose regressive lag frequency
    meets the stiff skid-contact modes (~20 Hz on the Blade 360 model) whirls up with little
    lag damping, not with dampers, not in the air; off by default."""

    @staticmethod
    def lag_after(nu, zeta, blades=4, ground=True, seconds=1.0):
        h = Heli()
        h.blades, h.lag_zeta = blades, zeta
        h.reset()
        h.w = h.nominal_w
        if ground:
            for _ in range(int(0.3 / DT)):
                h.step(DT, 0.65, 0, 0, 0, -0.3)
        else:
            h.ground = h.on_ground = False
            h.pos = [0.0, 0.0, -100.0]
        h.lag_nu = nu
        h.lag = [0.002j, 0j]
        for _ in range(int(seconds / DT)):
            h.step(DT, 0.65, 0, 0, 0, -0.3 if ground else h.coll_hover)
        return abs(h.lag[0])

    def test_whirls_up_on_the_ground(self):
        self.assertGreater(self.lag_after(0.4, 0.01), 10 * 0.002)

    def test_lag_dampers_cure_it(self):
        self.assertLess(self.lag_after(0.4, 0.3), 0.002)

    def test_not_in_the_air(self):
        self.assertLess(self.lag_after(0.4, 0.01, ground=False), 0.002)

    def test_off_by_default(self):
        self.assertEqual(Heli.lag_nu, 0.0)


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
        # with every harmonic on the default exponent 2 (the floor always is)
        from unittest import mock
        import sensors
        with mock.patch.object(sensors, 'rpm_exponents', lambda h: [2.0, 2.0, 2.0]):
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

    def test_vibration_rpm_exponents(self):
        # a harmonic's own exponent where the logs pin it, clamped to 0..4, else 2
        from sensors import rpm_exponents
        h = {'rpm_exp': [1.2, 7.5, -3.0], 'rpm_exp_se': [0.2, 0.3, 0.4]}
        self.assertEqual(rpm_exponents(h), [1.2, 4.0, 0.0])
        h = {'rpm_exp': [1.2, 3.0, 0.5], 'rpm_exp_se': [0.9, None, 0.5]}
        self.assertEqual(rpm_exponents(h), [2.0, 2.0, 0.5])
        self.assertEqual(rpm_exponents({'rpm_exp': [5.0, 5.0, 5.0]}), [2.0, 2.0, 2.0])   # older files


class ModelFiles(unittest.TestCase):
    """--model takes a preset name or a JSON parameter file on top of a preset."""

    def write(self, d):
        import json, tempfile
        f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump(d, f)
        f.close()
        self.addCleanup(os.remove, f.name)
        return f.name

    def test_preset_name(self):
        self.assertIsInstance(load_model('m7r'), M7R)

    def test_file_matches_preset_with_the_same_arguments(self):
        h = load_model(self.write({'base': 'm7r', 'args': {'Lb1s': 900.0, 'pack_cells': 6}}))
        ref = M7R(Lb1s=900.0, pack_cells=6)
        self.assertIsInstance(h, M7R)
        self.assertEqual((h.Lb1s, h.cells, h.nominal_w), (ref.Lb1s, ref.cells, ref.nominal_w))

    def test_constants_and_keywords(self):
        h = load_model(self.write({'base': 'm7r', 'args': {'Lb1s': 900.0}, 'attrs': {'v_hover': 2.5}}),
                       Lb1s=800.0)
        self.assertEqual((h.v_hover, h.Lb1s), (2.5, 800.0))     # keywords win over the file
        self.assertEqual(M7R.v_hover, 3.3)                      # the preset is untouched

    def test_typos_are_refused(self):
        with self.assertRaises(ValueError):
            load_model(self.write({'base': 'm7r', 'attrs': {'v_hoover': 2.5}}))
        with self.assertRaises(ValueError):
            load_model(self.write({'bse': 'm7r'}))
        for bad in ([1, 2], {'base': 'm9'}, {'base': 'm7r', 'args': [1]}, {'base': 'm7r', 'attrs': 3}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                load_model(self.write(bad))

    def test_presets_are_fitted_parameter_files(self):
        # the fitted presets' numbers live in heli/presets/*.json, on the generic Fitted class
        for cls in (Stratos200, M7R):
            with self.subTest(model=cls.__name__):
                self.assertTrue(issubclass(cls, Fitted))
                self.assertEqual(cls.params['base'], 'fitted')
                self.assertIn('fitted', cls.__doc__)

    def test_fitted_base_reproduces_a_preset(self):
        # a trained model.json uses base "fitted" with all of its numbers: the M7R preset's
        # file contents on that base must give the M7R
        p = dict(M7R.params)
        h, ref = load_model(self.write(p)), M7R()
        self.assertNotIsInstance(h, M7R)
        for k in ('Lb1s', 'Ma1s', 'Lflt', 'Mflg', 'tf_inv_roll', 'torque_yaw', 'nominal_w', 'gear', 'cells',
                  'motor_fit', 'thrust_inflow', 'Nped', 'ctrl_rpm_exp', 'coll_hover'):
            self.assertEqual(getattr(h, k), getattr(ref, k), k)

    def test_motorized_tail_rpm(self):
        h = Stratos200()
        h.tail_throttle = 0.215
        self.assertAlmostEqual(h.tail_rpm, 14340.0)
        self.assertEqual(M7R().tail_rpm, 0.0)       # servo tail

    def test_unknown_name(self):
        with self.assertRaises(ValueError):
            load_model('no_such_heli')
        self.assertIsNone(resolve('no_such_heli'))
        self.assertEqual(resolve('m7r'), 'm7r')


class TrainedModels(unittest.TestCase):
    """Every trained heli (fitting/helis/<heli>/model.json, fitting/scripts/train.py) loads
    by name and is flyable: a hover throttle the motor can give, a damped flap/body mode in
    the range real helis have, and it holds its height in hover with the rpm held."""

    def test_trained_models(self):
        names = trained_models()
        for name in names:
            with self.subTest(heli=name):
                h = load_model(name)
                self.assertIsInstance(h, Fitted)
                # throttle that holds the nominal rotor speed at the hover collective
                w, c = h.nominal_w, h.coll_hover
                if h.motor_fit:
                    A, B = h.motor_fit[:2]
                    u = (B * w + w * w * h.drag_shape(c)) / (A * h.pack_voltage(0.0) * 0.95)
                else:
                    u = (h.B * w + h.k_d * w * w * h.drag_shape(c)) / h.A
                self.assertTrue(0.15 < u < 1.0, 'hover throttle %.2f' % u)
                f = mode_freq(load_model(name), dt=DT)
                self.assertIsNotNone(f)
                self.assertTrue(1.0 < f < 12.0, 'flap/body mode %.1f Hz' % f)
                # hover at the collective the thrust law needs (the Stratos preset's Zcol
                # says 0.636, its thrust law 0.6395: 1.3 m/s apart at its inflow share)
                h = airborne(load_model(name))
                if h.thrust_inflow:
                    K, ki = h.thrust_inflow
                    c = 3.0 / K + 1.5 * ki
                hold_rpm(h, int(2.0 / DT), DT, (0.5, 0.0, 0.0, 0.0, c / h.coll_scale))
                self.assertLess(abs(h.vel[2]), 0.3, 'climb or sink after 2 s of hover')


if __name__ == '__main__':
    unittest.main()
