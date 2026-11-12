"""
Gyro and accelerometer errors for the SITL: chip noise and rotor-locked vibration.

The model's rates and specific force are exact; the firmware's filters (RPM filter,
notches, gyro and D-term low-passes) only get something to do when the sensor data
carries what a real heli's gyro sees:

  gyro = true rate + chip noise + vibration
  acc  = true specific force + chip noise

Chip noise (per axis, from an Allan deviation fit of a flight controller at rest,
`fitting/scripts/allan.py` -> `fitting/sensors/<chip>.json`; chips without a soak -
ICM42688P, ICM42605, BMI270, MPU6000 - carry their datasheet's white noise density alone,
"measured": false):
  white noise      density N (deg/s/rtHz): N / sqrt(dt) per sample
  bias             a first-order Gauss-Markov process with the bias instability B as its
                   std and the tau of the Allan minimum as its time constant
  rate walk        K (deg/s/rt s), integrated, if the fit found one
The accelerometer takes the white noise and bias the same way (the axis that carried 1 g
in the soak is quantisation-limited and replaced by the other two).

Vibration (per heli, order tracking of logged gyroRAW, `fitting/scripts/vibration.py`
-> `fitting/helis/<heli>/vibration.json`): for every harmonic of the main rotor, tail
rotor and motor, A_axis (rpm / rpm_ref)^k sin(order x rotor angle + phase_axis), locked
to the model's rotor angle so the firmware's RPM filter can track it; plus a white
broadband floor with the logged density above 100 Hz, x (rpm / rpm_ref)^2. k is the
harmonic's own fitted exponent where the logs pin it (rpm_exp_se <= 0.5, clamped 0..4),
else 2 (an out-of-balance force grows with the square of the speed): over eight helis
the well-pinned exponents scatter from 0 to 4 around ~1.9 - resonances, not one law.
No accelerometer vibration: the logs carry none.

Quantisation and range clipping happen in the SITL (16.4 LSB/deg/s, 256 LSB/g, int16).
Everything is seeded, so a check stays repeatable. Only the Python standard library.
"""

import json
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
FIT = os.path.join(HERE, '..', 'fitting')
D2R = math.pi / 180
G = 9.80665


def _resolve(name, folder, filename):
    """A path, or a name under fitting/<folder> (chip: <name>.json, heli: <name>/<filename>)."""
    if os.path.exists(name):
        return name
    for cand in (os.path.join(FIT, folder, name + '.json'), os.path.join(FIT, folder, name, filename)):
        if os.path.exists(cand):
            return cand
    raise SystemExit('no %s data for %r (looked in %s)' % (folder, name, os.path.join(FIT, folder)))


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def available(folder):
    d = os.path.join(FIT, folder)
    if not os.path.isdir(d):
        return []
    if folder == 'sensors':
        return sorted(f[:-5] for f in os.listdir(d) if f.endswith('.json'))
    return sorted(f for f in os.listdir(d) if os.path.exists(os.path.join(d, f, 'vibration.json')))


EXP_SE_MAX = 0.5          # a harmonic's own rpm exponent is used if pinned this well
EXP_RANGE = (0.0, 4.0)


def rpm_exponents(h):
    """Per axis, the rpm exponent of a harmonic's amplitude: its own fit where the logs pin
    it (standard error <= 0.5, vibration.py's rpm_exp_se), clamped to 0..4, else 2. Over
    eight helis the well-pinned exponents scatter from 0 to 4 (median 1.7: an amplitude
    rises or falls as its line moves towards or away from a frame resonance), so 2 - an
    out-of-balance force - stays the default."""
    out = []
    for i in range(3):
        se = (h.get('rpm_exp_se') or [None] * 3)[i]
        if se is not None and se <= EXP_SE_MAX:
            out.append(min(max(h['rpm_exp'][i], EXP_RANGE[0]), EXP_RANGE[1]))
        else:
            out.append(2.0)
    return out


class Sensors:
    def __init__(self, chip=None, vibration=None, seed=1, vib_scale=1.0, tail_ratio=None, gear_ratio=None):
        self.rng = random.Random(seed)
        self.chip = _load(_resolve(chip, 'sensors', '')) if chip else None
        self.vib = _load(_resolve(vibration, 'helis', 'vibration.json')) if vibration else None
        self.vib_scale = vib_scale
        self.gyro_bias = [0.0, 0.0, 0.0]       # rad/s
        self.gyro_walk = [0.0, 0.0, 0.0]
        self.acc_bias = [0.0, 0.0, 0.0]        # m/s^2
        self.theta = 0.0                       # main rotor angle, rad
        self.harm = []
        self.tail_ratio = tail_ratio
        if self.vib and not tail_ratio:
            self.tail_ratio = next((h['order'] for h in self.vib['harmonics']
                                    if h['source'] == 'tail' and h['n'] == 1), None)
        if self.vib:
            for h in self.vib['harmonics']:
                order = h['order']
                # a heli file can override the measured orders (another gear set on the same heli)
                if h['source'] == 'tail' and tail_ratio:
                    order = h['n'] * tail_ratio
                if h['source'] == 'motor' and gear_ratio:
                    order = h['n'] * gear_ratio
                phases = [self.rng.uniform(0, 2 * math.pi) for _ in range(3)]
                amps = [a * D2R * vib_scale for a in h['amp_dps']]
                self.harm.append((order, amps, phases, rpm_exponents(h)))
            self.rpm_ref = float(self.vib['rpm_ref'])
            self.floor = [a * D2R * vib_scale for a in self.vib.get('floor_asd_dps_rthz', [0, 0, 0])]
        if self.chip:
            # a datasheet chip (no soak) has white noise only: its bias terms are null
            per_axis = lambda v, d: [d if x is None else x for x in (v or [None] * 3)]
            g = self.chip['gyro']
            self.g_white = [n * D2R for n in g['white_dps_rthz']]
            self.g_bias = [b * D2R for b in per_axis(g.get('bias_instability_dps'), 0.0)]
            self.g_tau = [max(1.0, t) for t in per_axis(g.get('bias_tau_s'), 1.0)]
            self.g_walk = [k * D2R for k in per_axis(g.get('rate_walk_dps_rts'), 0.0)]
            a = self.chip['acc']
            white = list(a['white_g_rthz'])
            bias = per_axis(a.get('bias_instability_g'), 0.0)
            # the soak's 1 g axis sits on one count (no dither): take the other axes' median
            lo = sorted(white)[1]
            for i in range(3):
                if white[i] < 0.3 * lo:
                    white[i] = sorted(w for j, w in enumerate(white) if j != i)[0]
                    bias[i] = sorted(b for j, b in enumerate(bias) if j != i)[0]
            self.a_white = [w * G for w in white]
            self.a_bias = [b * G for b in bias]
            self.a_tau = [max(1.0, t) for t in per_axis(a.get('bias_tau_s'), 1.0)]
            # start inside the bias distribution, as a powered-up FC would
            self.gyro_bias = [self.rng.gauss(0, s) for s in self.g_bias]
            self.acc_bias = [self.rng.gauss(0, s) for s in self.a_bias]

    def describe(self):
        out = []
        if self.chip:
            out.append('gyro chip %s%s: white %s deg/s/rtHz, bias %s deg/h' % (
                self.chip['chip'], '' if self.chip.get('measured', True) else ' (datasheet)',
                self.chip['gyro']['white_dps_rthz'], self.chip['gyro'].get('bias_instability_deg_h') or '-'))
        if self.vib:
            out.append('vibration %s at %d rpm x%.2f: %d harmonics, floor %s deg/s/rtHz' % (
                self.vib['heli'], self.rpm_ref, self.vib_scale, len(self.harm), self.vib.get('floor_asd_dps_rthz')))
        return '; '.join(out) or 'ideal sensors'

    def _gm(self, x, sigma, tau, dt):
        # first-order Gauss-Markov with stationary std sigma
        a = math.exp(-dt / tau)
        return a * x + sigma * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)

    def step(self, dt, rates, force, rotor_w):
        """rates (rad/s, FRD), specific force (m/s^2), rotor speed (rad/s) -> measured."""
        g = list(rates)
        f = list(force)
        if self.chip:
            sq = 1.0 / math.sqrt(dt)
            for i in range(3):
                self.gyro_bias[i] = self._gm(self.gyro_bias[i], self.g_bias[i], self.g_tau[i], dt)
                if self.g_walk[i]:
                    self.gyro_walk[i] += self.g_walk[i] * math.sqrt(dt) * self.rng.gauss(0, 1)
                g[i] += self.gyro_bias[i] + self.gyro_walk[i] + self.g_white[i] * sq * self.rng.gauss(0, 1)
                self.acc_bias[i] = self._gm(self.acc_bias[i], self.a_bias[i], self.a_tau[i], dt)
                f[i] += self.acc_bias[i] + self.a_white[i] * sq * self.rng.gauss(0, 1)
        if self.vib:
            # not wrapped: the orders are not integers (an hour at 2000 rpm is ~8e5 rad,
            # still 1e-10 rad resolution in a double)
            self.theta += rotor_w * dt
            if rotor_w > 1.0:
                wr = rotor_w * 60 / (2 * math.pi) / self.rpm_ref
                s = wr * wr
                for order, amps, phases, expo in self.harm:
                    ang = order * self.theta
                    for i in range(3):
                        g[i] += wr ** expo[i] * amps[i] * math.sin(ang + phases[i])
                sq = s / math.sqrt(2 * dt)      # one-sided density over 0..1/(2 dt)
                for i in range(3):
                    g[i] += self.floor[i] * sq * self.rng.gauss(0, 1)
        return g, f
