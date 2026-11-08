"""
Synthetic flight logs: a heli of known parameters (heli_model.Fitted) flown by a simple
autopilot, logged with the blackbox fields the fits read. The training tests fit them and
check that the numbers come back - no private logs needed.

The autopilot: a governor on the throttle, a height hold on the collective (with small
pumps for the rotor law), and per axis a rate loop with feedforward under an angle loop
that keeps the heli level. A flight is a sequence of phases per headspeed: calm hover
(the hover and thrust stages), band-limited stick noise (rate gains, yaw) and stick
doublets (sharp inputs for the flap fit).
"""
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'heli'))
from heli_model import euler, from_params  # noqa: E402

D2R = math.pi / 180

# a 700-class servo-tail heli whose numbers differ from every preset and size prior
TRUTH = {
    'base': 'fitted',
    'attrs': {'motor_fit': [3.2, 0.36, 4.5e-4, 6.0e-4, 2.0], 'motor_poles': 10, 'v_hover': 3.3,
              'rotor_radius': 0.79, 'hub_height': 0.4, 'ctrl_rpm_exp': 0.0,
              'Zcol': -9.80665 / 0.30, 'thrust_inflow': [3.0 / 0.09, 0.21 / 1.5], 'Nped': 48.0, 'Nr': -1.5},
    'args': {'rate_gain_roll': 6.0, 'rate_gain_pitch': 5.2, 'Lb1s': 900.0, 'Ma1s': 600.0, 'tf': 0.05,
             'tf_roll': 0.045, 'tf_pitch': 0.06, 'stiff_rpm_exp': 2.0, 'tf_rpm_exp': 1.0, 'yaw_trim': 0.18,
             'nominal_rpm': 1800.0, 'gear_ratio': 8.75, 'hover_throttle': 0.6, 'pack_cells': 12,
             'pack_ah': 4.5, 'pack_r': 0.047, 'hover_power_w': 1100.0, 'cyclic_scale': 1.0, 'yaw_scale': 1.0},
}


class SynthLog:
    """The interface of common.Log over arrays."""

    def __init__(self, name, cols, hdr, events):
        self.name = name
        self.N = list(cols)
        self.a = np.column_stack([np.asarray(cols[n], float) for n in self.N])
        self.hdr = hdr
        self.events = events
        self.t = (self.c('time') - self.c('time')[0]) / 1e6
        self.dt = float(np.median(np.diff(self.t)))

    def c(self, n):
        return self.a[:, self.N.index(n)].astype(float)


def band_noise(rng, n, dt, hz, amp):
    """Smooth random signal: white noise through two first-order low-passes at hz."""
    a = dt * 2 * math.pi * hz
    x = y = 0.0
    out = np.empty(n)
    sc = amp * math.sqrt(2 / a) * 1.6
    for i in range(n):
        x += (rng.gauss(0, sc) - x) * a
        y += (x - y) * a
        out[i] = y
    return out


def fly(params=TRUTH, name='synth', rpms=(1800,), seed=1, hover_s=14.0, noise_s=30.0, doublets=8,
        craft='Synth 700', baro=True, dt=0.001, log_every=2):
    """One flight; returns a SynthLog. Each headspeed: calm hover, stick noise, doublets
    (alternating roll and pitch), then calm again."""
    rng = random.Random(seed)
    h = from_params(params)
    h.reset()
    h.ground = False
    h.on_ground = False
    h.set_altitude(10.0)
    h.w = h.nominal_w * rpms[0] / params['args']['nominal_rpm']
    K = (params['args']['rate_gain_roll'], params['args']['rate_gain_pitch'])
    N = params['args']['nominal_rpm']

    # phase plan: (seconds, kind, rpm)
    plan = []
    for rpm in rpms:
        plan += [(3.0, 'calm', rpm), (hover_s, 'calm', rpm), (noise_s, 'noise', rpm),
                 (doublets * 1.6, 'doublets', rpm), (3.0, 'calm', rpm)]
    total = sum(p[0] for p in plan)
    n = int(total / dt)
    noise = [band_noise(rng, n, dt, 2.5, 50 * D2R), band_noise(rng, n, dt, 2.5, 50 * D2R),
             band_noise(rng, n, dt, 2.0, 40 * D2R)]
    coll_pump = band_noise(rng, n, dt, 0.8, 0.04)

    cols = {k: [] for k in ['time'] + ['setpoint[%d]' % i for i in range(4)] + ['mixer[%d]' % i for i in range(4)]
            + ['gyroADC[%d]' % i for i in range(3)] + ['attitude[%d]' % i for i in range(3)]
            + ['rcCommand[%d]' % i for i in range(5)]
            + ['altitude', 'vario', 'headspeed', 'govTarget', 'motor[0]', 'Vbat', 'Ibat']}
    if not baro:
        del cols['altitude'], cols['vario']
    i_rate = [0.0, 0.0, 0.0]
    i_gov = 0.0
    i_alt = 0.0
    t = 0.0
    k = 0
    for secs, kind, rpm in plan:
        t_end = t + secs
        t0 = t
        while t < t_end - 1e-9 and k < n:
            roll, pitch, yaw = euler(h.q)
            p, q, r = h.rates
            wr = rpm / N
            # angle loop keeps the heli level; the excitation rides on it
            sp = [-3.0 * roll, -3.0 * pitch, 0.0]
            if kind == 'noise':
                sp = [sp[0] + noise[0][k], sp[1] + noise[1][k], noise[2][k]]
            elif kind == 'doublets':
                j = int((t - t0) / 1.6)
                ph = (t - t0) - 1.6 * j
                amp = 220 * D2R * (1 if j % 4 < 2 else -1)
                v = amp if ph < 0.3 else -amp if ph < 0.6 else 0.0
                sp[j % 2] += v
            # rate loops: feedforward through the model's gain at this rpm, PI on the error
            u = []
            for ax in range(2):
                kk = K[ax] * wr
                e = sp[ax] - h.rates[ax]
                i_rate[ax] += e * dt
                u.append(sp[ax] / kk + 0.25 / kk * e + 1.0 / kk * i_rate[ax])
            e = sp[2] - r
            i_rate[2] += e * dt
            nped = params['attrs']['Nped'] * wr * wr
            yaw_in = (0.15 * e + 0.8 * i_rate[2]) * 50.0 / nped + sp[2] * 0.05
            # governor and height hold
            hs = h.rotor_rpm
            ge = (rpm - hs) / rpm
            i_gov += ge * dt
            thr = min(1.0, max(0.0, 0.55 + 2.0 * ge + 3.0 * i_gov))
            ae = 10.0 - h.altitude
            i_alt += ae * dt
            # collective pumps for the rotor law in the stick-noise phases only: a pumping
            # hover climbs and sinks, and the inflow makes its median collective no trim
            pump = coll_pump[k] if kind == 'noise' else 0.0
            coll = 0.30 / wr ** 1.6 + 0.04 * ae + 0.06 * h.vel[2] + 0.01 * i_alt + pump
            h.step(dt, thr, u[0], u[1], yaw_in, coll)
            if k % log_every == 0:
                cols['time'].append(t * 1e6)
                for ax in range(3):
                    cols['setpoint[%d]' % ax].append(sp[ax] / D2R + rng.gauss(0, 0.2))
                    cols['gyroADC[%d]' % ax].append(h.rates[ax] / D2R + rng.gauss(0, 0.5))
                cols['setpoint[3]'].append(coll * 1000)
                for ax, v in enumerate((u[0], u[1], yaw_in, coll)):
                    cols['mixer[%d]' % ax].append(v * 1000)
                for ax, v in enumerate((roll, pitch, yaw)):
                    cols['attitude[%d]' % ax].append(v / D2R * 10)
                for ax, v in enumerate((sp[0] / D2R, sp[1] / D2R, sp[2] / D2R, coll * 500, thr * 1000)):
                    cols['rcCommand[%d]' % ax].append(v)
                if baro:
                    # the baro reads zero on the ground before take-off (the AIRBORNE event)
                    cols['altitude'].append(h.altitude * 100 if t >= 0.5 else 0.0)
                    cols['vario'].append(-h.vel[2] * 100)
                cols['headspeed'].append(hs)
                cols['govTarget'].append(rpm)
                cols['motor[0]'].append(thr * 1000)
                cols['Vbat'].append(h.vbat * 100)
                cols['Ibat'].append(h.ibat * 100)
            t += dt
            k += 1
    hdr = {'Craft name': craft, 'Firmware revision': 'Rotorflight 4.6.0 (synthetic)', 'Board information': 'SYNTH',
           'collectiveRange': '-1250,1250', 'motor_pwm_protocol': '0', 'dshot_bidir': '0',
           'Log start datetime': '2026-11-08T12:00:%02d.000+00:00' % (seed % 60), 'rollPID': '50,100,20,100,0'}
    events = [(0.5e6, 52, [1])]
    return SynthLog(name, cols, hdr, events)
