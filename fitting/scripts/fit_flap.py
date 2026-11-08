"""
Flap/body dynamics from the sharp stick inputs in a heli's logs: control power, flap
stiffness and flap time constant per axis and headspeed.

    python -I scripts/fit_flap.py helis/<heli>/heli.json [--plot out.png]

The model's roll (pitch alike) is first-order flapping on a rigid body:

    p' = L b,    b' = -p - b / tf + K u(t - d)

u the mixer's cyclic input (what reaches the rotor, also in closed loop; logged in 0.012
deg of blade pitch, used here per 1000 counts = 12 deg, as everywhere in the fits), p the
gyro rate,
K the steady rate per unit input (rad/s; heli_model's Lflt / tf), L the flap stiffness
(1/s^2, Lb1s / Ma1s in heli_model), tf the flap time constant (s), d a delay (servos, filters, loop). Its mode:
s^2 + s / tf + L = 0. Each sharp input (setpoint swinging by > 150 deg/s within 0.3 s)
gives a window from 0.3 s before to 1.2 s after; the model is simulated from the measured
initial rate (b from the initial rate acceleration) through each window with the logged
input, and K, L, tf, d fitted by output error over all windows of one axis and headspeed.
VAF (variance accounted for) says how much of the rate the fit explains.

Writes helis/<heli>/flap.json.
"""
import json
import os
import sys

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402
from segment import segment  # noqa: E402

D2R = np.pi / 180
PRE, POST = 0.3, 1.2
RATE = 200.0


def events(log, ax, air):
    fs = 1.0 / log.dt
    n = max(1, int(0.3 * fs))
    sp = log.c('setpoint[%d]' % ax)
    out, i = [], int(PRE * fs)
    while i + n < len(sp) - int(POST * fs):
        if abs(sp[i + n] - sp[i]) > 150 and air[i - int(PRE * fs):i + int(POST * fs)].all():
            out.append(i)
            i += int(1.5 * fs)
        else:
            i += max(1, n // 2)
    return out


def simulate(params, u, p0, pd0, dt):
    """The rate through a window, exact for the continuous model with the input held over
    each step (zero-order hold): explicit Euler at 200 Hz damps a 4-6 Hz mode wrongly
    and biased tf short."""
    K, L, tf, d = params
    t = np.arange(len(u)) * dt
    # fractional delay, held at the start; each step holds the input at its middle (a
    # zero-order hold on the start value would lag half a step, 2.5 ms at 200 Hz)
    ud = np.interp(t + 0.5 * dt - d, t, u)
    M = np.zeros((3, 3))
    M[0, 1], M[1, 0], M[1, 1], M[1, 2] = L, -1.0, -1.0 / tf, K
    E = expm(M * dt)
    a00, a01, a10, a11, b0, b1 = E[0, 0], E[0, 1], E[1, 0], E[1, 1], E[0, 2], E[1, 2]
    p, b = p0, pd0 / L
    out = np.empty(len(u))
    for i in range(len(u)):
        out[i] = p
        p, b = a00 * p + a01 * b + b0 * ud[i], a10 * p + a11 * b + b1 * ud[i]
    return out


def fit(windows, dt, x0=None):
    """K, L, tf, delay by output error over the windows; x0 (a previous fit's dict) starts
    a single local fit from there instead of the multi-start search (the jackknife)."""
    def res(x):
        K, lnL, lntf, d = x
        r = []
        for u, y in windows:
            pd0 = (y[3] - y[0]) / (3 * dt)
            r.append(simulate((K, np.exp(lnL), np.exp(lntf), d), u, y[0], pd0, dt) - y)
        return np.concatenate(r)
    best = None
    starts = [(K0, np.log(L0), np.log(tf0), 0.02) for L0 in (200.0, 700.0, 2000.0)
              for tf0, K0 in ((0.03, 6.0), (0.08, 6.0), (0.03, -6.0), (0.08, -6.0))]
    if x0 is not None:
        starts = [(x0['K'], np.log(x0['L']), np.log(x0['tf']), min(max(x0['delay_s'], 0.0), 0.06))]
    for st in starts:
        r = least_squares(res, list(st),
                          bounds=([-30, np.log(20), np.log(0.005), 0.0], [30, np.log(2e4), np.log(0.5), 0.06]))
        if best is None or r.cost < best.cost:
            best = r
    K, lnL, lntf, d = best.x
    y = np.concatenate([w[1] for w in windows])
    vaf = 1 - np.var(best.fun) / np.var(y)
    return {'K': round(float(K), 3), 'L': round(float(np.exp(lnL)), 1), 'tf': round(float(np.exp(lntf)), 4),
            'delay_s': round(float(d), 4), 'vaf': round(float(vaf), 3), 'events': len(windows),
            'mode_hz': round(float(np.sqrt(max(0.0, np.exp(lnL) - 1 / (4 * np.exp(lntf) ** 2))) / (2 * np.pi)), 2)}


def collect(logs):
    """The sharp-input windows of the logs, per (axis, headspeed level): lists of (u, y)
    at RATE Hz, and the window's (log index, start sample) in where[(axis, level)]."""
    from fit_heli import targets
    per, where = {}, {}
    dt = 1.0 / RATE
    # 200 Hz is plenty for dynamics below 10 Hz, and keeps the fit fast; every window is
    # resampled to it, so logs of different rates pool
    grid = np.arange(-PRE, POST, dt)
    for li, log in enumerate(logs):
        fs = 1.0 / log.dt
        lab, _, _ = segment(log)
        air = ~lab['ground'] & ~lab['autorotation']
        tg = np.round(targets(log) / 100) * 100
        for ax in (0, 1):
            u = log.c('mixer[%d]' % ax) / 1000
            y = log.c('gyroADC[%d]' % ax) * D2R
            for i in events(log, ax, air):
                sl = slice(i - int(PRE * fs), i + int(POST * fs))
                lv = int(np.median(tg[sl]))
                tt = log.t[i] + grid
                per.setdefault((ax, lv), []).append((np.interp(tt, log.t, u), np.interp(tt, log.t, y)))
                where.setdefault((ax, lv), []).append((li, i))
    return per, where, dt


def main():
    heli = json.load(open(sys.argv[1]))
    plot = sys.argv[sys.argv.index('--plot') + 1] if '--plot' in sys.argv else None
    logs = []
    for entry in heli['logs']:
        name = entry['file'][:-4] if entry['file'].endswith('.bbl') else entry['file']
        try:
            logs.append(Log(name))
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
    per, _, dt = collect(logs)
    out = {'heli': heli['name'], 'fits': []}
    print('axis   rpm  events      K      L     tf  delay   mode    VAF')
    for (ax, lv), wins in sorted(per.items()):
        if len(wins) < 3 or lv <= 0:
            continue
        r = fit(wins, dt)
        r.update(axis='roll' if ax == 0 else 'pitch', rpm=lv)
        out['fits'].append(r)
        print('%-5s %5d %7d %6.2f %6.0f %6.3f %5.0fms %5.1fHz %6.2f'
              % (r['axis'], lv, r['events'], r['K'], r['L'], r['tf'], 1000 * r['delay_s'], r['mode_hz'], r['vaf']))
    dst = os.path.join(os.path.dirname(sys.argv[1]), 'flap.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print('->', dst)
    if plot and out['fits']:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        keys = [(0 if f['axis'] == 'roll' else 1, f['rpm']) for f in out['fits']][:6]
        fig, axs = plt.subplots(len(keys), 1, figsize=(11, 2.6 * len(keys)), squeeze=False)
        for a, (key, f) in zip(axs[:, 0], zip(keys, out['fits'])):
            wins = per[key][:4]
            t0 = 0
            for u, y in wins:
                tt = t0 + np.arange(len(y)) * dt
                pd0 = (y[3] - y[0]) / (3 * dt)
                a.plot(tt, y / D2R, 'k', lw=0.8)
                a.plot(tt, simulate((f['K'], f['L'], f['tf'], f['delay_s']), u, y[0], pd0, dt) / D2R, 'r', lw=0.8)
                t0 = tt[-1] + 0.1
            a.set_title('%s %d rpm: K %.2f L %.0f tf %.3f (VAF %.2f) - black log, red fit'
                        % (f['axis'], f['rpm'], f['K'], f['L'], f['tf'], f['vaf']))
            a.set_ylabel('deg/s')
        plt.tight_layout()
        plt.savefig(plot, dpi=80)


if __name__ == '__main__':
    main()
