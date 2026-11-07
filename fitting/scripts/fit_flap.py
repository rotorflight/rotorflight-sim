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
    K, L, tf, d = params
    t = np.arange(len(u)) * dt
    ud = np.interp(t - d, t, u)                 # fractional delay, held at the start
    p, b = p0, pd0 / L
    out = np.empty(len(u))
    for i in range(len(u)):
        out[i] = p
        bd = -p - b / tf + K * ud[i]
        p += L * b * dt
        b += bd * dt
    return out


def fit(windows, dt):
    def res(x):
        K, lnL, lntf, d = x
        r = []
        for u, y in windows:
            pd0 = (y[3] - y[0]) / (3 * dt)
            r.append(simulate((K, np.exp(lnL), np.exp(lntf), d), u, y[0], pd0, dt) - y)
        return np.concatenate(r)
    best = None
    for L0 in (200.0, 700.0, 2000.0):
        for tf0, K0 in ((0.03, 6.0), (0.08, 6.0), (0.03, -6.0), (0.08, -6.0)):
            r = least_squares(res, [K0, np.log(L0), np.log(tf0), 0.02],
                              bounds=([-30, np.log(20), np.log(0.005), 0.0], [30, np.log(2e4), np.log(0.5), 0.06]))
            if best is None or r.cost < best.cost:
                best = r
    K, lnL, lntf, d = best.x
    y = np.concatenate([w[1] for w in windows])
    vaf = 1 - np.var(best.fun) / np.var(y)
    return {'K': round(float(K), 3), 'L': round(float(np.exp(lnL)), 1), 'tf': round(float(np.exp(lntf)), 4),
            'delay_s': round(float(d), 4), 'vaf': round(float(vaf), 3), 'events': len(windows),
            'mode_hz': round(float(np.sqrt(max(0.0, np.exp(lnL) - 1 / (4 * np.exp(lntf) ** 2))) / (2 * np.pi)), 2)}


def main():
    heli = json.load(open(sys.argv[1]))
    plot = sys.argv[sys.argv.index('--plot') + 1] if '--plot' in sys.argv else None
    per = {}          # (axis, headspeed) -> windows
    dt = None
    for entry in heli['logs']:
        name = os.path.splitext(entry['file'])[0]
        try:
            log = Log(name)
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
            continue
        fs = 1.0 / log.dt
        dt = max(1, int(round(fs / RATE))) / fs
        lab, _, _ = segment(log)
        air = ~lab['ground'] & ~lab['autorotation']
        tg = np.round(log.c('govTarget') / 100) * 100
        for ax in (0, 1):
            u = log.c('mixer[%d]' % ax) / 1000
            y = log.c('gyroADC[%d]' % ax) * D2R
            for i in events(log, ax, air):
                sl = slice(i - int(PRE * fs), i + int(POST * fs))
                lv = int(np.median(tg[sl]))
                # 200 Hz is plenty for dynamics below 10 Hz, and keeps the fit fast
                step = max(1, int(round(fs / RATE)))
                per.setdefault((ax, lv), []).append((u[sl][::step], y[sl][::step]))
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
