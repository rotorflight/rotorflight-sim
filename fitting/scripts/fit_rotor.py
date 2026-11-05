"""
Fit the main rotor speed equation from the Stratos logs:

    dw/dt = A * u * V  -  B * w  -  w^2 * (k0 + k1 * |c|^n)

w headspeed (rad/s, direct drive), u main motor throttle 0..1, V pack volts,
c stabilized collective (mixer input, -1..1). A u V - B w is a DC motor (torque
falling with speed, per rotor inertia); the w^2 term is rotor drag rising with
collective. Linear least squares for a grid of n; derivative from a 5 Hz
low-passed headspeed. Fitted as u*V = (B w + w^2(k0 + k1|c|^n) + dw/dt) / A,
since under the governor dw/dt alone is mostly noise. Prints the fit per flight set and the values the
heli_model.Heli parameters correspond to.
"""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'scripts')
from common import Log, FLIGHTS

SPOOL = ['20261008_160146_04', '20261008_160146_07', '20261008_160146_08', '20261008_160146_09']


def samples(name):
    L = Log(name)
    fs = 1 / L.dt
    sos = butter(2, 5, fs=fs, output='sos')
    w = sosfiltfilt(sos, L.c('headspeed')) * 2 * np.pi / 60
    wd = np.gradient(w, L.dt)
    u = sosfiltfilt(sos, L.c('motor[0]') / 1000)
    V = sosfiltfilt(sos, L.c('Vbat') / 100)
    c = sosfiltfilt(sos, L.c('mixer[3]') / 1000)
    m = (w > 50) & (u > 0.05) & (V > 10)
    m[: int(fs)] = False
    m[-int(fs):] = False
    return dict(w=w[m], wd=wd[m], u=u[m], V=V[m], c=c[m], t=L.t[m], name=name)


def design(s, n):
    return np.column_stack([s['w'], s['w'] ** 2, s['w'] ** 2 * np.abs(s['c']) ** n, s['wd']])


def fit(ss, n):
    """Regress motor volts u*V on [w, w^2, w^2 |c|^n, dw/dt]; returns A, B, k0, k1 and R^2."""
    X = np.concatenate([design(s, n) for s in ss])
    y = np.concatenate([s['u'] * s['V'] for s in ss])
    q, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ q
    A = 1 / q[3]
    return (A, q[0] * A, q[1] * A, q[2] * A), 1 - r.var() / y.var()


def main():
    # fit on the six flights; the morning ground runs (SPOOL) only as a check, they sit
    # a constant ~1.2 V off (other conditions that session)
    ss = [samples(n) for n in FLIGHTS + SPOOL[:1]]
    fl = ss[:len(FLIGHTS)]
    best = max(((fit(fl, n), n) for n in (1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0)), key=lambda x: x[0][1])
    (A, B, k0, k1), r2 = best[0]
    n = best[1]
    print('fit over %d samples: A %.4f  B %.4f  k0 %.3e  k1 %.3e  n %.2f  (R^2 %.3f)'
          % (sum(len(s['w']) for s in fl), A, B, k0, k1, n, r2))
    for nn in (1.0, 1.5, 2.0):
        print('  n %.1f: R^2 %.3f' % (nn, fit(fl, nn)[1]))

    # what this means at the 3600 rpm hover
    hover = fl[2:]
    w_h = 3600 * 2 * np.pi / 60
    c = np.concatenate([s['c'] for s in hover]); u = np.concatenate([s['u'] for s in hover])
    V = np.concatenate([s['V'] for s in hover]); w = np.concatenate([s['w'] for s in hover])
    near = np.abs(w - w_h) < 5
    c_h, u_h, V_h = np.median(c[near]), np.median(u[near]), np.median(V[near])
    noload = A * u_h * V_h / B
    print('3600 rpm hover: collective %.3f, throttle %.3f at %.2f V; '
          'no-load speed at that throttle %.0f rpm -> load fraction %.2f'
          % (c_h, u_h, V_h, noload * 60 / 2 / np.pi, w_h / noload))
    print('  full-throttle no-load speed %.0f rpm at %.1f V (motor Kv*V/poles-ish)' % (A * V_h / B * 60 / 2 / np.pi, V_h))
    print('  spool torque/inertia at u=1, w=0: %.0f rad/s^2 -> %.2f s to 3600 rpm at full torque'
          % (A * V_h, w_h / (A * V_h)))
    print('  drag shape c0 = k0/k1 = %.3f  (heli_model uses 0.25 + |c|^1.5)' % (k0 / k1))

    # plot: measured vs fitted dw/dt on one flight with a punch, and a spool-up
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    for a_, s in zip(ax, [ss[3], ss[len(FLIGHTS)]]):
        A, B, k0, k1 = best[0][0]
        q = np.array([B / A, k0 / A, k1 / A, 1 / A])
        a_.plot(s['t'], s['u'] * s['V'], lw=0.6, label='measured motor volts u*V')
        a_.plot(s['t'], design(s, n) @ q, lw=0.6, label='fit')
        a_.set_title(s['name']); a_.set_ylabel('V'); a_.legend()
    ax[-1].set_xlabel('s')
    fig.tight_layout(); fig.savefig('rotor_fit.png', dpi=110)


main()
