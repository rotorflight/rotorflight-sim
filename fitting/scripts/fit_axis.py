"""
Per-axis plant identification (roll, pitch, yaw) from the closed-loop flights.

For each axis: G(f) = S_y,r / S_u,r with the axis' own setpoint r as instrument
(u = stabilized mixer input in model units, y = body rate FRD rad/s). Fitted with

    G(s) = K wn^2 / (s^2 + 2 zeta wn s + wn^2) * exp(-s tau)          (roll, pitch)
    G(s) = K / (T s + 1) * exp(-s tau)                                 (yaw)

over the band where the setpoint->rate coherence is >= 0.4. In heli_model terms a
second-order rate response is the single-axis flapping model:
    wn^2 = Lb1s, 2 zeta wn = 1/tf, K = Lflt / tf   (no cross flapping).
"""
import sys
import numpy as np
from scipy.signal import csd, welch, decimate
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'scripts')
from common import Log
from fit_cyclic import airborne_mask, FL3600, FL3300

D2R = np.pi / 180
FS = 250.0
NSEG = 1024
# (rate, input, setpoint, sign): FRD rate; model input dx = roll mixer, dy = -pitch mixer,
# yaw model input = -yaw mixer (sign_yaw = -1), r = -gyro yaw
AXES = {
    'roll':  ('gyroADC[0]', 'mixer[0]', 'setpoint[0]', +1),
    'pitch': ('gyroADC[1]', 'mixer[1]', 'setpoint[1]', -1),
    'yaw':   ('gyroADC[2]', 'mixer[2]', 'setpoint[2]', -1),
}


def axis_spectra(names, axis):
    gy, mx, sp, sg = AXES[axis]
    acc = {}
    for n in names:
        L = Log(n)
        q = int(round(1 / L.dt / FS))
        m = airborne_mask(L)
        y = sg * L.c(gy) * D2R
        u = sg * L.c(mx) / 1000
        r = sg * L.c(sp) * D2R
        edges = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            if (b - a) * L.dt < 2 * NSEG / FS:
                continue
            Y, U, R = (decimate(x[a:b], q, ftype='fir') for x in (y, u, r))
            for k, (x1, x2) in dict(yr=(Y, R), ur=(U, R), rr=(R, R), yy=(Y, Y)).items():
                f, s = csd(x1, x2, fs=FS, nperseg=NSEG)
                acc[k] = acc.get(k, 0) + s
    G = acc['yr'] / acc['ur']
    coh = np.abs(acc['yr']) ** 2 / (np.abs(acc['yy']) * np.abs(acc['rr']))
    return f, G, coh


def second_order(x, f):
    K, wn, z, tau = x
    s = 2j * np.pi * f
    return K * wn ** 2 / (s ** 2 + 2 * z * wn * s + wn ** 2) * np.exp(-s * tau)


def first_order(x, f):
    K, T, tau = x
    s = 2j * np.pi * f
    return K / (T * s + 1) * np.exp(-s * tau)


def fit_axis(f, G, coh, axis):
    # only where the setpoint explains the rate: above that the estimate drifts to -1/C
    sel = (f >= 0.4) & (f <= 15) & (coh >= 0.6)
    model = first_order
    w = 1 / np.abs(G[sel])

    def resid(x):
        d = (model(x, f[sel]) - G[sel]) * w
        return np.r_[d.real, d.imag]
    best = None
    starts = [[k, T, 0.02] for k in (2, 5, 10, 50) for T in (0.02, 0.05, 0.2, 0.5)]
    lo = [0.01, 0.002, 0.0]
    hi = [500, 5, 0.1]
    for x0 in starts:
        r = least_squares(resid, x0, bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    rel = np.sqrt(np.mean(np.abs((model(best.x, f[sel]) - G[sel]) * w) ** 2))
    return best.x, rel, sel, model


def main():
    out = {}
    fig, ax = plt.subplots(2, 3, figsize=(14, 6.5), sharex=True)
    for j, axis in enumerate(AXES):
        f, G, coh = axis_spectra(FL3600, axis)
        x, rel, sel, model = fit_axis(f, G, coh, axis)
        band = f[sel]
        if True:
            K, T, tau = x
            print('%-5s K %.2f rad/s per input, T %.3f s (K/T %.0f rad/s^2), delay %.0f ms; rel. error %.2f; band %.1f-%.1f Hz (%d bins)'
                  % (axis, K, T, K / T, tau * 1e3, rel, band.min(), band.max(), sel.sum()))
            out[axis] = dict(K=K, T=T, delay=tau)
        else:
            K, wn, z, tau = x
            tf = 1 / (2 * z * wn)
            print('%-5s K %.2f rad/s per input, %.2f Hz, zeta %.3f, delay %.0f ms; rel. error %.2f; band %.1f-%.1f Hz (%d bins)'
                  % (axis, K, wn / 2 / np.pi, z, tau * 1e3, rel, band.min(), band.max(), sel.sum()))
            print('       -> heli_model: L = wn^2 = %.0f, tf = %.3f s, control = K tf = %.3f' % (wn ** 2, tf, K * tf))
            out[axis] = dict(K=K, wn=wn, zeta=z, delay=tau, L=wn ** 2, tf=tf, ctrl=K * tf)
        pl = (f > 0.3) & (f < 30)
        ax[0][j].loglog(f[pl], np.abs(G[pl]), lw=0.8, label='flights')
        ax[0][j].loglog(f[pl], np.abs(model(x, f[pl])), lw=1, label='fit')
        ax[0][j].set_title(axis); ax[0][j].grid(True, which='both', lw=0.3)
        ax[1][j].semilogx(f[pl], np.degrees(np.unwrap(np.angle(G[pl]))), lw=0.8)
        ax[1][j].semilogx(f[pl], np.degrees(np.unwrap(np.angle(model(x, f[pl])))), lw=1)
        ax2 = ax[1][j].twinx(); ax2.semilogx(f[pl], coh[pl], color='0.6', lw=0.6); ax2.set_ylim(0, 1)
        ax2.set_ylabel('coherence', color='0.5')
        ax[1][j].set_xlabel('Hz'); ax[1][j].grid(True, which='both', lw=0.3)
        # rpm^2 check on the 3300 rpm flights
        f2, G2, c2 = axis_spectra(FL3300, axis)
        b = (f >= 1) & (f <= 3) & (coh > 0.4) & (c2 > 0.4)
        if b.any():
            print('       3300/3600 rpm gain ratio 1-3 Hz: %.2f (rpm^2: %.2f)' % (np.median(np.abs(G2[b]) / np.abs(G[b])), (3300 / 3600) ** 2))
    ax[0][0].set_ylabel('|rad/s per input|'); ax[1][0].set_ylabel('phase deg'); ax[0][0].legend()
    fig.tight_layout(); fig.savefig('axis_frf.png', dpi=110)
    np.save('axis_params.npy', out)


if __name__ == '__main__':
    main()
