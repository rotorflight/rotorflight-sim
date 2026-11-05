"""
Frequency-domain fit of the cyclic model (see fit_cyclic.py for the equations).

The flights are closed loop: the PID mixes gyro noise back into the mixer inputs, so
a direct mixer->gyro estimate is biased. The roll/pitch setpoints are used as
instruments (indirect closed-loop identification):

    S_yr(f) = G(f) S_ur(f)   ->   G(f) = S_yr(f) S_ur(f)^-1          (2x2 per frequency)

y = [p, q] (FRD, rad/s), u = [dx, dy] (model inputs), r = setpoints. Welch cross
spectra over the airborne segments of the 3600 rpm flights, 250 Hz, 4 s segments.
The 4-state flapping model with an input delay is then fitted to G over the
frequencies where the setpoint->gyro coherence is usable.
"""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt, csd, decimate
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


def segments(name):
    """Airborne stretches of one flight, decimated to 250 Hz: dict of channel arrays."""
    L = Log(name)
    q = int(round(1 / L.dt / FS))
    m = airborne_mask(L)
    ch = dict(p=L.c('gyroADC[0]') * D2R, q=-L.c('gyroADC[1]') * D2R,
              dx=L.c('mixer[0]') / 1000, dy=-L.c('mixer[1]') / 1000,
              rx=L.c('setpoint[0]') * D2R, ry=-L.c('setpoint[1]') * D2R)
    out = []
    edges = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        if (b - a) * L.dt < NSEG / FS * 2:
            continue
        out.append({k: decimate(v[a:b], q, ftype='fir') for k, v in ch.items()})
    return out


def spectra(names):
    keys = ['p', 'q', 'dx', 'dy', 'rx', 'ry']
    S = None
    for n in names:
        for seg in segments(n):
            cur = {}
            for a in keys:
                for b in ('rx', 'ry'):
                    f, s = csd(seg[a], seg[b], fs=FS, nperseg=NSEG)
                    cur[(a, b)] = s
            if S is None:
                S = {k: v.copy() for k, v in cur.items()}
            else:
                for k in S:
                    S[k] += cur[k]
    return f, S


def plant_frf(f, S):
    G = np.zeros((len(f), 2, 2), complex)
    for i in range(len(f)):
        Syr = np.array([[S[('p', 'rx')][i], S[('p', 'ry')][i]], [S[('q', 'rx')][i], S[('q', 'ry')][i]]])
        Sur = np.array([[S[('dx', 'rx')][i], S[('dx', 'ry')][i]], [S[('dy', 'rx')][i], S[('dy', 'ry')][i]]])
        G[i] = Syr @ np.linalg.pinv(Sur)
    return G


def model_frf(P, f):
    Lb1s, Ma1s, tfr, tfp, Lfa1s, Mfb1s, Lflt, Lflg, Mflt, Mflg, tau = P
    A = np.array([[0, 0, Lb1s, 0], [0, 0, 0, Ma1s],
                  [-1, 0, -1 / tfr, Lfa1s / tfr], [0, -1, Mfb1s / tfp, -1 / tfp]])
    B = np.array([[0, 0], [0, 0], [Lflt / tfr, Lflg / tfr], [Mflt / tfp, Mflg / tfp]])
    C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    out = np.zeros((len(f), 2, 2), complex)
    for i, fi in enumerate(f):
        s = 2j * np.pi * fi
        out[i] = C @ np.linalg.solve(s * np.eye(4) - A, B) * np.exp(-s * tau)
    return out


def main():
    f, S = spectra(FL3600)
    G = plant_frf(f, S)
    # setpoint -> gyro coherence per axis as the usable-band criterion
    coh = lambda a, b: np.abs(S[(a, b)]) ** 2 / (np.abs(S[(a, a)]) * np.abs(S[(b, b)])) if (a, a) in S else None
    sel = (f >= 0.5) & (f <= 12)
    w = 1 / (np.abs(G[sel]) + 0.05 * np.abs(G[sel]).max(axis=0))   # relative error

    def resid(x):
        d = (model_frf(x, f[sel]) - G[sel]) * w
        return np.r_[d.real.ravel(), d.imag.ravel()]

    # Starts from the measured shape: resonance near 5.5 Hz -> L ~ (2 pi 5.5)^2,
    # damping 1/(2 tf wn), flat low-frequency gain Lflt/tf ~ 4.5 (roll), Mflg/tf ~ 5.5 (pitch)
    lo = [10, 10, 0.005, 0.005, -10, -10, -10, -10, -10, -10, 0.0]
    hi = [20000, 20000, 1.0, 1.0, 10, 10, 10, 10, 10, 10, 0.08]
    best = None
    for tf0 in (0.05, 0.1, 0.2):
        for L0 in (800, 1200, 1800):
            for cross in (-1.0, 0.0, 1.0):
                x0 = [L0, L0, tf0, tf0, cross, -cross, 4.5 * tf0, 0.0, 0.0, 5.5 * tf0, 0.02]
                r = least_squares(resid, x0, bounds=(lo, hi), x_scale='jac', max_nfev=3000)
                if best is None or r.cost < best.cost:
                    best = r
    P = best.x
    names = ['Lb1s', 'Ma1s', 'tf_roll', 'tf_pitch', 'Lfa1s', 'Mfb1s', 'Lflt', 'Lflg', 'Mflt', 'Mflg', 'delay_s']
    fit = model_frf(P, f[sel])
    err = np.linalg.norm((fit - G[sel]) * w) / np.linalg.norm(G[sel] * w)
    print('frequency fit 0.5-15 Hz: relative error %.2f' % err)
    for k, v in zip(names, P):
        print('  %-9s %+.4f' % (k, v))
    A = np.array([[0, 0, P[0], 0], [0, 0, 0, P[1]],
                  [-1, 0, -1 / P[2], P[4] / P[2]], [0, -1, P[5] / P[3], -1 / P[3]]])
    print('open-loop modes:')
    for ev in np.linalg.eigvals(A):
        if ev.imag >= 0:
            print('  %6.2f Hz  zeta %+.3f' % (abs(ev) / 2 / np.pi, -ev.real / abs(ev)))

    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    lab = [['dx->p (roll)', 'dy->p'], ['dx->q', 'dy->q (pitch)']]
    full = model_frf(P, f[(f > 0.3) & (f < 30)])
    for i in range(2):
        for j in range(2):
            ax[i][j].loglog(f[(f > 0.3) & (f < 30)], np.abs(G[(f > 0.3) & (f < 30), i, j]), lw=0.8, label='flights (setpoint IV)')
            ax[i][j].loglog(f[(f > 0.3) & (f < 30)], np.abs(full[:, i, j]), lw=0.8, label='fitted model')
            ax[i][j].axvspan(0.5, 15, color='0.92', zorder=0)
            ax[i][j].set_title(lab[i][j]); ax[i][j].grid(True, which='both', lw=0.3)
    ax[0][0].legend(); ax[1][0].set_xlabel('Hz'); ax[1][1].set_xlabel('Hz')
    ax[0][0].set_ylabel('|rad/s per input|'); ax[1][0].set_ylabel('|rad/s per input|')
    fig.tight_layout(); fig.savefig('cyclic_frf.png', dpi=110)
    np.save('cyclic_params.npy', dict(zip(names, P)))

    # 3300 rpm: control power should scale with rpm^2
    f2, S2 = spectra(FL3300)
    G2 = plant_frf(f2, S2)
    band = (f >= 1) & (f <= 4)
    ratio = np.median(np.abs(G2[band]) / np.abs(G[band]), axis=0)
    print('3300/3600 rpm |G| ratio 1-4 Hz (diag): roll %.2f pitch %.2f; rpm^2 predicts %.2f'
          % (ratio[0, 0], ratio[1, 1], (3300 / 3600) ** 2))


if __name__ == '__main__':
    main()
