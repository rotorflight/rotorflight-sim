"""
Fit the cyclic (roll/pitch) part of heli_model.Heli from the Stratos flights.

Model (first-order flapping, ArduPilot SIM_Helicopter / Walker & Tischler form), FRD body:
    b1s' = -p - b1s/tf_r + (Lfa1s a1s + Lflt dx + Lflg dy) / tf_r      p' = Lb1s b1s
    a1s' = -q - a1s/tf_p + (Mfb1s b1s + Mflt dx + Mflg dy) / tf_p      q' = Ma1s a1s
Eliminating the flapping states with b1s = p'/Lb1s, a1s = q'/Ma1s:
    p'' = -Lb1s p - p'/tf_r + (Lb1s Lfa1s / (Ma1s tf_r)) q' + (Lb1s/tf_r)(Lflt dx + Lflg dy)
    q'' = -Ma1s q - q'/tf_p + (Ma1s Mfb1s / (Lb1s tf_p)) p' + (Ma1s/tf_p)(Mflt dx + Mflg dy)
linear in [p, p', q', dx, dy] (resp. [q, q', p', dx, dy]). Inputs delayed by d (grid).

Frames: the log is in the firmware frame (roll + right, pitch + nose down, yaw + nose
left); FRD q = -gyro pitch. Inputs: dx = mixer roll, dy = -mixer pitch (the model's
sign_pitch = -1), both as mixer inputs (-1..1). Airborne segments (event 52) of the
3600 rpm flights; derivatives from a 25 Hz low-pass.
"""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'scripts')
from common import Log

FL3600 = ['20261008_172721_00', '20261008_174938_00', '20261008_180641_00', '20261008_182311_01']
FL3300 = ['20261008_160146_11', '20261008_160146_12']
D2R = np.pi / 180


def airborne_mask(L):
    t0 = L.c('time')[0]
    m = np.zeros(len(L.t), bool)
    state, start = 0, None
    for ts, typ, data in L.events:
        if typ == 52:
            tt = (ts - t0) / 1e6
            if data[0] == 1:
                start = tt
            elif start is not None:
                m |= (L.t > start + 1.0) & (L.t < tt - 1.0)
                start = None
    if start is not None:
        m |= (L.t > start + 1.0) & (L.t < L.t[-1] - 2.0)
    return m


def signals(name, cutoff=25):
    L = Log(name)
    fs = 1 / L.dt
    sos = butter(2, cutoff, fs=fs, output='sos')
    f = lambda x: sosfiltfilt(sos, x)
    p = f(L.c('gyroADC[0]') * D2R)
    q = f(-L.c('gyroADC[1]') * D2R)
    dx = f(L.c('mixer[0]') / 1000)
    dy = f(-L.c('mixer[1]') / 1000)
    pd, qd = np.gradient(p, L.dt), np.gradient(q, L.dt)
    pdd, qdd = np.gradient(pd, L.dt), np.gradient(qd, L.dt)
    return dict(L=L, p=p, q=q, pd=pd, qd=qd, pdd=pdd, qdd=qdd, dx=dx, dy=dy,
                m=airborne_mask(L), hs=np.median(L.c('headspeed')[airborne_mask(L)]))


def regress(sigs, d, axis):
    X, y = [], []
    for s in sigs:
        m = s['m'].copy()
        m[:d + 2] = False
        idx = np.nonzero(m)[0]
        dx, dy = s['dx'][idx - d], s['dy'][idx - d]
        if axis == 'roll':
            X.append(np.column_stack([s['p'][idx], s['pd'][idx], s['qd'][idx], dx, dy])); y.append(s['pdd'][idx])
        else:
            X.append(np.column_stack([s['q'][idx], s['qd'][idx], s['pd'][idx], dx, dy])); y.append(s['qdd'][idx])
    X, y = np.concatenate(X), np.concatenate(y)
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return c, 1 - (y - X @ c).var() / y.var()


def to_model(cr, cp):
    """Regression coefficients -> heli_model parameters."""
    Lb1s, tf_r = -cr[0], -1 / cr[1]
    Ma1s, tf_p = -cp[0], -1 / cp[1]
    Lfa1s = cr[2] * Ma1s * tf_r / Lb1s
    Mfb1s = cp[2] * Lb1s * tf_p / Ma1s
    Lflt, Lflg = cr[3] * tf_r / Lb1s, cr[4] * tf_r / Lb1s
    Mflt, Mflg = cp[3] * tf_p / Ma1s, cp[4] * tf_p / Ma1s
    return dict(Lb1s=Lb1s, Ma1s=Ma1s, tf_roll=tf_r, tf_pitch=tf_p, Lfa1s=Lfa1s, Mfb1s=Mfb1s,
                Lflt=Lflt, Lflg=Lflg, Mflt=Mflt, Mflg=Mflg)


def modes(P):
    """Eigenvalues of the 4-state open-loop system [p, q, b1s, a1s]."""
    A = np.array([
        [0, 0, P['Lb1s'], 0],
        [0, 0, 0, P['Ma1s']],
        [-1, 0, -1 / P['tf_roll'], P['Lfa1s'] / P['tf_roll']],
        [0, -1, P['Mfb1s'] / P['tf_pitch'], -1 / P['tf_pitch']],
    ])
    return np.linalg.eigvals(A)


def main():
    sigs = [signals(n) for n in FL3600]
    print('airborne seconds:', ['%s %.0f s @ %.0f rpm' % (s['L'].name[-9:], s['m'].sum() * s['L'].dt, s['hs']) for s in sigs])
    best = None
    for d_ms in range(0, 61, 4):
        d = int(round(d_ms / 1000 / sigs[0]['L'].dt))
        cr, r2r = regress(sigs, d, 'roll')
        cp, r2p = regress(sigs, d, 'pitch')
        print('  delay %2d ms: R^2 roll %.3f pitch %.3f' % (d_ms, r2r, r2p))
        if best is None or r2r + r2p > best[0]:
            best = (r2r + r2p, d_ms, cr, cp, r2r, r2p)
    _, d_ms, cr, cp, r2r, r2p = best
    P = to_model(cr, cp)
    print('best delay %d ms (R^2 roll %.3f, pitch %.3f)' % (d_ms, r2r, r2p))
    for k, v in P.items():
        print('  %-9s %+.4f' % (k, v))
    print('open-loop modes (Hz, damping):')
    for ev in modes(P):
        if ev.imag >= 0:
            wn = abs(ev)
            print('  %6.2f Hz  zeta %+.3f  (%s)' % (wn / 2 / np.pi, -ev.real / wn if wn else 0, ev))

    # check: rpm^2 scaling of the control power from the 3300 rpm flights
    s33 = [signals(n) for n in FL3300]
    d = int(round(d_ms / 1000 / sigs[0]['L'].dt))
    cr33, r2 = regress(s33, d, 'roll')
    cp33, r2b = regress(s33, d, 'pitch')
    P33 = to_model(cr33, cp33)
    print('3300 rpm flights: R^2 %.3f/%.3f; Lb1s %.0f Ma1s %.0f  (3600 -> 3300 rpm^2 ratio %.2f, fitted ratio %.2f/%.2f)'
          % (r2, r2b, P33['Lb1s'], P33['Ma1s'], (3300 / 3600) ** 2, P33['Lb1s'] / P['Lb1s'], P33['Ma1s'] / P['Ma1s']))

    # plot measured vs one-step fitted p'' and q'' on a stretch of one flight
    s = sigs[2]
    i0 = np.nonzero(s['m'])[0][len(np.nonzero(s['m'])[0]) // 2]
    sl = slice(i0, i0 + int(6 / s['L'].dt))
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for a_, axis, c, key in ((ax[0], 'roll', cr, 'pdd'), (ax[1], 'pitch', cp, 'qdd')):
        idx = np.arange(sl.start, sl.stop)
        if axis == 'roll':
            X = np.column_stack([s['p'][idx], s['pd'][idx], s['qd'][idx], s['dx'][idx - d], s['dy'][idx - d]])
        else:
            X = np.column_stack([s['q'][idx], s['qd'][idx], s['pd'][idx], s['dx'][idx - d], s['dy'][idx - d]])
        a_.plot(s['L'].t[idx], s[key][idx], lw=0.7, label='measured %s accel derivative' % axis)
        a_.plot(s['L'].t[idx], X @ c, lw=0.7, label='fit')
        a_.set_ylabel('rad/s^3'); a_.legend(loc='upper right')
    ax[-1].set_xlabel('s (%s)' % s['L'].name)
    fig.tight_layout(); fig.savefig('cyclic_fit.png', dpi=110)
    np.save('cyclic_params.npy', dict(P, delay_ms=d_ms))


if __name__ == "__main__":
    main()
