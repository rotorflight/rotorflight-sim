"""
OMP M7R (700 class, 12S, servo tail, analog ESC) from one 333 s sport flight:
rotor/motor law, hover collective per headspeed, cyclic rate gains, servo-tail yaw,
gyro spectrum.

    python -I scripts/m7r_fit.py <decoded log .npy>      (bbl.js decode of the flight)
"""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt, csd, welch
from scipy.optimize import least_squares

sys.path.insert(0, 'scripts')
from common import bblog as _bblog
bblog = _bblog()

D2R = np.pi / 180
lg = bblog.load(sys.argv[1])
t = lg.t
fs = 1 / np.median(np.diff(t))
c = lambda n: np.asarray(lg[n], float)

air = np.zeros(len(t), bool)
st = None
for e in lg.events_named('AIRBORNE_STATE'):
    tt = lg.event_t(e)
    if e['data']['state'] == 1:
        st = tt
    elif st is not None:
        air |= (t > st + 1) & (t < tt - 1)
        st = None
target = c('govTarget')
hs = c('headspeed')
coll = c('mixer[3]') / 1000
u = c('motor[0]') / 1000
V = c('Vbat') / 100

# ---- rotor / motor law -------------------------------------------------------------------
lp = butter(2, 4, fs=fs, output='sos')
w = sosfiltfilt(lp, hs) * 2 * np.pi / 60
uf, Vf, cf = sosfiltfilt(lp, u), sosfiltfilt(lp, V), sosfiltfilt(lp, coll)
wd = np.gradient(w, t)
m = air & (hs > 1000)
m[::1] &= np.arange(len(t)) % 10 == 0        # decimate to 100 Hz


def res(p, n):
    A, B, k0, k1 = p
    return (A * uf[m] * Vf[m] - B * w[m] - w[m] ** 2 * (k0 + k1 * np.abs(cf[m]) ** n) - wd[m]) / 100


best = None
for n in (1.0, 1.5, 2.0):
    r = least_squares(res, [100, 1, 1e-3, 1e-2], args=(n,), bounds=([0, 0, 0, 0], [np.inf] * 4))
    yhat = wd[m] - r.fun * 100
    r2 = 1 - np.var(r.fun * 100) / np.var(wd[m] + 1e-9)
    # equation error on the speed balance: compare A u V with the rest (motor volts form)
    lhs = r.x[0] * uf[m] * Vf[m]
    rhs = r.x[1] * w[m] + w[m] ** 2 * (r.x[2] + r.x[3] * np.abs(cf[m]) ** n) + wd[m]
    r2v = 1 - np.var(lhs - rhs) / np.var(lhs)
    print('rotor law n %.1f: A %.2f B %.4f k0 %.3e k1 %.3e  R^2 (volts form) %.3f' % (n, *r.x, r2v))
    if best is None or r2v > best[0]:
        best = (r2v, n, r.x)
r2v, n, (A, B, k0, k1) = best
print('-> motor_fit = (%.4g, %.4g, %.4g, %.4g, %.1f); Kv-ish A/B %.1f rad/s per V (%.0f rpm/V at the rotor)'
      % (A, B, k0, k1, n, A / B, A / B * 60 / (2 * np.pi)))

# ---- hover collective per headspeed --------------------------------------------------------
att_r, att_p = c('attitude[0]') / 10, c('attitude[1]') / 10
calm = air & (np.abs(att_r) < 10) & (np.abs(att_p) < 10)
for i in (0, 1, 2):
    calm &= np.abs(c('setpoint[%d]' % i)) < 30
print('\nhover (upright, |att| < 10 deg, |setpoint| < 30 deg/s):')
hover = {}
for tg in sorted(set(np.round(target[air], -2))):
    k = calm & (np.abs(target - tg) < 30)
    if k.sum() < 2000:
        continue
    hover[tg] = (np.median(coll[k]), np.median(u[k]), np.median(V[k]), np.median(c('mixer[2]')[k]) / 1000)
    print('  %4.0f rpm: %5.1f s, collective %.3f, throttle %.3f at %.1f V, yaw mixer %+.3f'
          % (tg, k.sum() / fs, *hover[tg]))
if len(hover) >= 2:
    # c = a/wr^2 + b/wr, wr = rpm / nominal
    nom = 1800.0
    X = np.array([[1 / (tg / nom) ** 2, 1 / (tg / nom)] for tg in hover])
    y = np.array([hover[tg][0] for tg in hover])
    (a, b), *_ = np.linalg.lstsq(X, y, rcond=None)
    print('  inflow law at %d rpm nominal: a %.3f b %.3f; residuals %s' % (nom, a, b, np.round(y - X @ [a, b], 3)))
    lin = np.array([hover[nom][0] * (nom / tg) ** 2 for tg in hover]) if nom in hover else None
    if lin is not None:
        print('  thrust ~ rpm^2 x coll would predict %s (measured %s)' % (np.round(lin, 3), np.round(y, 3)))

# ---- cyclic rate gains (setpoint as instrument) ---------------------------------------------
print('\ncyclic rate gain G = S_yr / S_ur, rad/s per unit mixer input, airborne at 1800 rpm:')
seg = air & (np.abs(target - 1800) < 30)
idx = np.flatnonzero(seg)
for ax, name in ((0, 'roll'), (1, 'pitch'), (2, 'yaw')):
    r = c('setpoint[%d]' % ax)[idx] * D2R
    uu = c('mixer[%d]' % ax)[idx] / 1000
    y = c('gyroADC[%d]' % ax)[idx] * D2R
    f, Syr = csd(r, y, fs=fs, nperseg=4096)
    _, Sur = csd(r, uu, fs=fs, nperseg=4096)
    _, Srr = welch(r, fs=fs, nperseg=4096)
    _, Syy = welch(y, fs=fs, nperseg=4096)
    G = Syr / Sur
    coh = np.abs(Syr) ** 2 / (Srr * Syy)
    out = []
    for lo, hi in ((0.3, 1.0), (1.0, 2.0), (2.0, 4.0)):
        b = (f >= lo) & (f < hi)
        out.append('%.1f-%.1f Hz |G| %.2f (phase %+.0f deg, coh %.2f)'
                   % (lo, hi, np.median(np.abs(G[b])), np.degrees(np.median(np.angle(G[b]))), np.median(coh[b])))
    print('  %-5s ' % name + ' | '.join(out))

# ---- yaw: servo tail, regression of yaw acceleration ----------------------------------------
print('\nyaw (servo tail), 0.5-8 Hz regression r_dot = N_ped y + N_col c + N_r r per headspeed:')
bp = butter(2, [0.5, 8], 'bandpass', fs=fs, output='sos')
r_ = sosfiltfilt(bp, c('gyroADC[2]') * D2R)
rd = np.gradient(r_, t)
yy = sosfiltfilt(bp, c('mixer[2]') / 1000)
cc = sosfiltfilt(bp, coll)
for tg in sorted(hover):
    k = air & (np.abs(target - tg) < 30)
    k[::1] &= np.arange(len(t)) % 5 == 0
    for lag_ms in (0, 20, 40, 60):
        L = int(lag_ms * fs / 1000)
        X = np.c_[yy[k.nonzero()[0] - L], cc[k.nonzero()[0] - L], r_[k]]
        coef, *_ = np.linalg.lstsq(X, rd[k], rcond=None)
        r2 = 1 - np.var(rd[k] - X @ coef) / np.var(rd[k])
        print('  %4.0f rpm lag %2d ms: N_ped %+7.1f  N_col %+7.1f  N_r %+6.2f  R^2 %.2f' % (tg, lag_ms, *coef, r2))

# ---- spectra ---------------------------------------------------------------------------------
print('\ngyro spectrum peaks 2-15 Hz, airborne, per headspeed:')
for tg in sorted(hover):
    k = air & (np.abs(target - tg) < 30)
    out = []
    for ax in (0, 1, 2):
        x = c('gyroADC[%d]' % ax)[k]
        f, P = welch(x - x.mean(), fs=fs, nperseg=8192)
        b = (f > 2) & (f < 15)
        cc_ = b & (P > 0.5 * P[b].max())
        fc = np.sum(f[cc_] * P[cc_]) / np.sum(P[cc_])
        out.append('%s %.2f Hz (%.3f/rev)' % ('rpy'[ax], fc, fc / (tg / 60)))
    print('  %4.0f rpm: %s' % (tg, ', '.join(out)))
