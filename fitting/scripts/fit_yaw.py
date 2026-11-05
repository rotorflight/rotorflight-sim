"""
Yaw with a motorized tail, time domain (the closed-loop frequency estimate in fit_axis.py
was not trustworthy for yaw, and its gain bound forced the wrong sign).

    r' = Nr r + Nt m_lag + c         m_lag' = (m - m_lag) / tau

r yaw rate FRD (rad/s, + nose right = -gyro yaw), m tail motor 0..1, c the main rotor
torque reaction (nearly constant in hover). Least squares on the band-limited yaw
acceleration of the airborne segments, grid over the tail motor lag tau.
"""
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt, lfilter

sys.path.insert(0, 'scripts')
from common import Log
from fit_cyclic import airborne_mask, FL3600

D2R = np.pi / 180


def data(name, tau):
    L = Log(name)
    fs = 1 / L.dt
    lp = butter(2, 10, fs=fs, output='sos')
    r = sosfiltfilt(lp, -L.c('gyroADC[2]') * D2R)
    rd = np.gradient(r, L.dt)
    m = L.c('motor[1]') / 1000
    a = L.dt / (tau + L.dt)
    m_lag = lfilter([a], [1, -(1 - a)], m)
    m_lag = sosfiltfilt(lp, m_lag)
    k = airborne_mask(L)
    return r[k], rd[k], m_lag[k], L.c('mixer[3]')[k] / 1000


def main():
    best = None
    for tau_ms in (10, 20, 30, 40, 50, 60, 80):
        X, y = [], []
        for n in FL3600:
            r, rd, ml, c = data(n, tau_ms / 1000)
            X.append(np.column_stack([r, ml, np.ones_like(r)])); y.append(rd)
        X, y = np.concatenate(X), np.concatenate(y)
        p, *_ = np.linalg.lstsq(X, y, rcond=None)
        r2 = 1 - (y - X @ p).var() / y.var()
        print('tau %2d ms: Nr %+.2f  Nt %+.1f rad/s^2 per motor  c %+.1f  R^2 %.3f' % (tau_ms, *p, r2))
        if best is None or r2 > best[0]:
            best = (r2, tau_ms, p)
    r2, tau_ms, (Nr, Nt, c) = best
    print('best: tau %d ms, Nr %.2f /s, Nt %.1f rad/s^2 per unit tail motor, torque %.1f rad/s^2 '
          '(hover tail = -c/Nt = %.3f)' % (tau_ms, Nr, Nt, c, -c / Nt))


main()
