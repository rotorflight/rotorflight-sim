"""
Servo tail dynamics from a real log and a sim replay of it (heli_model tail_lag,
yaw_mode_hz, yaw_mode_zeta and Nped): what the yaw loop sees between the tail command
(mixer[2]) and the gyro (gyroADC[2]) beyond the model's first-order yaw axis.

    python -I scripts/fit_tail.py <real .npy> <sim .npy> T0,T1 RPM_LO,RPM_HI [EX0,EX1]

<sim .npy>: the decoded blackbox of a replay of the same flight through the SITL with the
tail dynamics off (tail_lag 0, yaw_mode_hz 0) and the real filter settings - the RPM filter
above all, whose 1x notch adds most of the firmware's lag at 12 Hz - logged at 400 Hz or
more (--cli "set blackbox_rate_denom = 2", --bbl). T0,T1 and RPM_LO,RPM_HI select the real
log's stretch at one headspeed (the replay is cut by the same headspeed band), EX0,EX1
leaves out a stretch of it (the tail wag itself).

Closed loop, the plain mixer-to-gyro estimate returns the controller's inverse wherever
the gyro noise drives the loop (flat and at -180 deg above ~8 Hz). The yaw setpoint is the
instrument instead: H = S_r,gyro / S_r,mixer. On the sim's own replay this returns the
model's Nped rpm^2 / (s - Nr) within a few percent, so the ratio real / sim cancels the
firmware's filters and leaves the tail's own dynamics, fitted as

    R(s) = Nped / Nped_sim  /  (1 + s tail_lag)  *  wn^2 / (s^2 + 2 zeta wn s + wn^2)

in log magnitude and phase over 1-13 Hz (above, the setpoint has too little power).
OMP M7R (OMP_M7R_20261022_155150 against a replay of its m7r_cruise case, the 1500 rpm
stretches of 70-400 s without the wag at 236-242 s): Nped 91, tail_lag 6 ms, yaw mode
16.4 Hz at zeta 0.23 - the M7R preset's values.

    python -I scripts/fit_tail.py <log dir>/OMP_M7R_20261022_155150.0.npy <replay>.0.npy \
        70,400 1450,1560 236,242
"""
import sys

import numpy as np
from scipy import signal
from scipy.optimize import least_squares

sys.path.insert(0, 'scripts')
from common import bblog as _bblog

NPED_SIM = 55.0       # the preset's Nped in the sim replay (the M7R's before this fit)
F = np.arange(1.0, 13.01, 0.5)


def iv_frf(path, t0, t1, lo, hi, excl=None):
    """Yaw plant mixer[2] -> gyroADC[2] with setpoint[2] as instrument, at the frequencies F."""
    lg = _bblog().load(path)
    c = lambda n: np.asarray(lg[n], float)
    t = lg.t - lg.t[0]
    fs = 1 / np.median(np.diff(t))
    g, m, r, hs = c('gyroADC[2]'), c('mixer[2]'), c('setpoint[2]'), c('headspeed')
    sel = (hs > lo) & (hs < hi) & (t > t0) & (t < t1)
    if excl:
        sel &= ~((t > excl[0]) & (t < excl[1]))
    idx = np.flatnonzero(sel)
    nps = int(2 * fs)
    s_ry = s_ru = 0
    for seg in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
        if len(seg) < 2 * nps:
            continue
        x = r[seg] - r[seg].mean()
        f, ry = signal.csd(x, g[seg] - g[seg].mean(), fs, nperseg=nps)
        _, ru = signal.csd(x, m[seg] - m[seg].mean(), fs, nperseg=nps)
        s_ry, s_ru = s_ry + ry * len(seg), s_ru + ru * len(seg)
    h = s_ry / s_ru
    return np.interp(F, f, h.real) + 1j * np.interp(F, f, h.imag)


def model(q, f):
    n, tau, fn, z = q
    s = 2j * np.pi * f
    wn = 2 * np.pi * fn
    return n / NPED_SIM / (1 + s * tau) * wn ** 2 / (s ** 2 + 2 * z * wn * s + wn ** 2)


def main():
    real, sim = sys.argv[1], sys.argv[2]
    t0, t1 = (float(x) for x in sys.argv[3].split(','))
    lo, hi = (float(x) for x in sys.argv[4].split(','))
    excl = tuple(float(x) for x in sys.argv[5].split(',')) if len(sys.argv) > 5 else None
    ratio = iv_frf(real, t0, t1, lo, hi, excl) / iv_frf(sim, 0, 1e9, lo, hi)

    def res(q):
        d = np.log(model(q, F)) - np.log(ratio)
        return np.concatenate([d.real, np.angle(np.exp(1j * d.imag))])
    q = least_squares(res, [80, 0.005, 14, 0.25], bounds=([30, 0, 6, 0.03], [200, 0.05, 40, 1.5])).x
    print('Nped %.1f  tail_lag %.4f s  yaw_mode_hz %.2f  yaw_mode_zeta %.3f' % tuple(q))
    print('   Hz   real/sim |R|  phase     fit |R|  phase')
    for f, r, m in zip(F, ratio, model(q, F)):
        print('%5.1f   %10.2f %6.1f   %9.2f %6.1f'
              % (f, abs(r), np.degrees(np.angle(r)), abs(m), np.degrees(np.angle(m))))


main()
