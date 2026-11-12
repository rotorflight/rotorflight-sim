"""
The M7R log's header holds the gains of the profile that was active when logging started -
not the one flown first: the log switches to profile 1 before the spool-up (the header's
yaw P 110 / D 20 against the 152 / 48 profile 1 flies with). So the gains of profiles 1
(1500 rpm), 2 (1800 rpm) and 3 (2000 rpm) are fitted absolutely from the logged PID terms:

    python -I scripts/m7r_profiles.py <decoded log .npy>

- rates: setpoint against stick, the ROTORFLIGHT curve rate*5*(x(1-e) + x^(s/16+2) e)
  fitted for rate and expo (srate fixed at the header's), collective as a plain ratio
- P, I, D, F: each logged term regressed on its input through the firmware's own scaling
  (pid.h *_TERM_SCALE, x1000 in the log): P on the error (yaw: times the CW/CCW stop gain,
  which the firmware blends over +-10 deg/s), F on the setpoint, D on the gyro's rate of
  change behind the profile's D cutoff (BW header, first order), I's rate of change on the
  error. Checked on SITL logs with known gains: P, D, F within ~5 %; I reads 7-14 % low
  (iterm relax holds it back on fast errors)
- the governor's P against its headspeed error, as a ratio to profile 1's (its scale
  differs per governor mode)
Prints CLI lines per profile.
"""
import json
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

sys.path.insert(0, 'scripts')
from common import bblog as _bblog

SEG = {1: (252.0, 324.0), 2: (26.0, 169.0), 3: (173.0, 236.0)}
AX = ('roll', 'pitch', 'yaw')
# pid.h: P, I, D, F term scales per axis (the log holds the terms x1000)
SCALE = ((6.66666e-6, 0.0002, 0.1e-6, 0.000025),
         (6.66666e-6, 0.0002, 1.0e-6, 0.000025),
         (6.666666e-5, 0.0005, 1.0e-6, 0.000025))


def main():
    npy = sys.argv[1]
    lg = _bblog().load(npy)
    h = json.load(open(npy[:-4] + '.json', encoding='utf-8'))['rawHeader']
    num = lambda k: [int(x) for x in h[k].split(',')]
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
            air |= (t > st + 0.5) & (t < tt - 0.5)
            st = None

    srate = num('rates')
    cw, ccw = (v / 100 for v in num('yaw_stop_gain'))
    bw = [num(a + 'BW') for a in AX]

    def slope(y, x, m):
        x, y = x[m], y[m]
        return float(np.dot(x, y) / np.dot(x, x))

    res = {}
    for p, (a, b) in SEG.items():
        m = air & (t > a) & (t < b)
        r = {}
        for ax in range(3):
            x = np.abs(c('rcCommand[%d]' % ax)) / 500
            y = np.abs(c('setpoint[%d]' % ax))
            mm = m & (x > 0.02)
            s = srate[ax] / 16 + 2

            def f(q):
                return q[0] * 5 * (x[mm] * (1 - q[1]) + x[mm] ** s * q[1]) - y[mm]
            q = least_squares(f, [80, 0.3], bounds=([1, -1], [255, 1])).x
            r['rate', ax] = (q[0], q[1] * 100)
            sP, sI, sD, sF = (1000 * v for v in SCALE[ax])
            gyro = c('gyroADC[%d]' % ax)
            sp = c('setpoint[%d]' % ax)
            err = sp - gyro
            k = np.interp(err, [-10, 10], [ccw, cw]) if ax == 2 else 1.0
            r['P', ax] = slope(c('axisP[%d]' % ax), err * k, m) / sP
            r['F', ax] = slope(c('axisF[%d]' % ax), sp, m) / sF
            al = 1 - np.exp(-2 * np.pi * bw[ax][1] / fs)
            dg = lfilter([al], [1, al - 1], np.gradient(-gyro, t))
            r['D', ax] = slope(c('axisD[%d]' % ax), dg, m) / sD
            di = np.gradient(c('axisI[%d]' % ax), t)
            r['I', ax] = slope(di, err * k, m & (np.abs(err) < 20)) / sI
        ge = c('govTarget') - c('headspeed')
        r['govP'] = slope(c('govP'), ge, m & (np.abs(ge) < 200))
        x = c('rcCommand[3]')
        r['coll'] = slope(c('setpoint[3]'), x, m & (np.abs(x) > 20))
        res[p] = r

    for p in SEG:
        r = res[p]
        print('# profile %d (%.0f-%.0f s)' % (p, *SEG[p]))
        for ax in range(3):
            g = [max(0, round(r[k, ax])) for k in 'PIDF']
            rr, ex = r['rate', ax]
            print('  %-5s P %4d I %4d D %4d F %4d   rate %5.1f expo %4.0f' % (AX[ax], *g, rr, ex))
        print('  governor P: x%.2f of profile 1' % (r['govP'] / res[1]['govP']))
        print('  collective setpoint/stick %.3f (x%.2f of profile 1)' % (r['coll'], r['coll'] / res[1]['coll']))


main()
