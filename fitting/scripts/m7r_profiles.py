"""
The M7R log's header holds only PID/rate profile 1 (the cruise profile). Profiles 2
(1800 rpm) and 3 (2000 rpm) are recovered from the logged PID terms and setpoints:

    python -I scripts/m7r_profiles.py <decoded log .npy>

- rates: setpoint against stick, the ROTORFLIGHT curve rate*5*(x(1-e) + x^(s/16+2) e)
  fitted for rate and expo (srate fixed at the header's), collective as a plain ratio
- yaw P per error sign over the CW/CCW stop gains (the profiles keep profile 1's stop
  gains: both signs drop alike); the governor's P against its headspeed error
- P, I, D, F: each logged term regressed on its input (error, error integral rate,
  gyro rate of change, setpoint), as a ratio to profile 1 whose header gains are known -
  filters and scalings cancel as long as they are global
Prints CLI lines per profile.
"""
import json
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, sosfiltfilt

sys.path.insert(0, 'scripts')
from common import bblog as _bblog

SEG = {1: (252.0, 324.0), 2: (26.0, 169.0), 3: (173.0, 236.0)}
AX = ('roll', 'pitch', 'yaw')


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

    lp = butter(2, 20, fs=fs, output='sos')
    srate = num('rates')
    cw, ccw = (v / 100 for v in num('yaw_stop_gain'))

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
            err = c('axisError[%d]' % ax)
            gyro = c('gyroADC[%d]' % ax)
            if ax == 2:
                # yaw P carries the CW/CCW stop gain: fit each error sign, keep the stop gains
                r['P', ax] = np.mean([slope(c('axisP[2]'), err, m & (sg * err > 10)) * k
                                      for sg, k in ((1, 1 / cw), (-1, 1 / ccw))])
            else:
                r['P', ax] = slope(c('axisP[%d]' % ax), err, m)
            r['F', ax] = slope(c('axisF[%d]' % ax), c('setpoint[%d]' % ax), m)
            dg = np.gradient(sosfiltfilt(lp, gyro), t)
            r['D', ax] = -slope(sosfiltfilt(lp, c('axisD[%d]' % ax)), dg, m)
            di = np.gradient(c('axisI[%d]' % ax), t)
            r['I', ax] = slope(di, err, m & (np.abs(err) < 20))
        ge = c('govTarget') - c('headspeed')
        r['govP'] = slope(c('govP'), ge, m & (np.abs(ge) < 200))
        x = c('rcCommand[3]')
        r['coll'] = slope(c('setpoint[3]'), x, m & (np.abs(x) > 20))
        res[p] = r

    hdr = {'P': 0, 'I': 1, 'D': 2, 'F': 3}
    pid = [num(a + 'PID') for a in AX]
    for p in SEG:
        r = res[p]
        print('# profile %d (%.0f-%.0f s)' % (p, *SEG[p]))
        for ax in range(3):
            g = []
            for k in 'PIDF':
                base = res[1][k, ax]
                g.append(round(pid[ax][hdr[k]] * r[k, ax] / base) if abs(base) > 1e-9 else pid[ax][hdr[k]])
            rr, ex = r['rate', ax]
            print('  %-5s P %4d I %4d D %4d F %4d   rate %5.1f expo %4.0f' % (AX[ax], *g, rr, ex))
        print('  gov_p_gain %d' % round(num('govPID')[0] * r['govP'] / res[1]['govP']))
        print('  collective setpoint/stick %.3f (x%.2f of profile 1)' % (r['coll'], r['coll'] / res[1]['coll']))


main()
