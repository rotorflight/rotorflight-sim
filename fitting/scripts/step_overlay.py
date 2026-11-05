"""
Time-domain check at a sharp stick input: real vs SITL replay, same sticks, same gains.

    python -I scripts/step_overlay.py <flight> <sim.npy> <axis 0|1|2> <t_real> [out.png]

Aligns the two logs on the sticks (compare_replay.align), takes -1..+3 s around t_real,
and reports for setpoint and gyro: peak rate, time to peak, overshoot over the setpoint
peak, and the RMS difference between the two gyro traces.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'scripts')
from common import Log
from compare_replay import Npz, align

AXES = ('roll', 'pitch', 'yaw')


def window(L, t0, t1, n):
    m = (L.t >= t0) & (L.t <= t1)
    return L.t[m] - t0, L.c(n)[m]


def main():
    flight, simpath, axis, t_real = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
    out = sys.argv[5] if len(sys.argv) > 5 else None
    real, sim = Log(flight), Npz(simpath)
    lag = align(real, sim)
    a, b = t_real - 1.0, t_real + 3.0
    tr, gr = window(real, a, b, 'gyroADC[%d]' % axis)
    _, spr = window(real, a, b, 'setpoint[%d]' % axis)
    ts, gs = window(sim, a + lag, b + lag, 'gyroADC[%d]' % axis)
    _, sps = window(sim, a + lag, b + lag, 'setpoint[%d]' % axis)
    gs_on_r = np.interp(tr, ts, gs)
    sign = np.sign(spr[np.argmax(np.abs(spr))])

    def describe(t, sp, g):
        i = np.argmax(sign * g)
        j = np.argmax(sign * sp)
        return sign * sp[j], sign * g[i], t[i] - t[j]

    print('%s step at %.1f s (sim aligned +%.2f s)' % (AXES[axis], t_real, lag))
    for label, (spk, gpk, dt) in (('real', describe(tr, spr, gr)), ('sim', describe(ts, sps, gs))):
        print('  %-4s setpoint peak %6.1f deg/s, gyro peak %6.1f deg/s (%+.0f %%), %.0f ms after the setpoint peak'
              % (label, spk, gpk, 100 * (gpk / spk - 1), dt * 1000))
    print('  gyro RMS difference sim - real: %.1f deg/s (real gyro RMS %.1f)'
          % (np.sqrt(np.mean((gs_on_r - gr) ** 2)), np.sqrt(np.mean(gr ** 2))))
    if out:
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(tr - 1, spr, 'k--', lw=0.8, label='setpoint (real)')
        ax.plot(tr - 1, gr, lw=0.9, label='gyro, real heli')
        ax.plot(ts - 1, gs, lw=0.9, label='gyro, SITL + model')
        ax.set_xlabel('s from the input'); ax.set_ylabel('%s rate, deg/s' % AXES[axis])
        ax.legend(); ax.grid(lw=0.3)
        fig.tight_layout(); fig.savefig(out, dpi=110)


main()
