"""
Compare a SITL replay (its own blackbox, decoded) with the real flight it replays.

    python -I scripts/compare_replay.py <real flight> <sim.npz> [label] [--plot out.png]

Both logs are aligned by cross-correlating the pilot's sticks (rcCommand, identical by
construction), then compared over the real flight's airborne window:
  - rate tracking: RMS of setpoint - gyro per axis (the firmware's own error)
  - gyro spectrum: band power 4-8 Hz (the Bo105 whirl) and the peak frequency
  - vibration: RMS of the filtered gyro (gyroADC, what the PIDs see) and, where both logs
    have it, of the unfiltered gyro (gyroRAW, what the sensor saw) from 20 Hz to Nyquist -
    with a replay flown with --vibration, how close the sim's vibration and the firmware's
    filtering of it come to the real heli's
  - hover: throttle, tail motor, headspeed spread
  - spectra: the log-spectral distance of the filtered gyro (what the PIDs see) per axis,
    1-30 Hz - the RMS of 10 log10(P_sim / P_real) over that band, in dB: how far the sim's
    whole closed-loop spectrum (flap/body mode, whirl, disturbances) is from the real one
"""
import os
import sys
import numpy as np
from scipy.signal import welch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'scripts')
from common import Log, SKILL
from fit_cyclic import airborne_mask


class Npz:
    """A decoded sim log: the blackbox-decode skill's .npy (bbl.js) or the older .npz."""

    def __init__(self, path):
        if path.endswith('.npy'):
            sys.path.insert(0, SKILL)
            import bblog
            lg = bblog.load(path)
            self.get = lambda n: np.asarray(lg[n], float)
            self.t = np.asarray(lg.t, float)
        else:
            z = np.load(path)
            a, N = z['data'], list(z['names'])
            self.get = lambda n: a[:, N.index(n)].astype(float)
            self.t = (self.get('time') - self.get('time')[0]) / 1e6
        self.dt = float(np.median(np.diff(self.t)))

    def c(self, n):
        return self.get(n)


def align(real, sim):
    """Time offset sim - real from the sticks (roll+pitch+collective), 100 Hz grid."""
    def sig(L):
        tt = np.arange(0, L.t[-1], 0.01)
        # stick motion, not level: the collective step at spool-up would dominate
        return np.diff(sum(np.interp(tt, L.t, L.c('rcCommand[%d]' % i)) for i in (0, 1, 3)))
    r, s = sig(real), sig(sim)
    # full cross-correlation (FFT): any offset, normalised by the overlap, >= 10 s of it
    from scipy.signal import correlate
    xc = correlate(s, r, mode='full', method='fft')
    ks = np.arange(-(len(r) - 1), len(s))
    overlap = np.minimum(len(s), ks + len(r)) - np.maximum(0, ks)
    xc = np.where(overlap >= 1000, xc / np.maximum(overlap, 1), -np.inf)
    lag = ks[np.argmax(xc)] * 0.01
    return lag


def metrics(L, t0, t1):
    m = (L.t >= t0) & (L.t <= t1)
    out = {}
    for i, ax in enumerate(('roll', 'pitch', 'yaw')):
        e = L.c('setpoint[%d]' % i)[m] - L.c('gyroADC[%d]' % i)[m]
        out[ax + '_err_rms'] = np.sqrt(np.mean(e ** 2))
        f, P = welch(L.c('gyroADC[%d]' % i)[m], fs=1 / L.dt, nperseg=4096)
        band = (f >= 4) & (f <= 8)
        out[ax + '_4_8Hz'] = np.sqrt(np.trapz(P[band], f[band]))
        wide = (f >= 2) & (f <= 15)
        out[ax + '_peak_hz'] = f[wide][np.argmax(P[wide])]
        out[ax + '_psd'] = (f, P)
        hi = f >= 20
        out[ax + '_vib_filt'] = np.sqrt(np.trapz(P[hi], f[hi]))
        try:
            g = L.c('gyroRAW[%d]' % i)[m]
            fr, Pr = welch(g - g.mean(), fs=1 / L.dt, nperseg=4096)
            hr = fr >= 20
            out[ax + '_vib_raw'] = np.sqrt(np.trapz(Pr[hr], fr[hr]))
            out[ax + '_psd_raw'] = (fr, Pr)
        except ValueError:
            out[ax + '_vib_raw'] = float('nan')
    out['throttle'] = np.median(L.c('motor[0]')[m]) / 10
    try:
        out['tail'] = np.median(L.c('motor[1]')[m]) / 10       # motorized tail, %
    except ValueError:
        out['tail'] = np.median(L.c('mixer[2]')[m]) / 10       # servo tail: yaw mixer, %
    out['hs_std'] = np.std(L.c('headspeed')[m])
    out['volts'] = np.median(L.c('Vbat')[m]) / 100
    return out


def spectral_distance(R, S, ax, lo=1.0, hi=30.0):
    """RMS dB difference of the filtered gyro PSDs of two metrics() results over lo-hi Hz
    (the sim's spectrum interpolated onto the real one's frequencies)."""
    fr, Pr = R[ax + '_psd']
    fs_, Ps = S[ax + '_psd']
    b = (fr >= lo) & (fr <= hi)
    ps = np.interp(fr[b], fs_, Ps)
    return float(np.sqrt(np.mean((10 * np.log10(np.maximum(ps, 1e-12) / np.maximum(Pr[b], 1e-12))) ** 2)))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    plot = sys.argv[sys.argv.index('--plot') + 1] if '--plot' in sys.argv else None
    if plot in args:
        args.remove(plot)
    real, sim = Log(args[0]), Npz(args[1])
    label = args[2] if len(args) > 2 else 'sim'
    lag = align(real, sim)
    m = airborne_mask(real)
    t0, t1 = real.t[m][0] + 2, real.t[m][-1] - 2
    powered = sim.t[sim.c('motor[0]') > 0]
    t1 = min(t1, powered[-1] - lag - 1)        # a shorter replay covers less; not its disarmed end
    R = metrics(real, t0, t1)
    S = metrics(sim, t0 + lag, t1 + lag)
    print('aligned: sim = real + %.2f s; window %.0f-%.0f s of the real flight (%.0f s)' % (lag, t0, t1, t1 - t0))
    print('%-22s %10s %10s' % ('', 'real', label))
    for k in ('roll_err_rms', 'pitch_err_rms', 'yaw_err_rms', 'roll_4_8Hz', 'pitch_4_8Hz', 'yaw_4_8Hz',
              'roll_peak_hz', 'pitch_peak_hz', 'roll_vib_raw', 'pitch_vib_raw', 'yaw_vib_raw',
              'roll_vib_filt', 'pitch_vib_filt', 'yaw_vib_filt', 'throttle', 'tail', 'hs_std', 'volts'):
        print('%-22s %10.2f %10.2f' % (k, R[k], S[k]))
    for ax in ('roll', 'pitch', 'yaw'):
        print('%-22s %21.1f dB' % (ax + ' spectrum distance', spectral_distance(R, S, ax)))
    if plot:
        fig, axs = plt.subplots(2, 3, figsize=(14, 8))
        ax = axs[0]
        for i, axn in enumerate(('roll', 'pitch', 'yaw')):
            for d, lab in ((R, 'real'), (S, label)):
                f, P = d[axn + '_psd']
                sel = (f > 0.5) & (f < 30)
                ax[i].semilogy(f[sel], P[sel], lw=0.8, label=lab)
            ax[i].axvspan(4, 8, color='0.92', zorder=0)
            ax[i].set_title('%s gyro PSD' % axn); ax[i].set_xlabel('Hz')
            # vibration: unfiltered (dashed) and filtered gyro up to Nyquist
            for d, lab, c in ((R, 'real', 'C0'), (S, label, 'C1')):
                f, P = d[axn + '_psd']
                axs[1][i].semilogy(f[f >= 10], P[f >= 10], lw=0.7, color=c, label=lab + ' filtered')
                if axn + '_psd_raw' in d:
                    f, P = d[axn + '_psd_raw']
                    axs[1][i].semilogy(f[f >= 10], P[f >= 10], lw=0.7, color=c, ls='--', label=lab + ' raw')
            axs[1][i].set_title('%s vibration' % axn); axs[1][i].set_xlabel('Hz')
        ax[0].set_ylabel('(deg/s)^2/Hz'); ax[0].legend(); axs[1][0].legend(fontsize=7)
        fig.tight_layout(); fig.savefig(plot, dpi=110)
    return R, S


if __name__ == '__main__':
    main()
