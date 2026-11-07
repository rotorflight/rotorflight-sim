"""
Gyro and accelerometer noise of a flight controller at rest, by Allan deviation.

    python -I scripts/allan.py <soak.csv> --chip ICM40608 [--lsb-per-dps 16.4] [--plot out.png]

Input: a long recording of a flight controller lying still (30 min or more), columns
t (s), gyro_[xyz]_lsb, acc_[xyz]_g - the format of the imu_test soak runs (MSP_RAW_IMU at
~200 Hz: gyroADCf in counts, acc in g). The samples are resampled onto a uniform grid.

Fitted per axis from the overlapping Allan deviation sigma(tau):
  white noise   N  (angle / velocity random walk): sigma = N / sqrt(tau), read at tau = 1 s
  bias          B  (bias instability): min sigma / 0.664, and the tau of that minimum
  rate walk     K  (rate / acceleration random walk): sigma = K sqrt(tau / 3), from the
                rising part past the minimum, if there is one
and the bias drift over the run (first 5 min against the last 5 min).

What MSP sampling allows: the gyro comes through the firmware's filters and a 200 Hz poll,
so the spectrum above ~50 Hz is not the sensor's. Allan deviation at tau >= 1 s only
depends on the noise below ~1 Hz, where the filters pass everything, so N, B and K hold;
N converts to the white-noise density the sim needs (deg/s/rtHz = N[deg/rt h] / 60), which
compares directly with the datasheet figure. Quantisation: 1 count (0.061 deg/s at
16.4 LSB/dps) is larger than the noise at rest, so the dither of the noise itself carries
the fit; it is fine for sigma at tau >= 1 s, not for the sample-level distribution.

Writes fitting/sensors/<chip>.json, which the sim's sensor model reads (heli/sensors.py).
"""
import argparse
import csv
import json
import os

import numpy as np

AXES = 'xyz'


def load(path, lsb_per_dps):
    t, g, a = [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r['t']))
            g.append([float(r['gyro_%s_lsb' % ax]) / lsb_per_dps for ax in AXES])
            a.append([float(r['acc_%s_g' % ax]) for ax in AXES])
    t = np.array(t)
    return t - t[0], np.array(g), np.array(a)


def resample(t, x, fs):
    tg = np.arange(0, t[-1], 1 / fs)
    return tg, np.column_stack([np.interp(tg, t, x[:, i]) for i in range(x.shape[1])])


def adev(x, fs, per_decade=10):
    """Overlapping Allan deviation of a rate signal x sampled at fs."""
    theta = np.cumsum(x) / fs
    n = len(x)
    m = np.unique(np.logspace(0, np.log10(n // 3), int(np.log10(n // 3) * per_decade)).astype(int))
    tau, sig = [], []
    for k in m:
        d = theta[2 * k:] - 2 * theta[k:-k] + theta[:-2 * k]
        tau.append(k / fs)
        sig.append(np.sqrt(np.mean(d * d) / 2) / (k / fs))
    return np.array(tau), np.array(sig)


def fit_terms(tau, sig):
    n = float(np.exp(np.interp(0.0, np.log(tau), np.log(sig))))      # sigma at tau = 1 s
    i = int(np.argmin(sig))
    b = float(sig[i] / 0.664)
    k = None
    if i < len(tau) - 3:
        # rising part: sigma = K sqrt(tau/3); least squares on the points past the minimum
        # where the slope is near +1/2
        tt, ss = tau[i + 1:], sig[i + 1:]
        slope = np.gradient(np.log(ss), np.log(tt))
        s = slope > 0.25
        if s.sum() >= 2:
            k = float(np.exp(np.median(np.log(ss[s]) - 0.5 * np.log(tt[s] / 3))))
    return n, b, float(tau[i]), k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--chip', required=True)
    ap.add_argument('--lsb-per-dps', type=float, default=16.4)
    ap.add_argument('--fs', type=float, default=100.0, help='resampling rate (default 100 Hz)')
    ap.add_argument('--out', default=os.path.join('sensors'))
    ap.add_argument('--plot')
    a = ap.parse_args()
    t, g, acc = load(a.csv, a.lsb_per_dps)
    tg, g = resample(t, g, a.fs)
    _, acc = resample(t, acc, a.fs)
    five = int(300 * a.fs)
    res = {'chip': a.chip, 'source': os.path.basename(a.csv), 'duration_s': round(float(tg[-1]), 1),
           'lsb_per_dps': a.lsb_per_dps, 'gyro': {}, 'acc': {}}
    curves = []
    print('%s, %.0f s at rest' % (a.chip, tg[-1]))
    for kind, x, unit, conv in (('gyro', g, 'deg/s', 1.0), ('acc', acc, 'g', 1.0)):
        out = {'N': [], 'B': [], 'B_tau_s': [], 'K': [], 'drift': [], 'quant': []}
        for i in range(3):
            tau, sig = adev(x[:, i] - x[:, i].mean(), a.fs)
            curves.append((kind, AXES[i], tau, sig))
            n, b, bt, k = fit_terms(tau, sig)
            drift = float(x[-five:, i].mean() - x[:five, i].mean())
            out['N'].append(n); out['B'].append(b); out['B_tau_s'].append(bt); out['K'].append(k)
            out['drift'].append(drift)
        if kind == 'gyro':
            res['gyro'] = {
                'white_dps_rthz': [round(v, 5) for v in out['N']],      # sigma(1 s) = density
                'arw_deg_rth': [round(v * 60, 4) for v in out['N']],
                'bias_instability_dps': [round(v, 5) for v in out['B']],
                'bias_instability_deg_h': [round(v * 3600, 2) for v in out['B']],
                'bias_tau_s': [round(v, 1) for v in out['B_tau_s']],
                'rate_walk_dps_rts': [round(v, 6) if v else None for v in out['K']],
                'drift_dps': [round(v, 4) for v in out['drift']],
                'quant_dps': round(1.0 / a.lsb_per_dps, 4),
            }
            print('gyro  white %s deg/s/rtHz (ARW %s deg/rt h), bias instability %s deg/h at %s s, '
                  'rate walk %s deg/s/rt s, drift %s deg/s'
                  % (res['gyro']['white_dps_rthz'], res['gyro']['arw_deg_rth'], res['gyro']['bias_instability_deg_h'],
                     res['gyro']['bias_tau_s'], res['gyro']['rate_walk_dps_rts'], res['gyro']['drift_dps']))
        else:
            res['acc'] = {
                'white_g_rthz': [round(v, 6) for v in out['N']],
                'vrw_mps_rth': [round(v * 9.80665 * 60, 4) for v in out['N']],
                'bias_instability_g': [round(v, 6) for v in out['B']],
                'bias_tau_s': [round(v, 1) for v in out['B_tau_s']],
                'acc_walk_g_rts': [round(v, 7) if v else None for v in out['K']],
                'drift_g': [round(v, 5) for v in out['drift']],
            }
            print('acc   white %s g/rtHz, bias instability %s g at %s s, drift %s g'
                  % (res['acc']['white_g_rthz'], res['acc']['bias_instability_g'], res['acc']['bias_tau_s'],
                     res['acc']['drift_g']))
    os.makedirs(a.out, exist_ok=True)
    dst = os.path.join(a.out, a.chip + '.json')
    json.dump(res, open(dst, 'w'), indent=1)
    print('->', dst)
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
        for kind, ax, tau, sig in curves:
            axs[0 if kind == 'gyro' else 1].loglog(tau, sig, label=ax)
        axs[0].set_title('%s gyro Allan deviation' % a.chip); axs[0].set_ylabel('deg/s')
        axs[1].set_title('%s acc Allan deviation' % a.chip); axs[1].set_ylabel('g')
        for x in axs:
            x.set_xlabel('tau s'); x.grid(True, which='both', alpha=0.3); x.legend()
        plt.tight_layout()
        plt.savefig(a.plot, dpi=80)


if __name__ == '__main__':
    main()
