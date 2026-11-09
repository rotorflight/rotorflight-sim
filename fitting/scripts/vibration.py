"""
Gyro vibration of a heli from its flight logs, by order tracking: the amplitude of the main
rotor, tail rotor and motor harmonics per axis against headspeed, and the broadband floor.

    python -I scripts/vibration.py helis/<heli>/heli.json [--plot out.png]

Needs logs with the unfiltered gyro (`gyroRAW`, logged by Rotorflight 4.6+ at any debug
mode; it is the decimated gyro, before the RPM filter, notches and low-pass) and the
headspeed. The heli file names the logs (decoded by bbl.js into FIT_DATA_DIR, as
<stem>.0.npy) and the tail and motor gear ratios; the ratios can be read off the plot
of a first run (peaks locked to the rotor at a non-integer order).

Each steady 4 s window (headspeed within 0.5 %, airborne when the log has AIRBORNE_STATE
events) gives a Welch spectrum per axis; a harmonic's power is the spectrum over +-2.5 %
of its frequency (at least 1.5 Hz) above the local floor, its amplitude sqrt(2 power).
Harmonics above 90 % of Nyquist are skipped (the decimation low-pass and aliasing).
Per harmonic and axis, amplitude = a (rpm / rpm_ref)^k exp(m (collective - coll_ref)) is
fitted over all windows (log-linear least squares; k with its standard error rpm_exp_se,
m as coll_coef per unit collective; a term is left out where its range is too small).
The relative width (RMS spread of the floor-corrected peak in the order spectrum, in
fractions of its frequency) tells how broad the peak is: the main rotor 2x is broadened
by the flap/lag modes on the M-series helis.

Writes helis/<heli>/vibration.json, which the sim's sensor model reads (heli/sensors.py).
"""
import argparse
import json
import os
import sys

import numpy as np
from scipy.signal import welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402

AXES = ('roll', 'pitch', 'yaw')
WIN = 4.0          # s
BAND = 0.025       # half width of a harmonic's band, fraction of its frequency
FLOOR = (0.04, 0.10)


def sources(heli):
    """(kind, harmonic, order): orders of the main rotor, except kind 'tailmotor' - a
    motorized tail turns at its own speed, its harmonics are orders of the tail rotor
    (the log's tailspeed)."""
    out = [('main', k, float(k)) for k in (1, 2, 3, 4)]
    tail = heli.get('tail', {})
    tr = tail.get('gear_ratio')
    if tail.get('type') == 'motor':
        out += [('tailmotor', k, float(k)) for k in (1, 2)]
    elif tr:
        out += [('tail', k, k * tr) for k in (1, 2, 3)]
    gr = heli.get('drive', {}).get('gear_ratio')
    if gr and gr != 1.0:
        out += [('motor', k, k * gr) for k in (1, 2)]
    return out


def airborne(log):
    air = np.zeros(len(log.t), bool)
    ev = [(tu, d) for tu, typ, d in log.events if typ in (52, 'AIRBORNE_STATE') and tu is not None]
    if not ev:
        return None
    t0 = (log.c('time')[0])
    start = None
    for tu, d in ev:
        state = d[0] if d else None
        ts = (tu - t0) / 1e6
        if state == 1:
            start = ts
        elif start is not None:
            air |= (log.t > start + 1) & (log.t < ts - 1)
            start = None
    if start is not None:
        air |= log.t > start + 1
    return air


def band_power(f, P, fc, df, others=()):
    """Power of the harmonic at fc above the local floor. Its band is +-BAND (at least
    1.5 Hz), but at most half the distance to the nearest other harmonic, so two close
    lines (the M6's tail 2x at 9.5 and motor 1x at 9.31 orders) split the bins between
    them instead of both counting the same peak."""
    half = max(1.5, BAND * fc)
    near = [abs(o - fc) for o in others if o != fc]
    if near:
        half = min(half, 0.5 * min(near))
    k = np.abs(f - fc) <= half
    fl = (np.abs(f - fc) >= FLOOR[0] * fc + half) & (np.abs(f - fc) <= FLOOR[1] * fc + half)
    floor = np.median(P[fl]) if fl.any() else 0.0
    return max(0.0, float(np.sum(P[k] - floor) * df)), floor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('heli')
    ap.add_argument('--plot')
    ap.add_argument('--group', help='setup group (default: the one of the trained model, else all logs)')
    a = ap.parse_args()
    heli = json.load(open(a.heli))
    group = a.group
    mpath = os.path.join(os.path.dirname(a.heli), 'model.json')
    if group is None and os.path.exists(mpath):
        group = (json.load(open(mpath)).get('provenance') or {}).get('group')
    if group:
        heli = dict(heli, logs=[e for e in heli['logs'] if e.get('group') in (None, group)])
        print('setup %s: %d logs' % (group, len(heli['logs'])))
    src = sources(heli)
    rows = []          # (rpm the source is locked to, coll, src index, axis, amp)
    floors = []        # (rpm, axis, asd 100-450 Hz)
    order_spec = {}    # rounded rpm level -> list of (orders, P) per axis
    for entry in heli['logs']:
        name = os.path.splitext(entry['file'])[0]
        try:
            log = Log(name)
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
            continue
        if 'gyroRAW[0]' not in log.N:
            print('skip %s: no gyroRAW' % name)
            continue
        fs = 1.0 / log.dt
        hs = log.c('headspeed')
        tsp = log.c('tailspeed') if 'tailspeed' in log.N else None
        coll = log.c('mixer[3]') / 1000 if 'mixer[3]' in log.N else np.zeros(len(hs))
        air = airborne(log)
        g = [log.c('gyroRAW[%d]' % i) for i in range(3)]
        n = int(WIN * fs)
        used = 0
        for i0 in range(0, len(hs) - n, n // 2):
            sl = slice(i0, i0 + n)
            h = hs[sl]
            m = h.mean()
            if m < 500 or h.std() > 0.005 * m:
                continue
            if air is not None and not air[sl].all():
                continue
            used += 1
            f0 = m / 60
            # a motorized tail counts only where its own speed is steady too
            tm = tsp[sl].mean() if tsp is not None else 0.0
            tail_ok = tsp is not None and tm > 500 and tsp[sl].std() < 0.01 * tm

            def lock(kind):
                return (tm if kind == 'tailmotor' else m), (tm / 60 if kind == 'tailmotor' else f0)
            for ax in range(3):
                f, P = welch(g[ax][sl] - g[ax][sl].mean(), fs=fs, nperseg=int(fs))
                df = f[1] - f[0]
                freqs = [o * lock(kind)[1] for kind, _, o in src if kind != 'tailmotor' or tail_ok]
                for si, (kind, _, order) in enumerate(src):
                    if kind == 'tailmotor' and not tail_ok:
                        continue
                    rpm_l, base = lock(kind)
                    fc = order * base
                    if fc > 0.45 * fs:
                        continue
                    pw, _ = band_power(f, P, fc, df, freqs)
                    rows.append((rpm_l, coll[sl].mean(), si, ax, np.sqrt(2 * pw)))
                hb = (f >= 100) & (f <= 0.45 * fs)
                for fc in freqs:
                    hb &= np.abs(f - fc) > max(3.0, 0.05 * fc)
                floors.append((m, ax, float(np.sqrt(np.median(P[hb])))))
                lvl = int(round(m, -2))
                order_spec.setdefault(lvl, [[], [], []])[ax].append((f / f0, P))
        print('%s: %d windows of %.0f s' % (name, used, WIN))
    if not rows:
        raise SystemExit('no usable windows')
    if any(kind == 'tailmotor' for kind, _, _ in src) and not any(
            src[int(r[2])][0] == 'tailmotor' for r in rows):
        print('motorized tail: its speed never held within 1 %% over %.0f s (it is the yaw control) - '
              'no tail lines; they stay in the main rotor lines they cross' % WIN)
    rows = np.array(rows)
    main_rows = np.isin(rows[:, 2], [i for i, (kind, _, _) in enumerate(src) if kind != 'tailmotor'])
    rpm_ref = float(np.median(rows[main_rows, 0]))
    coll_ref = float(np.median(rows[main_rows, 1]))
    out = {'heli': heli['name'], 'rpm_ref': round(rpm_ref), 'coll_ref': round(coll_ref, 3),
           'source': 'gyroRAW, order tracking (vibration.py)',
           'windows': int(len(rows) / 3 / len(src)) if len(src) else 0, 'harmonics': []}
    print('\nreference %.0f rpm; amplitude deg/s (peak) per axis at it, rpm exponent, relative width' % rpm_ref)
    for si, (kind, k, order) in enumerate(src):
        sel = rows[:, 2] == si
        if not sel.any():
            continue
        amp, expo, expo_se, ccoef = [], [], [], []
        ref = rpm_ref if kind != 'tailmotor' else float(np.median(rows[sel, 0]))
        for ax in range(3):
            s = sel & (rows[:, 3] == ax) & (rows[:, 4] > 0)
            if s.sum() < 5:
                amp.append(0.0); expo.append(0.0); expo_se.append(None); ccoef.append(0.0)
                continue
            # log amplitude against log rpm and collective together: collective changes
            # with headspeed in a flight, and alone it would pass for an rpm exponent
            x, y, c = np.log(rows[s, 0] / ref), np.log(rows[s, 4]), rows[s, 1] - coll_ref
            cols = [np.ones_like(x)]
            if np.ptp(x) > 0.05:
                cols.append(x)
            if np.ptp(c) > 0.1:
                cols.append(c)
            X = np.column_stack(cols)
            beta, *_ = np.linalg.lstsq(X, y, rcond=None)
            res = y - X @ beta
            cov = np.linalg.pinv(X.T @ X) * (res @ res) / max(1, len(y) - X.shape[1])
            kk = beta[1] if np.ptp(x) > 0.05 else 0.0
            se = float(np.sqrt(cov[1, 1])) if np.ptp(x) > 0.05 else None
            cc = beta[-1] if np.ptp(c) > 0.1 else 0.0
            amp.append(float(np.exp(beta[0]))); expo.append(float(kk)); expo_se.append(se); ccoef.append(float(cc))
        width = []
        for ax in range(3 if kind != 'tailmotor' else 0):
            num = den = 0.0
            for lvl, per_ax in order_spec.items():
                for o, P in per_ax[ax]:
                    w = np.abs(o - order) <= 0.08 * order
                    if not w.any():
                        continue
                    fl = np.median(P[(np.abs(o - order) > 0.05 * order) & w]) if w.sum() > 4 else 0.0
                    p = np.clip(P[w] - fl, 0, None)
                    num += np.sum(p * (o[w] - order) ** 2)
                    den += np.sum(p)
            width.append(float(np.sqrt(num / den) / order) if den > 0 else 0.0)
        width = width or [0.0, 0.0, 0.0]
        h = {'source': kind, 'n': k, 'order': round(order, 4),
             'amp_dps': [round(v, 3) for v in amp],
             'rpm_exp': [round(v, 2) for v in expo],
             'rpm_exp_se': [None if v is None else round(v, 2) for v in expo_se],
             'coll_coef': [round(v, 2) for v in ccoef],
             'rel_width': [round(v, 4) for v in width]}
        if kind == 'tailmotor':
            # locked to the tail rotor's own speed, amplitudes at its reference speed
            h.update(locked='tail', rpm_ref=round(ref))
        out['harmonics'].append(h)
        print('  %-9s %dx  order %6.3f   amp %6.2f %6.2f %6.2f   exp %+5.2f %+5.2f %+5.2f   width %.3f %.3f %.3f%s'
              % (kind, k, order, *amp, *expo, *width,
                 '   (tail rotor orders, at %.0f rpm)' % ref if kind == 'tailmotor' else ''))
    fl = np.array(floors)
    out['floor_asd_dps_rthz'] = [round(float(np.median(fl[fl[:, 1] == ax, 2])), 4) for ax in range(3)]
    print('broadband floor 100 Hz-Nyquist, deg/s/rtHz: %s' % out['floor_asd_dps_rthz'])
    dst = os.path.join(os.path.dirname(a.heli), 'vibration.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print('->', dst)

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        grid = np.linspace(0.2, 16, 3200)
        for ax in range(3):
            for lvl, per_ax in sorted(order_spec.items()):
                if not per_ax[ax]:
                    continue
                spec = np.median([np.interp(grid, o, P) for o, P in per_ax[ax]], axis=0)
                axs[ax].semilogy(grid, np.sqrt(spec), lw=0.7, label='%d rpm (%d windows)' % (lvl, len(per_ax[ax])))
            for kind, k, order in src:
                if kind == 'tailmotor':
                    continue            # not a main rotor order
                axs[ax].axvline(order, color={'main': 'k', 'tail': 'r', 'motor': 'b', 'tailmotor': 'm'}[kind], alpha=0.25, lw=0.8)
            axs[ax].set_ylabel('%s deg/s/rtHz' % AXES[ax])
            axs[ax].grid(True, which='both', alpha=0.3)
            axs[ax].legend(fontsize=8)
        axs[0].set_title('%s gyroRAW, median spectrum in rotor orders (black main, red tail, blue motor)' % heli['name'])
        axs[-1].set_xlabel('rotor order')
        plt.tight_layout()
        plt.savefig(a.plot, dpi=80)


if __name__ == '__main__':
    main()
