"""
The pack from logged Vbat and Ibat: capacity and internal resistance against the model's
LiPo curve.

    python -I scripts/fit_pack.py helis/<heli>/heli.json [CELLS]     # -> helis/<heli>/pack.json

The model's pack (heli_model.Heli.pack_voltage):

    V = cells * OCV(soc) - I * R * (1 + 1.5 (1 - soc)^4),   soc = soc0 - Ah(t) / C

OCV the open-circuit voltage per cell against the state of charge (heli_model.LIPO_OCV),
Ah(t) the charge drawn since the log started (the integral of Ibat), R the internal
resistance rising towards empty. C and R are shared by all logs; soc0 is each log's own
(a pack is not always full at take-off). Fitted by least squares on the powered samples,
low-passed to 0.5 Hz and taken at 2 Hz.

C is identified only where the logs draw a good part of the pack (ah_share: the largest
charge drawn in one log over C), R where the current varies. Their standard errors come
from leaving one log out at a time (three logs or more).
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import butter, sosfiltfilt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'heli'))
from heli_model import LIPO_OCV  # noqa: E402

OCV_X = np.linspace(0.0, 1.0, len(LIPO_OCV))
OCV_Y = np.array(LIPO_OCV)


def series(log):
    """(Ah drawn, I in A, V in V) of a log's powered samples at 2 Hz, or None."""
    if 'Vbat' not in log.N or 'Ibat' not in log.N:
        return None
    fs = 1.0 / log.dt
    v, i = log.c('Vbat') / 100, log.c('Ibat') / 100
    if not np.any(i > 0.5) or not np.any(v > 1):
        return None
    lp = butter(2, 0.5, fs=fs, output='sos')
    vf, iff = sosfiltfilt(lp, v), sosfiltfilt(lp, i)
    ah = np.concatenate([[0.0], np.cumsum(0.5 * (i[1:] + i[:-1]) * np.diff(log.t))]) / 3600
    m = (v > 1) & (iff > 0.2)
    m[:int(2 * fs)] = False
    m[-int(2 * fs):] = False
    sel = np.flatnonzero(m)[::max(1, int(fs / 2))]
    if len(sel) < 20:
        return None
    return ah[sel], iff[sel], vf[sel]


def voltage(cells, C, R, soc0, ah, i):
    soc = np.clip(soc0 - ah / C, 0.0, 1.0)
    return cells * np.interp(soc, OCV_X, OCV_Y) - i * R * (1 + 1.5 * (1 - soc) ** 4)


def fit_soc0(cells, C, R, s):
    """The starting charge of one log for a given pack: (soc0, RMS volts)."""
    ah, i, v = s
    r = minimize_scalar(lambda x: np.mean((voltage(cells, C, R, x, ah, i) - v) ** 2),
                        bounds=(0.02, 1.05), method='bounded')
    return r.x, float(np.sqrt(r.fun))


def fit(ss, cells, x0=None):
    """C, R and one soc0 per series. Returns a dict."""
    n = len(ss)

    def res(x):
        C, R = np.exp(x[0]), np.exp(x[1])
        return np.concatenate([voltage(cells, C, R, x[2 + k], s[0], s[1]) - s[2] for k, s in enumerate(ss)])
    # starting charge and capacity trade off when the logs draw little near the top of the
    # curve: a grid of starts (a single one found a minimum 4000x worse on clean data)
    if x0:
        starts = [(x0['capacity_ah'], x0['resistance_ohm'], q) for q in (0.7, 1.0)]
    else:
        starts = [(C0, R0, q) for C0 in (0.6, 2.0, 6.0) for R0 in (0.01, 0.06) for q in (0.6, 0.85, 1.0)]
    best = None
    for C0, R0, q in starts:
        st = np.r_[np.log(C0), np.log(R0), np.full(n, q)]
        r = least_squares(res, st, bounds=(np.r_[np.log(0.1), np.log(1e-4), np.full(n, 0.02)],
                                           np.r_[np.log(30.0), np.log(1.0), np.full(n, 1.05)]))
        if best is None or r.cost < best.cost:
            best = r
    C, R = float(np.exp(best.x[0])), float(np.exp(best.x[1]))
    v = np.concatenate([s[2] for s in ss])
    return {'capacity_ah': round(C, 3), 'resistance_ohm': round(R, 5), 'cells': cells,
            'rms_v': round(float(np.sqrt(np.mean(best.fun ** 2))), 3),
            'r2': round(float(1 - np.var(best.fun) / np.var(v)), 3),
            'soc0': [round(float(x), 3) for x in best.x[2:]],
            'ah_share': round(float(max(s[0][-1] for s in ss) / C), 3), 'logs': n}


def fit_logs(logs, cells, say=print):
    """The pack of a set of logs with leave-one-log-out standard errors; None without
    Vbat/Ibat."""
    ss = [s for s in (series(lg) for lg in logs) if s is not None]
    if not ss or not cells:
        return None
    out = fit(ss, cells)
    if len(ss) >= 3:
        reps = [fit(ss[:k] + ss[k + 1:], cells, x0=out) for k in range(len(ss))]
        n = len(reps)
        for q, key in (('capacity_ah', 'capacity_se'), ('resistance_ohm', 'resistance_se')):
            x = np.array([r[q] for r in reps])
            out[key] = round(float(np.sqrt((n - 1) / n * np.sum((x - x.mean()) ** 2))), 5)
    say('pack %dS: %.2f Ah%s, %.1f mOhm%s, RMS %.2f V, %d logs, largest draw %.0f %% of the pack'
        % (cells, out['capacity_ah'], (' +- %.2f' % out['capacity_se']) if 'capacity_se' in out else '',
           1000 * out['resistance_ohm'], (' +- %.1f' % (1000 * out['resistance_se'])) if 'resistance_se' in out else '',
           out['rms_v'], out['logs'], 100 * out['ah_share']))
    return out


def main():
    import fit_heli
    heli = json.load(open(sys.argv[1]))
    cells = int(sys.argv[2]) if len(sys.argv) > 2 else (heli.get('pack') or {}).get('cells')
    out = fit_logs(fit_heli.load_logs(fit_heli.log_names(heli)), cells)
    dst = os.path.join(os.path.dirname(sys.argv[1]), 'pack.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print('->', dst)


if __name__ == '__main__':
    main()
