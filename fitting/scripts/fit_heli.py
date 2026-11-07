"""
Open-loop fits of a heli from all the logs its heli file lists: the stages of
m7r_fit.py, generalised, pooled over the logs and split by headspeed.

    python -I scripts/fit_heli.py helis/<heli>/heli.json

  rotor law       dw/dt = A u V - B w - w^2 (k0 + k1 |c|^n), in the motor-volts form
                  (R^2 of A u V against the rest), pooled over all airborne data
  segments        ground / autorotation / inverted / aerobatic / hover / cruise seconds per
                  log (segment.py)
  hover           collective, throttle, volts and yaw mixer per headspeed in the hover
                  segments out of ground effect; gate: 5 s per headspeed, else reported as
                  insufficient. The inflow law c = a / wr^2 + b / wr from two gated
                  headspeeds or more
  rate gains      |S_ry / S_ru| per axis and headspeed (setpoint as instrument, mixer input
                  to gyro), 0.3-4 Hz, with coherence
  yaw             servo tail: r_dot = N_ped y + N_col c + N_r r, 0.5-8 Hz band, per headspeed
  whirl           gyro spectral peak 2-15 Hz per axis and headspeed

Writes helis/<heli>/fit.json, the input for the physical model's calibration
(PhysicalHeli) and the joint fit across helis.
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, csd, sosfiltfilt, welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402
from segment import LABELS, airborne, segment  # noqa: E402

HOVER_GATE_S = 5.0

D2R = np.pi / 180


def level(x, step=100):
    return np.round(x / step) * step


def main():
    heli = json.load(open(sys.argv[1]))
    logs = []
    for entry in heli['logs']:
        name = os.path.splitext(entry['file'])[0]
        try:
            logs.append(Log(name))
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
    if not logs:
        raise SystemExit('no logs')
    out = {'heli': heli['name'], 'logs': [lg.name for lg in logs]}
    fs = 1.0 / logs[0].dt

    # ---- segments -----------------------------------------------------------------------
    SEG = []
    out['segments'] = {}
    for lg in logs:
        lab, alt, ige = segment(lg)
        SEG.append((lab['hover'] & ~ige, ige))
        out['segments'][lg.name] = {k: round(float(lab[k].sum() / fs), 1) for k in LABELS}
        out['segments'][lg.name]['hover_in_ground_effect'] = round(float(ige.sum() / fs), 1)
        print('%s: %s' % (lg.name, ', '.join('%s %.0f s' % (k, v) for k, v in out['segments'][lg.name].items())))

    # ---- pooled series ------------------------------------------------------------------
    lp = butter(2, 4, fs=fs, output='sos')
    W, WD, U, V, C, AIR, TG = [], [], [], [], [], [], []
    for lg in logs:
        air = airborne(lg)
        w = sosfiltfilt(lp, lg.c('headspeed')) * 2 * np.pi / 60
        W.append(w); WD.append(np.gradient(w, lg.t))
        U.append(sosfiltfilt(lp, lg.c('motor[0]') / 1000)); V.append(sosfiltfilt(lp, lg.c('Vbat') / 100))
        C.append(sosfiltfilt(lp, lg.c('mixer[3]') / 1000)); AIR.append(air & (lg.c('headspeed') > 500))
        TG.append(lg.c('govTarget'))
    cat = lambda xs: np.concatenate(xs)
    w, wd, u, v, cl, air = cat(W), cat(WD), cat(U), cat(V), cat(C), cat(AIR)
    m = air & (np.arange(len(w)) % 10 == 0)

    # ---- rotor law ------------------------------------------------------------------------
    best = None
    for n in (1.0, 1.5, 2.0):
        def res(p):
            A, B, k0, k1 = p
            return (A * u[m] * v[m] - B * w[m] - w[m] ** 2 * (k0 + k1 * np.abs(cl[m]) ** n) - wd[m]) / 100
        r = least_squares(res, [3, 0.3, 5e-4, 5e-4], bounds=([0] * 4, [np.inf] * 4))
        A, B, k0, k1 = r.x
        lhs = A * u[m] * v[m]
        rhs = B * w[m] + w[m] ** 2 * (k0 + k1 * np.abs(cl[m]) ** n) + wd[m]
        r2 = 1 - np.var(lhs - rhs) / np.var(lhs)
        if best is None or r2 > best[0]:
            best = (r2, n, r.x)
    r2, n, (A, B, k0, k1) = best
    out['motor_fit'] = [float(A), float(B), float(k0), float(k1), n]
    out['motor_fit_r2'] = round(float(r2), 3)
    print('rotor law: motor_fit = (%.4g, %.4g, %.4g, %.4g, %.1f)  R^2 (volts form) %.3f, A/B %.0f rpm/V at the rotor'
          % (A, B, k0, k1, n, r2, A / B * 60 / (2 * np.pi)))

    # ---- per headspeed ------------------------------------------------------------------------
    hover, gains, yaw, whirl = {}, {}, {}, {}
    levels = sorted(set(level(cat(TG)[air])) - {0})
    for tg in levels:
        calm_n, ige_n, coll_, thr_, volt_, ym_ = 0, 0, [], [], [], []
        Syr = {0: 0, 1: 0, 2: 0}; Sur = dict(Syr); Srr = dict(Syr); Syy = dict(Syr)
        spec = {0: [], 1: [], 2: []}
        yaw_X, yaw_y = [], []
        dur = 0.0
        for lg, a_, (hov, ige) in zip(logs, AIR, SEG):
            k = a_ & (np.abs(lg.c('govTarget') - tg) < 30)
            calm = hov & (np.abs(lg.c('govTarget') - tg) < 30)
            ige_n += (ige & (np.abs(lg.c('govTarget') - tg) < 30)).sum()
            if calm.any():
                calm_n += calm.sum()
                coll_.append(lg.c('mixer[3]')[calm] / 1000); thr_.append(lg.c('motor[0]')[calm] / 1000)
                volt_.append(lg.c('Vbat')[calm] / 100); ym_.append(lg.c('mixer[2]')[calm] / 1000)
            if k.sum() < 4 * fs:
                continue
            dur += k.sum() / fs
            # contiguous stretches for the spectra
            idx = np.flatnonzero(k)
            for seg in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
                if len(seg) < 4096:
                    continue
                for ax in (0, 1, 2):
                    r = lg.c('setpoint[%d]' % ax)[seg] * D2R
                    uu = lg.c('mixer[%d]' % ax)[seg] / 1000
                    y = lg.c('gyroADC[%d]' % ax)[seg] * D2R
                    f, s_yr = csd(r, y, fs=fs, nperseg=4096)
                    _, s_ur = csd(r, uu, fs=fs, nperseg=4096)
                    _, s_rr = welch(r, fs=fs, nperseg=4096)
                    _, s_yy = welch(y, fs=fs, nperseg=4096)
                    wgt = len(seg)
                    Syr[ax] = Syr[ax] + s_yr * wgt; Sur[ax] = Sur[ax] + s_ur * wgt
                    Srr[ax] = Srr[ax] + s_rr * wgt; Syy[ax] = Syy[ax] + s_yy * wgt
                    spec[ax].append((s_yy, wgt))
            # yaw regression
            bp = butter(2, [0.5, 8], 'bandpass', fs=fs, output='sos')
            r_ = sosfiltfilt(bp, lg.c('gyroADC[2]') * D2R)
            rd = np.gradient(r_, lg.t)
            yy = sosfiltfilt(bp, lg.c('mixer[2]') / 1000)
            cc = sosfiltfilt(bp, lg.c('mixer[3]') / 1000)
            sel = np.flatnonzero(k)[::5]
            yaw_X.append(np.c_[yy[sel], cc[sel], r_[sel]]); yaw_y.append(rd[sel])
        key = '%d' % tg
        if calm_n:
            hover[key] = {'seconds': round(calm_n / fs, 1),
                          'in_ground_effect_s': round(ige_n / fs, 1),
                          'gate': 'ok' if calm_n / fs >= HOVER_GATE_S else 'insufficient',
                          'collective': round(float(np.median(np.concatenate(coll_))), 4),
                          'throttle': round(float(np.median(np.concatenate(thr_))), 4),
                          'volts': round(float(np.median(np.concatenate(volt_))), 2),
                          'yaw_mixer': round(float(np.median(np.concatenate(ym_))), 4)}
        if dur < 10:
            continue
        gains[key] = {}
        whirl[key] = {}
        for ax, name in ((0, 'roll'), (1, 'pitch'), (2, 'yaw')):
            if isinstance(Syr[ax], int):
                continue
            G = Syr[ax] / Sur[ax]
            coh = np.abs(Syr[ax]) ** 2 / (Srr[ax] * Syy[ax])
            b = (f >= 0.3) & (f < 4.0)
            gains[key][name] = {'gain': round(float(np.median(np.abs(G[b]))), 3),
                                'coherence': round(float(np.median(coh[b])), 3)}
            P = sum(s * wt for s, wt in spec[ax]) / sum(wt for _, wt in spec[ax])
            bb = (f > 2) & (f < 15)
            cc_ = bb & (P > 0.5 * P[bb].max())
            whirl[key][name] = round(float(np.sum(f[cc_] * P[cc_]) / np.sum(P[cc_])), 2)
        if yaw_X:
            X, y = np.concatenate(yaw_X), np.concatenate(yaw_y)
            coef, *_ = np.linalg.lstsq(X, y, rcond=None)
            yaw[key] = {'N_ped': round(float(coef[0]), 2), 'N_col': round(float(coef[1]), 2),
                        'N_r': round(float(coef[2]), 3),
                        'r2': round(float(1 - np.var(y - X @ coef) / np.var(y)), 3)}
        h = hover.get(key, {})
        print('%5s rpm, %4.0f s airborne: hover %s s (%s) coll %s thr %s at %s V, yaw mixer %s | rate gain r/p/y %s | '
              'whirl %s Hz | yaw N_ped %s R^2 %s'
              % (key, dur, h.get('seconds'), h.get('gate'), h.get('collective'), h.get('throttle'), h.get('volts'),
                 h.get('yaw_mixer'),
                 '/'.join('%.2f(%.2f)' % (g['gain'], g['coherence']) for g in gains[key].values()),
                 '/'.join('%.1f' % x for x in whirl[key].values()),
                 yaw.get(key, {}).get('N_ped'), yaw.get(key, {}).get('r2')))
    out['hover'], out['rate_gains'], out['yaw'], out['whirl_hz'] = hover, gains, yaw, whirl
    for key, h in hover.items():
        if key not in gains:
            print('%5s rpm: hover %s s (%s) coll %s' % (key, h['seconds'], h['gate'], h['collective']))
    hv = {int(k): v['collective'] for k, v in hover.items() if v['gate'] == 'ok'}
    if len(hv) >= 2:
        nom = max(hv, key=lambda k: hover['%d' % k]['seconds'])
        X = np.array([[1 / (tg / nom) ** 2, 1 / (tg / nom)] for tg in hv])
        y = np.array(list(hv.values()))
        (a, b), *_ = np.linalg.lstsq(X, y, rcond=None)
        out['inflow_law'] = {'rpm_nominal': nom, 'a': round(float(a), 4), 'b': round(float(b), 4),
                             'residuals': [round(float(x), 4) for x in y - X @ [a, b]]}
        print('inflow law at %d rpm: a %.3f b %.3f (inflow share %.0f %%), residuals %s'
              % (nom, a, b, 100 * b / (a + b), out['inflow_law']['residuals']))
    else:
        print('inflow law: needs gated hover at two headspeeds (have %s)' % (sorted(hv) or 'none'))
    dst = os.path.join(os.path.dirname(sys.argv[1]), 'fit.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print('->', dst)


if __name__ == '__main__':
    main()
