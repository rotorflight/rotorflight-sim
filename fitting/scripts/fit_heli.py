"""
Open-loop fits of a heli from all the logs its heli file lists: the stages of
m7r_fit.py, generalised, pooled over the logs and split by headspeed.

    python -I scripts/fit_heli.py helis/<heli>/heli.json

  rotor law       dw/dt = A u V - B w - w^2 (k0 + k1 |c|^n), in the motor-volts form
                  (R^2 of u V), pooled over every powered sample - spool-ups on the ground
                  included: they identify A (fit_rotor, powered)
  segments        ground / autorotation / inverted / aerobatic / hover / cruise seconds per
                  log (segment.py)
  hover           collective, throttle, volts, yaw mixer and power per headspeed in the
                  hover segments out of ground effect, within 0.2 m/s of level (level_flight) (logs without a baro: hover of unknown
                  height, counted in height_unknown_s); gate: 5 s per headspeed, else
                  reported as insufficient. The inflow law c = a / wr^2 + b / wr from two gated
                  headspeeds or more
  rate gains      G = S_ry / S_ru per axis and headspeed (setpoint as instrument, mixer input
                  to gyro), 0.3-4 Hz, with coherence; roll and pitch: the steady gain K of the
                  flap/body magnitude K L / |L - w^2 + j w / tf| fitted to |G| (gain_fit; the
                  plain median |G| as median_gain)
  yaw             servo tail: r_dot = N_ped y + N_col c + N_r r, 0.5-8 Hz band, per headspeed
  whirl           gyro spectral peak 2-15 Hz per axis and headspeed

Writes helis/<heli>/fit.json, the input for the physical model's calibration
(PhysicalHeli) and the joint fit across helis.
"""
import json
import os
import sys

import numpy as np
from scipy.optimize import least_squares, lsq_linear
from scipy.ndimage import uniform_filter1d
from scipy.signal import butter, csd, sosfiltfilt, welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402
from segment import LABELS, airborne, has_baro, segment  # noqa: E402

HOVER_GATE_S = 5.0

D2R = np.pi / 180


def level(x, step=100):
    return np.round(x / step) * step


def targets(lg):
    """Headspeed target per sample: the governor's, else the measured headspeed in 100 rpm
    steps (helis flown on a throttle curve)."""
    tg = lg.c('govTarget') if 'govTarget' in lg.N else np.zeros(len(lg.t))
    if not np.any(tg > 0) and 'headspeed' in lg.N:
        fs = 1.0 / lg.dt
        hs = sosfiltfilt(butter(2, 0.5, fs=fs, output='sos'), lg.c('headspeed'))
        tg = np.where(hs > 500, level(hs), 0.0)
    return tg


def level_flight(lg, max_vario=0.2):
    """Samples within max_vario m/s of level (1 s mean of the baro vario; everything
    without a baro). Segmentation calls hover anything within 0.5 m/s, but a slow climb of
    0.4 m/s takes ~5 % more collective than hover (induced inflow): the hover collective
    uses the level samples only."""
    if not has_baro(lg) or 'vario' not in lg.N:
        return np.ones(len(lg.t), bool)
    fs = 1.0 / lg.dt
    return np.abs(uniform_filter1d(lg.c('vario') / 100, max(1, int(fs)))) < max_vario


def powered(lg, w):
    """Samples for the rotor law: motor powered and the rotor turning, on the ground too -
    the spool-ups and headspeed changes are what identify the motor torque against the
    rotor inertia (A); under the governor in the air the rotor speed barely moves. The
    first and last second are left out (filter edges)."""
    fs = 1.0 / lg.dt
    m = (w > 0.25 * np.percentile(w, 95)) & (lg.c('motor[0]') > 50) & (lg.c('Vbat') > 100)
    m[:int(fs)] = False
    m[-int(fs):] = False
    return m


def prepare(logs):
    """The per-log series the stages need, computed once (segmentation, filtering)."""
    prep = []
    for lg in logs:
        fs = 1.0 / lg.dt
        lab, alt, ige = segment(lg)
        lp = butter(2, 4, fs=fs, output='sos')
        w = sosfiltfilt(lp, lg.c('headspeed')) * 2 * np.pi / 60
        prep.append({
            'log': lg, 'fs': fs, 'tg': targets(lg),
            'seconds': dict({k: round(float(lab[k].sum() / fs), 1) for k in LABELS},
                            hover_in_ground_effect=round(float(ige.sum() / fs), 1)),
            'hover': lab['hover'] & ~ige & level_flight(lg), 'ige': ige, 'baro': has_baro(lg),
            'air': airborne(lg) & (lg.c('headspeed') > 500),
            'powered': powered(lg, w),
            'w': w, 'wd': np.gradient(w, lg.t),
            'u': sosfiltfilt(lp, lg.c('motor[0]') / 1000), 'v': sosfiltfilt(lp, lg.c('Vbat') / 100),
            'c': sosfiltfilt(lp, lg.c('mixer[3]') / 1000),
            'power': lg.c('Vbat') * lg.c('Ibat') / 1e4 if 'Ibat' in lg.N and np.any(lg.c('Ibat') > 0) else None,
        })
    return prep


def gain_fit(f, mag, coh):
    """Steady rate per cyclic from a frequency response: the flap/body model's magnitude
    K L / |L - w^2 + j w / tf| fitted in log over the band, weighted by coherence. The
    median |G| over 0.3-4 Hz it replaces read 15-25 % high on helis whose mode sits in the
    band (2-6 Hz): the resonance lifts it. Returns (K, L, tf), or None with too few
    coherent points."""
    ok = coh > 0.3
    if ok.sum() < 6:
        return None
    w, m, wt = 2 * np.pi * f[ok], mag[ok], np.sqrt(coh[ok])

    def res(x):
        K, L, tf = np.exp(x)
        return wt * (np.log(K * L / np.abs(L - w * w + 1j * w / tf)) - np.log(m))
    best = None
    for L0 in (150.0, 600.0, 2000.0):
        r = least_squares(res, np.log([np.median(m[:3]), L0, 0.05]),
                          bounds=(np.log([0.05, 60.0, 0.005]), np.log([200.0, 3e4, 0.5])))
        if best is None or r.cost < best.cost:
            best = r
    K, L, tf = np.exp(best.x)
    return float(K), float(L), float(tf)


def fit_rotor(w, wd, u, v, cl):
    """dw/dt = A u V - B w - w^2 (k0 + k1 |c|^n), the best n of 1, 1.5, 2, fitted in the
    motor-volts form u V = (B w + w^2 (k0 + k1 |c|^n) + dw/dt) / A: linear in (B/A, k0/A,
    k1/A, 1/A), a bounded linear least-squares solve. Under the governor the rotor speed
    barely moves and dw/dt is mostly noise: regressed on dw/dt, A and B came out only as
    their ratio (the motor's Kv), ~50x too small on a direct-drive heli. Returns (R^2, n,
    (A, B, k0, k1)), R^2 of u V."""
    best = None
    y = u * v
    for n in (1.0, 1.5, 2.0):
        X = np.c_[w, w ** 2, w ** 2 * np.abs(cl) ** n, wd]
        sc = np.abs(X).max(axis=0) + 1e-12            # columns to unit scale
        r = lsq_linear(X / sc, y, bounds=(0, np.inf))
        c = r.x / sc
        if c[3] <= 0:
            continue
        A = 1.0 / c[3]
        B, k0, k1 = c[0] * A, c[1] * A, c[2] * A
        r2 = 1 - np.var(y - X @ c) / np.var(y)
        if best is None or r2 > best[0]:
            best = (r2, n, (A, B, k0, k1))
    return best


def fit(prep, keep=None, verbose=True):
    """All stages over the prepared logs; keep (one bool mask per log, or None) restricts
    the data: the jackknife leaves blocks out with it."""
    say = print if verbose else (lambda *a, **k: None)
    keep = keep or [None] * len(prep)
    out = {'logs': [p['log'].name for p in prep]}
    fs = prep[0]['fs']
    AIR, SEG = [], []
    for p, k in zip(prep, keep):
        AIR.append(p['air'] & k if k is not None else p['air'])
        SEG.append((p['hover'] & k if k is not None else p['hover'], p['ige']))

    out['segments'] = {p['log'].name: p['seconds'] for p in prep}
    for p in prep:
        say('%s: %s' % (p['log'].name, ', '.join('%s %.0f s' % (k, v) for k, v in p['seconds'].items())))

    # ---- rotor law ------------------------------------------------------------------------
    cat = lambda xs: np.concatenate(xs)
    w, wd, u, v, cl = (cat([p[k] for p in prep]) for k in ('w', 'wd', 'u', 'v', 'c'))
    air = cat(AIR)
    pw = cat([p['powered'] & k if k is not None else p['powered'] for p, k in zip(prep, keep)])
    m = pw & (np.arange(len(w)) % 5 == 0)
    rot = fit_rotor(w[m], wd[m], u[m], v[m], cl[m]) if m.sum() > 100 else None
    if rot:
        r2, n, (A, B, k0, k1) = rot
        out['motor_fit'] = [float(A), float(B), float(k0), float(k1), n]
        out['motor_fit_r2'] = round(float(r2), 3)
        say('rotor law: motor_fit = (%.4g, %.4g, %.4g, %.4g, %.1f)  R^2 (volts form) %.3f, A/B %.0f rpm/V at the rotor'
            % (A, B, k0, k1, n, r2, A / B * 60 / (2 * np.pi)))

    # ---- per headspeed ------------------------------------------------------------------------
    hover, gains, yaw, whirl = {}, {}, {}, {}
    TG = cat([p['tg'] for p in prep])
    levels = sorted(set(level(TG[air])) - {0})
    for tg in levels:
        calm_n, ige_n, baro_n, coll_, thr_, volt_, ym_, pw_ = 0, 0, 0, [], [], [], [], []
        Syr = {0: 0, 1: 0, 2: 0}; Sur = dict(Syr); Srr = dict(Syr); Syy = dict(Syr)
        spec = {0: [], 1: [], 2: []}
        yaw_X, yaw_y = [], []
        dur = 0.0
        for p, a_, (hov, ige) in zip(prep, AIR, SEG):
            lg = p['log']
            at = np.abs(p['tg'] - tg) < 30
            k = a_ & at
            calm = hov & at
            ige_n += (ige & at).sum()
            if calm.any():
                calm_n += calm.sum()
                baro_n += calm.sum() if p['baro'] else 0
                coll_.append(lg.c('mixer[3]')[calm] / 1000); thr_.append(lg.c('motor[0]')[calm] / 1000)
                volt_.append(lg.c('Vbat')[calm] / 100); ym_.append(lg.c('mixer[2]')[calm] / 1000)
                if p['power'] is not None:
                    pw_.append(p['power'][calm])
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
                          'height_unknown_s': round((calm_n - baro_n) / fs, 1),
                          'gate': 'ok' if calm_n / fs >= HOVER_GATE_S else 'insufficient',
                          'collective': round(float(np.median(np.concatenate(coll_))), 4),
                          'throttle': round(float(np.median(np.concatenate(thr_))), 4),
                          'volts': round(float(np.median(np.concatenate(volt_))), 2),
                          'yaw_mixer': round(float(np.median(np.concatenate(ym_))), 4)}
            if pw_:
                hover[key]['power_w'] = round(float(np.median(np.concatenate(pw_))), 1)
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
            g = {'gain': round(float(np.median(np.abs(G[b]))), 3), 'coherence': round(float(np.median(coh[b])), 3)}
            if name != 'yaw':
                mf = gain_fit(f[b], np.abs(G[b]), coh[b])
                if mf:
                    g = {'gain': round(mf[0], 3), 'coherence': g['coherence'], 'median_gain': g['gain'],
                         'L': round(mf[1], 1), 'tf': round(mf[2], 4)}
            gains[key][name] = g
            P = sum(s * wt for s, wt in spec[ax]) / sum(wt for _, wt in spec[ax])
            bb = (f > 2) & (f < 15)
            cc_ = bb & (P > 0.5 * P[bb].max())
            whirl[key][name] = round(float(np.sum(f[cc_] * P[cc_]) / np.sum(P[cc_])), 2)
        if not gains[key]:
            del gains[key], whirl[key]
        if yaw_X:
            X, y = np.concatenate(yaw_X), np.concatenate(yaw_y)
            coef, *_ = np.linalg.lstsq(X, y, rcond=None)
            yaw[key] = {'N_ped': round(float(coef[0]), 2), 'N_col': round(float(coef[1]), 2),
                        'N_r': round(float(coef[2]), 3),
                        'r2': round(float(1 - np.var(y - X @ coef) / np.var(y)), 3)}
        h = hover.get(key, {})
        say('%5s rpm, %4.0f s airborne: hover %s s (%s) coll %s thr %s at %s V, yaw mixer %s | rate gain r/p/y %s | '
            'whirl %s Hz | yaw N_ped %s R^2 %s'
            % (key, dur, h.get('seconds'), h.get('gate'), h.get('collective'), h.get('throttle'), h.get('volts'),
               h.get('yaw_mixer'),
               '/'.join('%.2f(%.2f)' % (g['gain'], g['coherence']) for g in gains.get(key, {}).values()),
               '/'.join('%.1f' % x for x in whirl.get(key, {}).values()),
               yaw.get(key, {}).get('N_ped'), yaw.get(key, {}).get('r2')))
    out['hover'], out['rate_gains'], out['yaw'], out['whirl_hz'] = hover, gains, yaw, whirl
    out['airborne_s'] = {'%d' % tg: round(float((air & (np.abs(TG - tg) < 30)).sum() / fs), 1) for tg in levels}
    for key, h in hover.items():
        if key not in gains:
            say('%5s rpm: hover %s s (%s) coll %s' % (key, h['seconds'], h['gate'], h['collective']))
    hv = {int(k): v['collective'] for k, v in hover.items() if v['gate'] == 'ok'}
    if len(hv) >= 2:
        nom = max(hv, key=lambda k: hover['%d' % k]['seconds'])
        X = np.array([[1 / (tg / nom) ** 2, 1 / (tg / nom)] for tg in hv])
        y = np.array(list(hv.values()))
        (a, b), *_ = np.linalg.lstsq(X, y, rcond=None)
        out['inflow_law'] = {'rpm_nominal': nom, 'a': round(float(a), 4), 'b': round(float(b), 4),
                             'residuals': [round(float(x), 4) for x in y - X @ [a, b]]}
        say('inflow law at %d rpm: a %.3f b %.3f (inflow share %.0f %%), residuals %s'
            % (nom, a, b, 100 * b / (a + b), out['inflow_law']['residuals']))
    else:
        say('inflow law: needs gated hover at two headspeeds (have %s)' % (sorted(hv) or 'none'))
    return out


def log_names(heli):
    return [e['file'][:-4] if e['file'].endswith('.bbl') else e['file'] for e in heli['logs']]


def load_logs(names):
    logs = []
    for name in names:
        try:
            logs.append(Log(name))
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
    return logs


def main():
    heli = json.load(open(sys.argv[1]))
    logs = load_logs(log_names(heli))
    if not logs:
        raise SystemExit('no logs')
    out = dict({'heli': heli['name']}, **fit(prepare(logs)))
    dst = os.path.join(os.path.dirname(sys.argv[1]), 'fit.json')
    json.dump(out, open(dst, 'w'), indent=1)
    print('->', dst)


if __name__ == '__main__':
    main()
