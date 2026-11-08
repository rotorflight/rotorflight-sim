"""
Scores of a model parameter set on held-out logs: logs no fit has seen. Each stage's
numbers predict something the held-out logs measured; lower is better everywhere.

  rotor    RMS throttle error, % (the rotor law's throttle for the logged headspeed, its
           change, collective and Vbat, over every powered sample: the spool-ups judge the
           motor torque against the rotor inertia)
  thrust   mean |hover collective error| (the thrust law against the held-out hovers,
           2 s or longer out of ground effect, per headspeed)
  rate     mean |ln(model / measured)| of the steady rate per cyclic (spectral gains with
           coherence >= 0.6, roll and pitch, per headspeed)
  flap     1 - VAF of the flap/body model on the held-out sharp inputs (fit_flap.py's
           output error with the model's K, L and tf at each window's headspeed)
  yaw      1 - R^2 of r' = Nped (w/w_n)^2 y + Nr r - Ncol (w/w_n)^2 c - torque reaction
           (firmware frame; the model's main rotor torque through its drag, band-passed
           alike) on the held-out yaw (servo tails),
           0.5-8 Hz, plus |tail trim error| in hover at the nominal headspeed
  power    |hover power error| / measured at the nominal headspeed
  pack     RMS Vbat error, V (fit_pack.py's pack model; each log's starting charge fitted)

The held-out logs' own numbers come from the same stages (fit_heli.fit, fit_flap.collect)
run on them alone: measured(), once per training.
"""
import os
import sys

import numpy as np
from scipy.signal import butter, sosfiltfilt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fit_flap  # noqa: E402
import fit_pack  # noqa: E402
import fit_heli  # noqa: E402
from model_params import flap_at, hover_collective, rate_gain, torque_terms  # noqa: E402

D2R = np.pi / 180


def measured(prep):
    """What the held-out logs say, stage by stage."""
    m = fit_heli.fit(prep, verbose=False)
    m['hover_all'] = {k: h for k, h in m['hover'].items() if h['seconds'] >= 2.0}
    per, _, dt = fit_flap.collect([p['log'] for p in prep])
    m['flap_windows'], m['flap_dt'] = per, dt
    # rotor series (airborne, decimated) and yaw regression samples
    series, yaw = [], []
    for p in prep:
        sel = np.flatnonzero(p['powered'])[::10]
        if len(sel):
            series.append(np.c_[p['w'][sel], p['wd'][sel], p['u'][sel], p['v'][sel], p['c'][sel]])
        lg = p['log']
        bp = butter(2, [0.5, 8], 'bandpass', fs=p['fs'], output='sos')
        r_ = sosfiltfilt(bp, lg.c('gyroADC[2]') * D2R)
        rd = np.gradient(r_, lg.t)
        yy = sosfiltfilt(bp, lg.c('mixer[2]') / 1000)
        cc = sosfiltfilt(bp, lg.c('mixer[3]') / 1000)
        sel5 = np.flatnonzero(p['air'] & (p['tg'] > 0))[::5]
        if len(sel5):
            # per log: the model's torque reaction is computed from the raw collective and
            # rotor speed and band-passed alike in score()
            yaw.append({'y': yy[sel5], 'r': r_[sel5], 'rd': rd[sel5], 'hs': lg.c('headspeed')[sel5],
                        'c': cc[sel5], 'c_raw': lg.c('mixer[3]') / 1000, 'w_raw': p['w'], 'sel': sel5, 'bp': bp})
    m['rotor_series'] = np.concatenate(series) if series else None
    m['pack_series'] = [x for x in (fit_pack.series(p['log']) for p in prep) if x is not None]
    m['yaw_logs'] = yaw
    return m


def model(p):
    sys.path.insert(0, os.path.join(HERE, '..', '..', 'heli'))
    from heli_model import from_params
    return from_params(p)


def score(p, m):
    """Per stage: (score or None when the held-out logs cannot judge it, detail)."""
    h = model(p)
    N = p['args']['nominal_rpm']
    out = {}

    # rotor: throttle for the logged rotor speed
    s = m['rotor_series']
    if s is not None and len(s) > 100:
        w, wd, u, v, c = s.T
        if h.motor_fit:
            A, B = h.motor_fit[:2]
            up = (B * w + w * w * np.array([h.drag_shape(x) for x in c]) + wd) / (A * np.maximum(v, 1.0))
        else:
            up = (h.B * w + h.k_d * w * w * np.array([h.drag_shape(x) for x in c]) + wd) / h.A
        err = 100 * np.sqrt(np.mean((up - u) ** 2))
        out['rotor'] = (float(err), 'RMS throttle error %.1f %% over %d samples' % (err, len(u)))
    else:
        out['rotor'] = (None, 'no powered headspeed in the held-out logs')

    # thrust: hover collective
    errs = []
    for k, hv in m['hover_all'].items():
        errs.append((int(k), hover_collective(p, int(k)) - hv['collective'], hv['seconds']))
    if errs:
        e = float(np.mean([abs(x[1]) for x in errs]))
        out['thrust'] = (e, 'hover collective error %s' % ', '.join('%+.3f at %d rpm (%.0f s)' % (x[1], x[0], x[2]) for x in errs))
    else:
        out['thrust'] = (None, 'no hover (2 s out of ground effect) in the held-out logs')

    # rate gain
    errs = []
    for k, g in m['rate_gains'].items():
        for ax in ('roll', 'pitch'):
            if ax in g and g[ax]['coherence'] >= 0.6:
                errs.append((ax, int(k), np.log(rate_gain(p, ax, int(k)) / g[ax]['gain'])))
    if errs:
        e = float(np.mean([abs(x[2]) for x in errs]))
        out['rate'] = (e, 'ln(model/measured) %s' % ', '.join('%s %d %+.2f' % x for x in errs))
    else:
        out['rate'] = (None, 'no coherent rate gain in the held-out logs')

    # flap: output error on the held-out sharp inputs
    res, tot, n = 0.0, 0.0, 0
    for (axn, lv), wins in m['flap_windows'].items():
        if lv <= 0:
            continue
        ax = 'roll' if axn == 0 else 'pitch'
        K, L, tf = flap_at(p, ax, lv)
        for u, y in wins:
            pd0 = (y[3] - y[0]) / (3 * m['flap_dt'])
            sim = fit_flap.simulate((K, L, tf, 0.0), u, y[0], pd0, m['flap_dt'])
            res += np.sum((sim - y) ** 2)
            tot += np.sum((y - y.mean()) ** 2)
            n += 1
    if n >= 3 and tot > 0:
        e = float(res / tot)
        out['flap'] = (e, '1 - VAF %.3f over %d sharp inputs' % (e, n))
    else:
        out['flap'] = (None, '%d sharp inputs in the held-out logs (needs 3)' % n)

    # yaw (servo tail)
    if h.tail_motor:
        out['yaw'] = (None, 'motorized tail')
    elif sum(len(d['sel']) for d in m['yaw_logs']) > 500:
        scale, drag = torque_terms(p)
        res, var = [], []
        for d in m['yaw_logs']:
            wr2 = (d['hs'] / N) ** 2
            # firmware frame: yaw and its rate are the model's -r, collective as is; the
            # main rotor's torque reaction (torque_yaw) responds to collective and rpm too
            torque = sosfiltfilt(d['bp'], scale * d['w_raw'] ** 2 * drag(d['c_raw']))[d['sel']]
            pred = h.Nped * wr2 * d['y'] + h.Nr * d['r'] - h.Ncol * wr2 * d['c'] - torque
            res.append(d['rd'] - pred)
            var.append(d['rd'])
        e = float(1 - np.var(np.concatenate(res)) / np.var(np.concatenate(var)))
        detail = 'R^2 %.3f' % e
        sc = 1 - e
        hv = m['hover_all'].get('%d' % N)
        if hv:
            te = abs(abs(hv['yaw_mixer']) - h.yaw_trim)
            sc += te
            detail += ', tail trim error %.3f' % te
        out['yaw'] = (float(sc), detail)
    else:
        out['yaw'] = (None, 'no yaw data in the held-out logs')

    # hover power
    hv = m['hover_all'].get('%d' % N)
    if hv and hv.get('power_w'):
        e = abs(h.hover_power - hv['power_w']) / hv['power_w']
        out['power'] = (float(e), 'hover power %.0f W vs %.0f W' % (h.hover_power, hv['power_w']))
    else:
        out['power'] = (None, 'no hover with power at %d rpm in the held-out logs' % N)

    # pack: Vbat for the drawn charge and current, each log's starting charge fitted
    if m['pack_series']:
        cells, C, R = p['args']['pack_cells'], p['args']['pack_ah'], p['args']['pack_r']
        rms = [fit_pack.fit_soc0(cells, C, R, x)[1] for x in m['pack_series']]
        e = float(np.mean(rms))
        out['pack'] = (e, 'RMS Vbat error %.2f V over %d logs' % (e, len(rms)))
    else:
        out['pack'] = (None, 'no Vbat/Ibat in the held-out logs')
    return out
