"""
From fits to a model parameter file (heli_model.load_model format, base "fitted").

Each stage of the fits proposes the model numbers it identifies, if its gates pass:

  stage    proposes                                   gates
  rotor    motor_fit (dw/dt = A u V - B w - ...)      R^2 >= 0.6 (volts form); A, B within 25 %
  thrust   thrust_inflow, Zcol (hover collective     a gated hover (5 s out of ground effect);
           against headspeed)                        collective within 10 %
  rate     rate_gain_roll / _pitch, ctrl_rpm_exp     coherence >= 0.6; gain within 20 %; the
           (steady rate per cyclic, its rpm law)      rpm law from two headspeeds 10 % apart
  flap     Lb1s / Ma1s, tf_roll / tf_pitch and the   VAF >= 0.85 over >= 5 sharp inputs; L and
           sharp inputs' rate gain K (fit_flap.py)    tf within 35 %
  yaw      Nped, Nr, Ncol, yaw_trim (servo tails)    R^2 >= 0.3; Nped within 25 %
  power    hover_power_w                             a gated hover with Vbat and Ibat
  pack     pack_r, pack_ah (fit_pack.py: Vbat/Ibat   R^2 >= 0.8; R within 30 %; C within 25 %
           against the LiPo curve)                    and a log that drew 25 % of it

"within x %" is the jackknife's relative standard error (jackknife.py). A stage whose
gates fail proposes nothing and its numbers stay at the prior. train.py then checks each
proposal against held-out logs (holdout.py) before it takes it.

The prior is what the heli has without its own fits: the preset its heli file names
("preset"), else a generic Fitted heli of its size class (servo or motor tail), with what
the heli file knows of its hardware (gear, poles, pack, rotor radius) on top.
"""
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'heli'))
from heli_model import G, MODELS, Stratos200, M7R  # noqa: E402

from jackknife import leaves  # noqa: E402

GATES = {'rotor_r2': 0.6, 'rotor_rel': 0.25, 'hover_rel': 0.10, 'coherence': 0.6, 'rate_rel': 0.20,
         'flap_vaf': 0.85, 'flap_events': 5, 'flap_rel': 0.35, 'yaw_r2': 0.3, 'yaw_rel': 0.25,
         'pack_r2': 0.8, 'pack_rel_r': 0.30, 'pack_rel_c': 0.25, 'pack_share': 0.25}

# typical rotor radius (m) and hover induced velocity (m/s) per size class
SIZES = {200: (0.21, 3.8), 380: (0.38, 3.5), 420: (0.41, 3.5), 450: (0.36, 3.4), 500: (0.52, 3.4),
         550: (0.60, 3.4), 600: (0.67, 3.3), 700: (0.79, 3.3), 800: (0.86, 3.3)}
# pack capacity (Ah) and resistance (Ohm) per size class, when the heli file has none
PACKS = {200: (0.85, 0.05), 380: (1.8, 0.03), 420: (2.2, 0.03), 450: (2.2, 0.03), 500: (3.3, 0.03),
         550: (3.3, 0.04), 600: (4.0, 0.04), 700: (4.5, 0.047), 800: (5.0, 0.047)}
STAGES = ('rotor', 'thrust', 'rate', 'flap', 'yaw', 'power', 'pack')


def nearest(table, size):
    k = min(table, key=lambda s: abs(s - (size or 550)))
    return table[k]


def prior(heli, cells=None):
    """The prior parameter dict of a heli and where its numbers come from."""
    name = (heli.get('preset') or '').lower()
    size = heli.get('size_class')
    tail = (heli.get('tail') or {}).get('type')
    if name in MODELS and name != 'blade360':
        p = copy.deepcopy(MODELS[name].params)
        src = 'preset %s' % name
    else:
        # a generic heli of its size: the M7R's (servo tail) or the Stratos' (motor tail)
        # dynamics, with the size class's rotor and pack
        base = Stratos200 if tail == 'motor' or (tail is None and size and size <= 250) else M7R
        p = copy.deepcopy(base.params)
        src = 'size prior (%s class, %s dynamics)' % (size or '?', base.__name__)
        R, vh = nearest(SIZES, size)
        p['attrs'].update(rotor_radius=R, v_hover=vh, hub_height=round(0.5 * R, 3))
        ah, r = nearest(PACKS, size)
        p['args'].update(pack_ah=ah, pack_r=r)
        # without a rotor law of its own: the generic DC motor, not another heli's fit
        p['attrs'].pop('motor_fit', None)
    p['base'] = 'fitted'
    p.pop('notes', None)
    prov = {k: src for k in list(p['args']) + list(p['attrs'])}
    hw = []
    drive, pack, rotor = heli.get('drive') or {}, heli.get('pack') or {}, heli.get('rotor') or {}
    if drive.get('gear_ratio'):
        p['args']['gear_ratio'] = drive['gear_ratio']; hw.append('gear_ratio')
    if drive.get('motor_poles'):
        p['attrs']['motor_poles'] = drive['motor_poles']; hw.append('motor_poles')
    if pack.get('cells') or cells:
        p['args']['pack_cells'] = pack.get('cells') or cells
        prov['pack_cells'] = 'heli file' if pack.get('cells') else 'estimated from Vbat'
    if pack.get('capacity_ah'):
        p['args']['pack_ah'] = pack['capacity_ah']; hw.append('pack_ah')
    if pack.get('resistance_ohm'):
        p['args']['pack_r'] = pack['resistance_ohm']; hw.append('pack_r')
    if rotor.get('radius_m'):
        p['attrs']['rotor_radius'] = rotor['radius_m']; hw.append('rotor_radius')
    if tail in ('motor', 'servo'):
        p['attrs']['tail_motor'] = tail == 'motor'; hw.append('tail_motor')
    for k in hw:
        prov[k] = 'heli file'
    return p, prov


def rpm_scale(p):
    """Exponents of the model's rpm laws (heli_model.Heli.step)."""
    a = p['attrs']
    ctrl = a.get('ctrl_rpm_exp', 2.0)
    return {'tf': p['args'].get('tf_rpm_exp', 0.0), 'stiff': p['args'].get('stiff_rpm_exp', 0.0), 'ctrl': ctrl}


def hover_collective(p, rpm):
    """Collective that carries the weight at rpm (thrust law of the model)."""
    wr = rpm / p['args']['nominal_rpm']
    ti = p['attrs'].get('thrust_inflow')
    if ti:
        K, ki = ti
        return 3.0 / (K * wr * wr) + 1.5 * ki / wr
    return G / -p['attrs']['Zcol'] / wr ** 2


def rate_gain(p, axis, rpm):
    """Steady rate per unit cyclic at rpm: K (w/w_n)^(tf_rpm_exp + ctrl_rpm_exp)."""
    wr = rpm / p['args']['nominal_rpm']
    e = rpm_scale(p)
    return p['args']['rate_gain_' + axis] * wr ** (e['tf'] + e['ctrl'])


def flap_at(p, axis, rpm):
    """(K, L, tf) of the flap/body mode at rpm."""
    wr = rpm / p['args']['nominal_rpm']
    e = rpm_scale(p)
    L = p['args']['Lb1s' if axis == 'roll' else 'Ma1s'] * wr ** e['stiff']
    tf = (p['args'].get('tf_' + axis) or p['args']['tf']) / wr ** e['tf']
    return rate_gain(p, axis, rpm), L, tf


def torque_terms(p):
    """(scale, drag(c) per w^2) of the model's main rotor torque reaction on yaw:
    r' += scale * w^2 * drag(c) (body frame, heli_model.Heli.step)."""
    from heli_model import from_params
    h = from_params(p)
    if not h.torque_yaw:
        return 0.0, lambda c: np.zeros_like(np.asarray(c, float))
    scale = h.torque_yaw * h.nominal_w / h.shaft_hover
    if h.motor_fit:
        k0, k1, n = h.motor_fit[2:5]
        return scale, lambda c: k0 + k1 * np.abs(c) ** n
    return scale, lambda c: h.k_d * (0.25 + np.abs(c) ** 1.5)


def torque_slope(p, rpm, c, eps=0.01):
    """d r'/d collective of the torque reaction at rpm and collective c (body frame)."""
    scale, drag = torque_terms(p)
    w = rpm / 60 * 2 * np.pi
    return float(scale * w * w * (drag(c + eps) - drag(c - eps)) / (2 * eps))


def nominal_rpm(fit):
    air = fit.get('airborne_s') or {}
    return int(max(air, key=air.get)) if air else None


def proposals(base, fit, flap, se, unstable, flap_se=None, pack=None):
    """Per stage: (changes {'args': {...}, 'attrs': {...}} or None, notes). base is the
    parameter dict the changes go on (its nominal_rpm and rpm laws)."""
    out = {}
    lv = leaves(fit)
    rel = lambda path: (se[path] / abs(lv[path])) if path in se and lv.get(path) else float('inf')
    N = base['args']['nominal_rpm']
    tail_motor = base['attrs'].get('tail_motor', False)

    # rotor law
    r2 = fit.get('motor_fit_r2')
    if fit.get('motor_fit') is None:
        out['rotor'] = (None, ['no rotor law (no headspeed / Vbat / throttle)'])
    elif r2 < GATES['rotor_r2']:
        out['rotor'] = (None, ['R^2 %.2f < %.2f' % (r2, GATES['rotor_r2'])])
    elif max(rel('motor_fit/0'), rel('motor_fit/1')) > GATES['rotor_rel']:
        out['rotor'] = (None, ['A or B uncertain: %.0f / %.0f %%' % (100 * rel('motor_fit/0'), 100 * rel('motor_fit/1'))])
    else:
        mf = [round(x, 6) for x in fit['motor_fit'][:4]] + [fit['motor_fit'][4]]
        out['rotor'] = ({'attrs': {'motor_fit': mf}}, ['R^2 %.3f, A %.4g +- %.0f %%, B %.4g +- %.0f %%'
                                                       % (r2, mf[0], 100 * rel('motor_fit/0'), mf[1], 100 * rel('motor_fit/1'))])

    # thrust law from the hover collective
    hv = {int(k): h for k, h in (fit.get('hover') or {}).items()
          if h['gate'] == 'ok' and 'hover/%s/collective' % k not in unstable
          and rel('hover/%s/collective' % k) <= GATES['hover_rel']}
    if not hv:
        out['thrust'] = (None, ['no gated hover with a stable collective (%s)' % ', '.join(
            '%s rpm %s s %s' % (k, h['seconds'], h['gate']) for k, h in (fit.get('hover') or {}).items()) or 'none'])
    else:
        K0, ki0 = base['attrs'].get('thrust_inflow') or (3.0 / 0.4, 0.6 * 0.4 / 1.5)
        a0, b0 = 3.0 / K0, 1.5 * ki0
        share = b0 / (a0 + b0)
        law = fit.get('inflow_law')
        if law and all(int(k) in hv for k in [law['rpm_nominal']]) and len(hv) >= 2:
            rn = law['rpm_nominal']
            a, b = law['a'] * (rn / N) ** 2, law['b'] * (rn / N)
            note = 'inflow law from %s rpm: share %.0f %%' % (', '.join(map(str, sorted(hv))), 100 * b / (a + b))
        else:
            rpm = max(hv, key=lambda k: hv[k]['seconds'])
            x = rpm / N
            cn = hv[rpm]['collective'] / ((1 - share) / x ** 2 + share / x)
            a, b = (1 - share) * cn, share * cn
            note = 'hover collective %.3f at %d rpm (%.1f s), the prior\'s inflow share %.0f %%' % (
                hv[rpm]['collective'], rpm, hv[rpm]['seconds'], 100 * share)
        if a <= 0 or b <= 0:
            out['thrust'] = (None, ['inflow law with a non-positive term (a %.3f, b %.3f)' % (a, b)])
        else:
            out['thrust'] = ({'attrs': {'thrust_inflow': [round(3.0 / a, 5), round(b / 1.5, 5)],
                                        'Zcol': round(-G / (a + b), 5)}},
                             [note, 'hover collective at %d rpm %.3f' % (N, a + b)])

    # rate gains and their rpm law
    gains = {}
    for k, g in (fit.get('rate_gains') or {}).items():
        for ax in ('roll', 'pitch'):
            path = 'rate_gains/%s/%s/gain' % (k, ax)
            if ax in g and g[ax]['coherence'] >= GATES['coherence'] and path not in unstable \
                    and rel(path) <= GATES['rate_rel']:
                gains.setdefault(ax, {})[int(k)] = (g[ax]['gain'], rel(path))
    if not gains:
        out['rate'] = (None, ['no coherent, stable rate gain'])
    else:
        ch, notes = {'args': {}, 'attrs': {}}, []
        e = rpm_scale(base)
        rpms = sorted({r for ax in gains for r in gains[ax]})
        if len(rpms) >= 2 and rpms[-1] / rpms[0] >= 1.1:
            X, y, w = [], [], []
            for ax in gains:
                for r, (g, rr) in gains[ax].items():
                    X.append([np.log(r / N), 1.0 if ax == 'roll' else 0.0, 1.0 if ax == 'pitch' else 0.0])
                    y.append(np.log(g)); w.append(1.0 / max(rr, 0.02))
            X, y, w = np.array(X), np.array(y), np.array(w)
            coef, *_ = np.linalg.lstsq(X * w[:, None], y * w, rcond=None)
            ex = float(coef[0])
            ctrl = float(np.clip(ex - e['tf'], -1.0, 3.0))
            ch['attrs']['ctrl_rpm_exp'] = round(ctrl, 3)
            notes.append('rate ~ rpm^%.2f over %s rpm -> ctrl_rpm_exp %.2f' % (ex, '/'.join(map(str, rpms)), ctrl))
            e = dict(e, ctrl=ctrl)
            for ax, c in (('roll', coef[1]), ('pitch', coef[2])):
                if ax in gains:
                    ch['args']['rate_gain_' + ax] = round(float(np.exp(c)), 3)
        else:
            for ax in gains:
                r, (g, _) = min(gains[ax].items(), key=lambda kv: abs(kv[0] - N))
                ch['args']['rate_gain_' + ax] = round(g / (r / N) ** (e['tf'] + e['ctrl']), 3)
        notes.append('at %d rpm: %s' % (N, ', '.join('%s %.2f' % (k[10:], v) for k, v in ch['args'].items())))
        out['rate'] = ({k: v for k, v in ch.items() if v}, notes)

    # flap/body mode
    ch, notes = {'args': {}}, []
    e = rpm_scale(base)
    for ax, key in (('roll', 'Lb1s'), ('pitch', 'Ma1s')):
        best = None
        for i, f in enumerate((flap or {}).get('fits', [])):
            if f['axis'] != ax or f['vaf'] < GATES['flap_vaf'] or f['events'] < GATES['flap_events']:
                continue
            fse = (flap_se or {}).get(i, {})
            rL, rt = fse.get('L', float('inf')) / f['L'], fse.get('tf', float('inf')) / f['tf']
            if max(rL, rt) > GATES['flap_rel']:
                notes.append('%s %d rpm: L or tf uncertain (%.0f / %.0f %%)' % (ax, f['rpm'], 100 * rL, 100 * rt))
                continue
            if best is None or f['events'] > best[0]['events']:
                best = (f, rL, rt)
        if best is None:
            continue
        f, rL, rt = best
        wr = f['rpm'] / N
        ch['args'][key] = round(f['L'] / wr ** e['stiff'], 1)
        ch['args']['tf_' + ax] = round(f['tf'] * wr ** e['tf'], 4)
        # the sharp inputs' own steady gain: the mode is fitted with it
        ch['args']['rate_gain_' + ax] = round(f['K'] / wr ** (e['tf'] + e['ctrl']), 3)
        notes.append('%s from %d sharp inputs at %d rpm: L %.0f +- %.0f %%, tf %.3f +- %.0f %%, VAF %.2f'
                     % (ax, f['events'], f['rpm'], f['L'], 100 * rL, f['tf'], 100 * rt, f['vaf']))
    out['flap'] = (ch if ch['args'] else None, notes or ['no sharp-input fit passes VAF >= %.2f with >= %d events'
                                                         % (GATES['flap_vaf'], GATES['flap_events'])])

    # yaw (servo tail)
    if tail_motor:
        out['yaw'] = (None, ['motorized tail: the yaw stage fits servo tails only'])
    else:
        ys = {int(k): y for k, y in (fit.get('yaw') or {}).items()
              if y['r2'] >= GATES['yaw_r2'] and 'yaw/%s/N_ped' % k not in unstable and rel('yaw/%s/N_ped' % k) <= GATES['yaw_rel']}
        ch, notes = {'attrs': {}, 'args': {}}, []
        if ys:
            r = min(ys, key=lambda k: abs(k - N))
            ch['attrs']['Nped'] = round(ys[r]['N_ped'] / (r / N) ** 2, 2)
            ch['attrs']['Nr'] = round(min(ys[r]['N_r'], -0.1), 3)
            # collective to yaw (what the firmware's precompensation cancels). The fit's N_col
            # is in the firmware frame (yaw = the model's -r) and holds all of it, the main
            # rotor's torque reaction through its drag too - which the model has already
            # (torque_yaw): Ncol is what is left, -N_col - that slope, per (w/w_n)^2
            slope = torque_slope(base, r, ys[r].get('collective', 0.0))
            ch['attrs']['Ncol'] = round((-ys[r]['N_col'] - slope) / (r / N) ** 2, 2)
            notes.append('N_ped %.1f at %d rpm (R^2 %.2f) -> Nped %.1f at %d rpm, Nr %.2f, Ncol %.1f'
                         % (ys[r]['N_ped'], r, ys[r]['r2'], ch['attrs']['Nped'], N, ch['attrs']['Nr'],
                            ch['attrs']['Ncol']))
        h = (fit.get('hover') or {}).get('%d' % N)
        if h and h['gate'] == 'ok':
            ch['args']['yaw_trim'] = round(abs(h['yaw_mixer']), 4)
            notes.append('hover tail trim %.3f at %d rpm' % (h['yaw_mixer'], N))
        ch = {k: v for k, v in ch.items() if v}
        out['yaw'] = (ch or None, notes or ['no yaw fit passes R^2 >= %.1f, no gated hover at %d rpm' % (GATES['yaw_r2'], N)])

    # hover power
    h = (fit.get('hover') or {}).get('%d' % N)
    if h and h['gate'] == 'ok' and h.get('power_w'):
        out['power'] = ({'args': {'hover_power_w': h['power_w'], 'hover_throttle': h['throttle']}},
                        ['hover %.0f W at %.1f V, throttle %.2f (%d rpm)' % (h['power_w'], h['volts'], h['throttle'], N)])
    else:
        out['power'] = (None, ['no gated hover at %d rpm with Vbat and Ibat' % N])

    # pack: capacity and resistance from Vbat/Ibat (fit_pack.py)
    if not pack:
        out['pack'] = (None, ['no Vbat/Ibat'])
    else:
        ch, notes = {'args': {}}, []
        rr = pack.get('resistance_se', float('inf')) / pack['resistance_ohm']
        rc = pack.get('capacity_se', float('inf')) / pack['capacity_ah']
        if pack['r2'] >= GATES['pack_r2'] and rr <= GATES['pack_rel_r']:
            ch['args']['pack_r'] = pack['resistance_ohm']
            notes.append('%.1f mOhm +- %.0f %% (%dS, RMS %.2f V over %d logs)'
                         % (1000 * pack['resistance_ohm'], 100 * rr, pack['cells'], pack['rms_v'], pack['logs']))
        else:
            notes.append('resistance not identified (R^2 %.2f, +- %.0f %%)' % (pack['r2'], 100 * rr))
        if ch['args'] and pack['ah_share'] >= GATES['pack_share'] and rc <= GATES['pack_rel_c']:
            ch['args']['pack_ah'] = pack['capacity_ah']
            notes.append('%.2f Ah +- %.0f %%, the largest draw %.0f %% of it'
                         % (pack['capacity_ah'], 100 * rc, 100 * pack['ah_share']))
        else:
            notes.append('capacity not identified (%.2f Ah +- %.0f %%, the largest draw %.0f %% of it)'
                         % (pack['capacity_ah'], 100 * rc, 100 * pack['ah_share']))
        out['pack'] = (ch if ch['args'] else None, notes)
    return out


def apply(p, changes):
    q = copy.deepcopy(p)
    for sec in ('args', 'attrs'):
        q.setdefault(sec, {}).update((changes or {}).get(sec, {}))
    return q
