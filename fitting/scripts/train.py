"""
Train a heli's model from its logs: fit, check against held-out logs, keep what is better.

    python -I scripts/train.py helis/<heli>/heli.json [--group ID] [--folds 8] [--dry-run]
                               [--elf <rotorflight_SITL>]  (closed-loop replay check, replay_check.py)

One command per heli, run whenever logs are added (intake.py). Steps:

 1. Setup: the logs are grouped by their setup fingerprint (fingerprint.py, or a "group"
    set by hand); the newest setup with three flights or more is trained (else the one
    with the most flights) unless --group names another. Logs of other groups are left
    alone: a changed heli does not pull the old one's numbers.
 2. Split: each log is "fit" or "validate" ("role" in the heli file). New logs get a role
    from their hash (one in four validates), so the split is stable as logs are added and
    can be changed by hand. Validate logs are never fitted.
 3. Fit: fit_heli.py's stages and fit_flap.py's sharp-input fit on the fit logs, with a
    jackknife over 20 s blocks (jackknife.py) for the standard error of every number.
    -> fit.json, flap.json (with "uncertainty"); fit_pack.py's pack -> pack.json
 4. Propose: each stage whose gates pass proposes its model numbers (model_params.py),
    on top of the current model (model.json, else the prior: the heli file's preset or a
    generic heli of its size).
 5. Validate: each proposal is scored on the held-out logs (holdout.py) against the
    current numbers, stage by stage in the order rotor, thrust, rate, flap, yaw, power,
    pack. A
    proposal is taken if it scores no worse on its own stage, and no more than 2 % worse
    on every other one (a stage's numbers feed other predictions: the rate gain is the
    flap model's K). Stages that depend on the one judged (yaw on the rotor law's torque
    reaction, the rotor law on the thrust law's hover point) are proposed again on top of
    it first, so a change is judged with what they will refit. One that the held-out logs cannot judge is taken only over a size prior (a
    guess), never over a preset or an earlier fit.
 6. Optional (--elf): the held-out logs' sticks are replayed through the SITL with the
    current and the new model (replay_check.py); the new model must track the real
    flight no worse. If it does not, the taken stages are dropped from the last one back (at
    most three) and replayed again; if none passes, nothing changes.
 7. Write: model.json (base "fitted", every number with its provenance) if anything
    changed, and train.json / TRAINING.md: the report - what each stage proposed, the
    held-out scores before and after, what was taken and why.

The sim flies a trained heli by name: heli_bridge.py --model <heli>.
"""
import argparse
import copy
import datetime
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fit_flap  # noqa: E402
import fit_heli  # noqa: E402
import fit_pack  # noqa: E402
import holdout  # noqa: E402
import jackknife  # noqa: E402
import model_params as mp  # noqa: E402
from common import Log  # noqa: E402
from fingerprint import KEY, differences, group_id, setup  # noqa: E402
from logstore import decoded_name, save  # noqa: E402

# stages whose proposals depend on another's numbers: re-proposed before judging it
DEPENDS = {'rotor': ('yaw', 'power'), 'thrust': ('rotor', 'yaw', 'power')}
MAX_BACKOFF = 3     # stages the replay check may drop, last taken first
TOL = 0.02          # relative: how much worse another stage may score (noise); its own may not
VALIDATE_EVERY = 4  # one log in four is held out
MIN_AIR_S = 20.0    # a log shorter than this in the air does not count as a flight


def starting_esc_tau(heli, folder, group):
    """The ESC lag (esc_tau) of the model training starts from - its model.json for this
    setup, else the prior. The rotor law is fitted and scored behind it, and no stage
    changes it, so every candidate's motor_fit goes with the lag it was fitted for."""
    path = os.path.join(folder, 'model.json')
    if os.path.exists(path):
        m = json.load(open(path))
        if (m.get('provenance') or {}).get('group') == group:
            return float((m.get('attrs') or {}).get('esc_tau', 0.0) or 0.0)
    prior, _ = mp.prior(heli, None)
    return float(prior['attrs'].get('esc_tau', 0.0) or 0.0)

def role_for(entry):
    h = entry.get('sha256') or hashlib.sha256(decoded_name(entry).encode()).hexdigest()
    key = hashlib.sha256((h + decoded_name(entry)).encode()).hexdigest()
    return 'validate' if int(key[:8], 16) % VALIDATE_EVERY == 0 else 'fit'


def air_s(entry):
    return (entry.get('intake') or {}).get('air_s', MIN_AIR_S)


def select(heli, group=None, say=print):
    """The logs of one setup group; groups the logs that have none."""
    groups = heli.setdefault('groups', {})
    for e in heli['logs']:
        if e.get('group'):
            continue
        try:
            st, info = setup(Log(decoded_name(e)))
        except SystemExit as err:
            say('skip %s: %s' % (decoded_name(e), err))
            continue
        e['group'] = group_id(st)
        groups.setdefault(e['group'], st)
        e.setdefault('start', info.get('log_start', ''))
    have = [e for e in heli['logs'] if e.get('group')]
    if not have:
        raise SystemExit('no usable logs')
    if group is None:
        # the newest setup with three flights or more (one can be held out), else the one
        # with the most flights
        flights = {}
        for e in have:
            if air_s(e) >= MIN_AIR_S:
                flights.setdefault(e['group'], []).append(e)
        enough = [g for g, es in flights.items() if len(es) >= 3]
        if enough:
            group = max(enough, key=lambda g: max(e.get('start') or '' for e in flights[g]))
        elif flights:
            group = max(flights, key=lambda g: len(flights[g]))
        else:
            group = max(have, key=lambda e: e.get('start') or '')['group']
    sel = [e for e in have if e['group'] == group]
    others = sorted({e['group'] for e in have} - {group})
    for g in others:
        n = sum(e['group'] == g for e in have)
        d = differences(groups.get(group, {}), groups.get(g, {}))
        say('setup %s: %d log(s) not used (%s)' % (g, n, '; '.join(d) or 'set by hand'))
    return group, sel


def assign_roles(entries):
    flights = [e for e in entries if air_s(e) >= MIN_AIR_S]
    for e in entries:
        if 'role' not in e:
            e['role'] = role_for(e) if e in flights else 'fit'
    val = [e for e in flights if e['role'] == 'validate']
    fit = [e for e in flights if e['role'] == 'fit']
    if not val and len(flights) >= 3:
        # always hold one flight out once there are three: the one with the smallest hash key
        e = min(flights, key=lambda e: hashlib.sha256(decoded_name(e).encode()).hexdigest())
        e['role'] = 'validate'
    if not fit and val:
        val[0]['role'] = 'fit'


def flap_fits(logs, K, say=print):
    """fit_flap per (axis, headspeed) with >= 3 sharp inputs, and the jackknife standard
    errors of K, L, tf over K folds of the windows (warm-started refits)."""
    per, _, dt = fit_flap.collect(logs)
    out, se = {'fits': []}, {}
    for (ax, lv), wins in sorted(per.items()):
        if len(wins) < 3 or lv <= 0:
            continue
        r = fit_flap.fit(wins, dt)
        r.update(axis='roll' if ax == 0 else 'pitch', rpm=lv)
        i = len(out['fits'])
        out['fits'].append(r)
        k = min(K, len(wins))
        if k >= 3:
            reps = []
            for j in range(k):
                sub = [w for n, w in enumerate(wins) if n % k != j]
                reps.append(fit_flap.fit(sub, dt, x0=r))
            se[i] = {q: float(np.sqrt((k - 1) / k * np.sum((np.array([x[q] for x in reps]) -
                                                            np.mean([x[q] for x in reps])) ** 2)))
                     for q in ('K', 'L', 'tf')}
            r['se'] = {q: round(v, 4) for q, v in se[i].items()}
        say('flap %-5s %5d rpm: %2d inputs, K %.2f L %.0f tf %.3f VAF %.2f%s'
            % (r['axis'], lv, r['events'], r['K'], r['L'], r['tf'], r['vaf'],
               ('  (+- L %.0f %%, tf %.0f %%)' % (100 * se[i]['L'] / r['L'], 100 * se[i]['tf'] / r['tf'])) if i in se else ''))
    return out, se


def keys_of(changes):
    return [k for sec in ('args', 'attrs') for k in (changes or {}).get(sec, {})]


def fmt(x):
    return '-' if x is None else '%.4g' % x


def train(heli_path, group=None, folds=8, dry=False, elf=None, say=print):
    heli = json.load(open(heli_path))
    folder = os.path.dirname(os.path.abspath(heli_path))
    group, entries = select(heli, group, say)
    assign_roles(entries)
    if not dry:
        save(heli, heli_path)
    fit_names = [decoded_name(e) for e in entries if e['role'] == 'fit' and air_s(e) >= MIN_AIR_S]
    val_names = [decoded_name(e) for e in entries if e['role'] == 'validate']
    say('setup %s (%s): %d fit, %d validate logs'
        % (group, ', '.join(str(heli['groups'][group].get(k)) for k in KEY), len(fit_names), len(val_names)))
    logs = fit_heli.load_logs(fit_names)
    if not logs:
        raise SystemExit('no logs to fit')
    esc_tau = starting_esc_tau(heli, folder, group)
    if esc_tau:
        say('rotor law fitted and scored behind the ESC lag of the starting model, %.3f s' % esc_tau)
    prep = fit_heli.prepare(logs, esc_tau=esc_tau)

    # ---- fit with uncertainty -----------------------------------------------------------
    full = fit_heli.fit(prep, verbose=False)
    reps = [fit_heli.fit(prep, keep=k, verbose=False) for k in jackknife.folds(prep, folds)]
    se, unstable = jackknife.spread(full, reps)
    full['uncertainty'] = {p: round(v, 6) for p, v in se.items()
                           if p.startswith(('motor_fit', 'hover/', 'rate_gains/', 'yaw/', 'inflow_law/'))
                           and not p.endswith(('seconds', '_s'))}
    full['unstable'] = [p for p in unstable if not p.startswith('segments/')]
    full['folds'] = folds
    flap, flap_se = flap_fits(logs, folds, say)
    cells = [setup(lg)[1].get('cells_estimate') for lg in logs]
    cells = int(np.median([c for c in cells if c])) if any(cells) else None
    if not dry:
        json.dump(dict({'heli': heli['name'], 'group': group}, **full), open(os.path.join(folder, 'fit.json'), 'w'), indent=1)
        json.dump(dict({'heli': heli['name'], 'group': group}, **flap), open(os.path.join(folder, 'flap.json'), 'w'), indent=1)

    # ---- prior and current ------------------------------------------------------------
    prior, prov = mp.prior(heli, cells)
    N = mp.nominal_rpm(full)
    if N and not (heli.get('preset') or '').lower() in mp.MODELS:
        prior['args']['nominal_rpm'] = float(N)
        prov['nominal_rpm'] = 'most flown headspeed'
    model_path = os.path.join(folder, 'model.json')
    current, cur_prov, note = prior, dict(prov), 'prior: %s' % next(iter(prov.values()))
    if os.path.exists(model_path):
        m = json.load(open(model_path))
        if (m.get('provenance') or {}).get('group') == group:
            current = {k: m[k] for k in ('base', 'args', 'attrs')}
            cur_prov = dict(m['provenance'].get('params', {}))
            note = 'model.json of %s' % m['provenance'].get('trained', '?')
        else:
            note = 'prior (model.json is of another setup, %s)' % (m.get('provenance') or {}).get('group')
    say('starting from the %s, nominal %d rpm' % (note, current['args']['nominal_rpm']))
    pack = fit_pack.fit_logs(logs, current['args'].get('pack_cells'), say)
    if not dry and pack:
        json.dump(dict({'heli': heli['name'], 'group': group}, **pack), open(os.path.join(folder, 'pack.json'), 'w'), indent=1)

    # ---- held-out logs ----------------------------------------------------------------
    vlogs = fit_heli.load_logs(val_names)
    meas = holdout.measured(fit_heli.prepare(vlogs, esc_tau=esc_tau)) if vlogs else None
    score = (lambda p: holdout.score(p, meas)) if meas else (lambda p: {s: (None, 'no held-out logs') for s in mp.STAGES})

    # ---- proposals, stage by stage ------------------------------------------------------
    # proposals are made against the model as it stands when their stage comes up: the yaw
    # stage takes the torque path of the rotor law the rotor stage may just have changed
    cand, cand_prov = copy.deepcopy(current), dict(cur_prov)
    today = datetime.date.today().isoformat()
    stages = {}
    for st in mp.STAGES:
        ch, notes = mp.proposals(cand, full, flap, se, unstable, flap_se, pack)[st]
        rec = {'proposal': ch, 'notes': notes}
        if ch is None:
            rec['decision'] = 'no proposal'
        else:
            trial = mp.apply(cand, ch)
            sb, sa = score(cand), score(trial)
            before, after = sb[st], sa[st]
            rec['score_before'], rec['score_after'] = before[0], after[0]
            rec['score_detail'] = {'before': before[1], 'after': after[1]}
            # never worse anywhere: the numbers of one stage feed others' predictions (the
            # rate gain is the flap model's K). A stage that depends on this one is first
            # proposed again on top of it (a new rotor law moves the torque reaction the
            # yaw stage's Ncol is taken against): judged with what it will refit
            look, refit = trial, []
            for d in DEPENDS.get(st, ()):
                chd = mp.proposals(look, full, flap, se, unstable, flap_se, pack)[d][0]
                if chd:
                    look = mp.apply(look, chd)
                    refit.append(d)
            sl = score(look) if refit else sa
            worse = ['%s %s -> %s' % (o, fmt(sb[o][0]), fmt(sl[o][0])) for o in mp.STAGES
                     if o != st and sb[o][0] is not None and sl[o][0] is not None
                     and sl[o][0] > sb[o][0] * (1 + TOL) + 1e-9]
            if refit:
                rec['judged_with_refit'] = refit
            rec['others_worse'] = worse
            guess = all(str(cand_prov.get(k, 'size prior')).startswith('size prior') for k in keys_of(ch))
            if worse:
                ok = False
                rec['decision'] = 'kept: makes %s worse' % ', '.join(worse)
            elif before[0] is None or after[0] is None:
                ok = guess
                rec['decision'] = ('taken, not validated (over a size prior)' if ok else
                                   'kept, not validated: %s' % (after[1] if after[0] is None else before[1]))
            else:
                ok = after[0] <= before[0] + 1e-12
                rec['decision'] = '%s: held-out %s -> %s' % ('taken' if ok else 'kept (worse)', fmt(before[0]), fmt(after[0]))
            if ok:
                cand = trial
                for k in keys_of(ch):
                    cand_prov[k] = 'fit %s (%s stage)' % (today, st)
        stages[st] = rec
        say('%-6s %s' % (st, rec['decision']))
        for n in notes:
            say('         ' + n)

    # ---- closed-loop replay check -------------------------------------------------------
    replay = None
    if elf and cand != current and vlogs:
        import replay_check
        val = [e for e in entries if e['role'] == 'validate']
        replay = replay_check.compare(elf, heli, current, cand, val, say=say)
        # worse in closed loop: drop the taken stages from the last one back, one at a
        # time, and replay again (the old model's replays are reused)
        taken = [st for st in mp.STAGES if stages[st]['decision'].startswith('taken')]
        tries = 0
        while replay['verdict'] == 'worse' and taken and tries < MAX_BACKOFF:
            st = taken.pop()
            tries += 1
            for k in keys_of(stages[st]['proposal']):
                for sec in ('args', 'attrs'):
                    if k in cand.get(sec, {}):
                        if k in current.get(sec, {}):
                            cand[sec][k] = current[sec][k]
                        else:
                            del cand[sec][k]
                if k in cur_prov:
                    cand_prov[k] = cur_prov[k]
                else:
                    cand_prov.pop(k, None)
            stages[st]['decision'] = 'dropped: the replay check found the model worse with it (%s -> %s)' % (
                fmt(replay.get('old')), fmt(replay.get('new')))
            say('replay check worse: %s dropped, replaying again' % st)
            if cand == current:
                break
            replay = replay_check.compare(elf, heli, current, cand, val, say=say, old_results=replay)
        if replay['verdict'] == 'worse':
            say('replay check: the new model tracks the held-out flights worse - nothing changes')
            cand, cand_prov = current, cur_prov
        replay.update(old_is='current' if current is not prior else 'prior',
                      model_args=cand['args'], model_attrs=cand['attrs'], model_trained=today)
        with open(os.path.join(folder, 'replay.json'), 'w') as f:
            json.dump(replay, f, indent=1)

    final = {'prior': score(prior), 'current': score(current), 'new': score(cand)}
    changed = cand != current
    report = {'heli': heli['name'], 'trained': today, 'group': group, 'setup': heli['groups'][group],
              'logs': {'fit': [lg.name for lg in logs], 'validate': [lg.name for lg in vlogs]},
              'start': note, 'stages': stages, 'replay': replay, 'changed': changed,
              'scores': {k: {s: v[0] for s, v in sc.items()} for k, sc in final.items()},
              'gates': mp.GATES, 'tolerance': TOL}
    if not dry:
        if changed:
            out = {'base': 'fitted',
                   'description': '%s: trained %s from %d logs (setup %s), checked on %d held-out'
                                  % (heli['name'], today, len(logs), group, len(vlogs)),
                   'attrs': cand['attrs'], 'args': cand['args'],
                   'provenance': {'trained': today, 'group': group, 'logs': report['logs'],
                                  'params': {k: cand_prov.get(k, '?') for k in keys_of(cand)},
                                  'held_out_scores': report['scores']['new']}}
            with open(model_path, 'w', newline='\n') as f:
                f.write(json.dumps(out, indent=1) + '\n')
            say('-> %s' % model_path)
        rpath = os.path.join(folder, 'replay.json')
        if replay is None and os.path.exists(rpath):
            # an earlier replay check (replay_report.py): kept in the report, marked when it
            # was of other numbers than the model now has
            old = json.load(open(rpath))
            # the gear ratio and motor poles only set the motor rpm the firmware sees
            dyn = lambda d: {k: v for k, v in (d or {}).items() if k not in ('gear_ratio', 'motor_poles')}
            same = (dyn(old.get('model_args')), dyn(old.get('model_attrs'))) == (dyn(cand['args']), dyn(cand['attrs']))
            report['replay'] = dict(old, stale=not same)
        json.dump(report, open(os.path.join(folder, 'train.json'), 'w'), indent=1, default=str)
        write_markdown(report, os.path.join(folder, 'TRAINING.md'))
    say('held-out scores (lower is better):')
    say('  %-8s %10s %10s %10s' % ('stage', 'prior', 'current', 'new'))
    for s in mp.STAGES:
        say('  %-8s %10s %10s %10s' % (s, fmt(final['prior'][s][0]), fmt(final['current'][s][0]), fmt(final['new'][s][0])))
    return report


def write_markdown(r, path):
    L = ['# %s - training report' % r['heli'], '',
         'Trained %s on setup `%s` (%s), from the %s. Written by `scripts/train.py`.'
         % (r['trained'], r['group'], ', '.join('%s %s' % (k, r['setup'].get(k)) for k in KEY), r['start']), '',
         '- fit logs (%d): %s' % (len(r['logs']['fit']), ', '.join(r['logs']['fit'])),
         '- held-out logs (%d): %s' % (len(r['logs']['validate']), ', '.join(r['logs']['validate']) or 'none'), '',
         '## Stages', '', '| stage | decision | held-out before | after |', '|---|---|---|---|']
    for s, rec in r['stages'].items():
        L.append('| %s | %s | %s | %s |' % (s, rec['decision'].split(':')[0], fmt(rec.get('score_before')), fmt(rec.get('score_after'))))
    L.append('')
    for s, rec in r['stages'].items():
        L.append('**%s**: %s' % (s, rec['decision']))
        L += ['- ' + n for n in rec['notes']]
        if rec.get('score_detail'):
            L.append('- held-out before: %s' % rec['score_detail']['before'])
            L.append('- held-out after: %s' % rec['score_detail']['after'])
        L.append('')
    L += ['## Held-out scores', '', 'Lower is better (`scripts/holdout.py`).', '',
          '| stage | prior | current | new |', '|---|---|---|---|']
    for s in mp.STAGES:
        L.append('| %s | %s | %s | %s |' % (s, fmt(r['scores']['prior'][s]), fmt(r['scores']['current'][s]), fmt(r['scores']['new'][s])))
    rp = r.get('replay')
    if rp:
        L += ['', '## Replay check', '',
              'Held-out flights replayed through the SITL (`replay_check.py`) with the %s and the new model, '
              'scored against the real flight (lower is better): %s -> %s, verdict **%s**.'
              % (rp.get('old_is', 'current'), fmt(rp.get('old')), fmt(rp.get('new')), rp['verdict'])]
        if rp.get('stale'):
            L.append('(Made for the model trained %s, whose numbers differ from the current ones: '
                     'run `replay_report.py` again.)' % rp.get('model_trained'))
        L.append('')
        for lg in rp['logs']:
            L += ['`%s`, %.0f-%.0f s at %d rpm:' % (lg['log'], lg['window'][0], lg['window'][1], lg['rpm']), '',
                  '| | real | %s | new |' % rp.get('old_is', 'current'), '|---|---|---|---|']
            om, nm = lg.get('old_metrics') or {}, lg.get('new_metrics') or {}
            for k in om:
                a, b = om[k], nm.get(k)
                if isinstance(a, list):
                    L.append('| %s | %s | %s | %s |' % (k, a[0], a[1], b[1] if b else '-'))
                else:
                    L.append('| %s | | %s | %s |' % (k, a, b if b is not None else '-'))
            L += ['| score | | %s | %s |' % (fmt(lg.get('old')), fmt(lg.get('new'))), '']
    L += ['', 'Model changed: %s.' % ('yes' if r['changed'] else 'no'), '']
    with open(path, 'w', newline='\n') as f:
        f.write('\n'.join(L))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[1].strip())
    ap.add_argument('heli')
    ap.add_argument('--group', help='setup group to train (default: the newest with 3+ flights)')
    ap.add_argument('--folds', type=int, default=8, help='jackknife folds (default 8)')
    ap.add_argument('--dry-run', action='store_true', help='report only, write nothing')
    ap.add_argument('--elf', help='SITL build: replay the held-out logs with the old and the new model')
    a = ap.parse_args()
    train(a.heli, a.group, a.folds, a.dry_run, a.elf)


if __name__ == '__main__':
    main()
