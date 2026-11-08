"""
Closed-loop check of a trained model: the held-out logs' sticks flown through the SITL
with the old and the new model, each scored against the real flight.

    python -I scripts/replay_check.py <rotorflight_SITL> helis/<heli>/heli.json OLD.json NEW.json

(train.py --elf runs it on its held-out logs.) Per held-out log, the flight is replayed
from its spool-up to the end of its longest upright airborne stretch at one headspeed (30 s
or more, within the first 360 s, gaps under 1 s bridged; a replay that starts mid-flight spends its time spooling
up, and the stick alignment needs the spool-up), with the flight's own gains from
its header (and the heli's CLI dump, "cli_dump" in the heli file, when the store has it),
the governor at that headspeed and the bridge's height hold. Both blackboxes are aligned
to the real one by the sticks (compare_replay.align) and scored over the stretch (from 5 s
into it):

    score = mean over roll, pitch, yaw of |ln(sim tracking error / real)|
            + |throttle sim - real| / 10 points + |tail sim - real| / 10 points

The tracking error (setpoint - gyro RMS) is what the firmware's controller makes of the
model's dynamics; throttle and tail trim are the rotor law and the torque balance. The new
model is "worse" if its mean score exceeds the old one's by more than 10 %.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from common import SKILL, Log  # noqa: E402
from fit_heli import targets  # noqa: E402
from logstore import decoded_name, find_file  # noqa: E402
from segment import segment  # noqa: E402

SIM = os.path.join(HERE, '..', '..', 'heli')
SETTLE_S = 5.0
WORSE = 1.10


def window(log, lo=30.0, longest=360.0):
    """(s0, t0, t1, rpm): the replay runs from s0 (3 s before the spool-up: a replay that
    starts mid-flight spends its time spooling and lifting off, and the stick alignment
    needs the spool-up) to t1; [t0, t1] is the longest upright airborne stretch at one
    headspeed within the first `longest` seconds (gaps under 1 s bridged), scored. None if it is under lo s."""
    lab, _, _ = segment(log)
    tg = np.round(targets(log) / 100) * 100
    on = np.flatnonzero(log.c('motor[0]') > 50)
    if not len(on):
        return None
    s0 = max(log.t[0], log.t[on[0]] - 3.0)
    ok = ~lab['ground'] & ~lab['inverted'] & ~lab['autorotation'] & (tg > 0) & (log.t <= s0 + longest)
    idx = np.flatnonzero(ok)
    if not len(idx):
        return None
    best = None
    fs = 1.0 / log.dt
    # gaps under 1 s (a moment of aerobatics, a segment edge) do not split a stretch
    cut = (np.diff(idx) > fs) | (np.diff(tg[idx]) != 0)
    for seg in np.split(idx, np.flatnonzero(cut) + 1):
        dur = log.t[seg[-1]] - log.t[seg[0]]
        if best is None or dur > best[2] - best[1]:
            best = (s0, log.t[seg[0]], log.t[seg[-1]], int(tg[seg[0]]))
    if best is None or best[2] - best[1] < lo:
        return None
    return best


def write_inputs(log, t0, t1, folder):
    """sticks.csv (rcCommand at 50 Hz) and tune.txt (the header's gains) for the bridge."""
    t = np.arange(t0, t1, 0.02)
    cols = [np.interp(t, log.t, log.c('rcCommand[%d]' % i)) for i in range(5)]
    with open(os.path.join(folder, 'sticks.csv'), 'w') as f:
        f.write('t,roll,pitch,yaw,coll,thr\n')
        for row in zip(t, *cols):
            f.write('%.3f,%.0f,%.0f,%.0f,%.0f,%.0f\n' % row)
    h = {k.strip(): str(v).strip() for k, v in log.hdr.items()}
    cmds = []
    for axis in ('roll', 'pitch', 'yaw'):
        if axis + 'PID' in h:
            v = [int(x) for x in h[axis + 'PID'].split(',')]
            for name, x in zip(('p', 'i', 'd', 'f', 'b'), v):
                cmds.append('set %s_%s_gain = %d' % (axis, name, x))
    if 'response_time' in h:
        for axis, v in zip(('roll', 'pitch', 'yaw'), h['response_time'].split(',')):
            cmds.append('set %s_response = %s' % (axis, v))
    if 'govPID' in h:
        for name, x in zip(('p', 'i', 'd', 'f'), h['govPID'].split(',')):
            cmds.append('set gov_%s_gain = %s' % (name, x))
    with open(os.path.join(folder, 'tune.txt'), 'w') as f:
        f.write('\n'.join(cmds) + '\n')


def executable(elf):
    """Windows runs a SITL build only under an .exe name: a copy beside it."""
    if os.name == 'nt' and elf.lower().endswith('.elf'):
        exe = elf[:-4] + '.exe'
        if not os.path.exists(exe) or os.path.getmtime(exe) < os.path.getmtime(elf):
            shutil.copy2(elf, exe)
        return exe
    return elf


def fly(elf, params, folder, tag, rpm, dump=None):
    """Replay through the SITL; returns the decoded sim blackbox (compare_replay.Npz)."""
    import compare_replay
    mp = os.path.join(folder, tag + '.json')
    json.dump(params, open(mp, 'w'))
    bbl = os.path.join(folder, tag + '.bbl')
    cmd = [sys.executable, os.path.join(SIM, 'heli_bridge.py'), executable(os.path.abspath(elf)), '--model', mp,
           '--governor', str(rpm), '--hold-alt', '10', '--replay', os.path.join(folder, 'sticks.csv'),
           '--cli-file', os.path.join(folder, 'tune.txt'), '--bbl', bbl]
    if dump:
        cmd += ['--dump', dump]
    with open(os.path.join(folder, tag + '.log'), 'w') as lf:
        subprocess.run(cmd, cwd=SIM, stdout=lf, stderr=subprocess.STDOUT, timeout=1800)
    subprocess.run([os.environ.get('NODE', 'node'), os.path.join(SKILL, 'bbl.js'), 'decode', bbl, '-o', bbl[:-4]],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
    npy = bbl[:-4] + '.0.npy'
    return compare_replay.Npz(npy) if os.path.exists(npy) else None


def tail(L, t0, t1):
    m = (L.t >= t0) & (L.t <= t1)
    try:
        m1 = L.c('motor[1]')[m]
        if np.ptp(m1) > 50:
            return float(np.median(m1)) / 10
    except ValueError:
        pass
    return float(np.median(L.c('mixer[2]')[m])) / 10


def score(real, sim, t0, t1):
    import compare_replay
    lag = compare_replay.align(real, sim)
    a, b = t0 + SETTLE_S, t1
    R = compare_replay.metrics(real, a, b)
    S = compare_replay.metrics(sim, a + lag, b + lag)
    # the tail: a motorized tail's throttle where motor[1] moves, else the yaw mixer (the
    # SITL logs a motor[1] of 0 for servo tails, which compare_replay would take)
    R['tail'], S['tail'] = tail(real, a, b), tail(sim, a + lag, b + lag)
    s = np.mean([abs(np.log(max(S[ax + '_err_rms'], 1e-3) / max(R[ax + '_err_rms'], 1e-3)))
                 for ax in ('roll', 'pitch', 'yaw')])
    s += abs(S['throttle'] - R['throttle']) / 10 + abs(S['tail'] - R['tail']) / 10
    keys = ('roll_err_rms', 'pitch_err_rms', 'yaw_err_rms', 'throttle', 'tail')
    return float(s), {k: (round(float(R[k]), 2), round(float(S[k]), 2)) for k in keys}


def compare(elf, heli, old, new, entries, say=print, max_logs=2):
    """Replay up to max_logs held-out logs with both models: {'verdict', 'logs': [...]}."""
    dump = find_file(heli['cli_dump']) if heli.get('cli_dump') else None
    if heli.get('cli_dump') and not dump:
        say('replay: CLI dump %s not in the store, flying the bridge default setup' % heli['cli_dump'])
    out = {'logs': []}
    for e in entries[:max_logs]:
        log = Log(decoded_name(e))
        w = window(log)
        if w is None:
            say('replay: %s has no upright stretch of 30 s at one headspeed' % log.name)
            continue
        s0, t0, t1, rpm = w
        folder = tempfile.mkdtemp(prefix='replay_')
        write_inputs(log, s0, t1, folder)
        rec = {'log': log.name, 'window': [round(t0, 1), round(t1, 1)], 'rpm': rpm}
        for tag, p in (('old', old), ('new', new)):
            sim = fly(elf, p, folder, tag, rpm, dump)
            if sim is None:
                rec[tag] = None
                say('replay %s %s: no SITL blackbox (see %s)' % (log.name, tag, folder))
                continue
            rec[tag], rec[tag + '_metrics'] = score(log, sim, t0, t1)
        say('replay %s %.0f-%.0f s at %d rpm: score old %s, new %s' % (log.name, t0, t1, rpm, rec.get('old'), rec.get('new')))
        out['logs'].append(rec)
    both = [r for r in out['logs'] if r.get('old') is not None and r.get('new') is not None]
    if not both:
        out['verdict'] = 'not run'
    else:
        o, n = np.mean([r['old'] for r in both]), np.mean([r['new'] for r in both])
        out['old'], out['new'] = round(float(o), 3), round(float(n), 3)
        out['verdict'] = 'worse' if n > o * WORSE else 'ok'
    return out


def main():
    elf, heli_path, old_path, new_path = sys.argv[1:5]
    heli = json.load(open(heli_path))
    load = lambda p: {k: v for k, v in json.load(open(p)).items() if k in ('base', 'args', 'attrs')}
    entries = [e for e in heli['logs'] if e.get('role') == 'validate']
    print(json.dumps(compare(elf, heli, load(old_path), load(new_path), entries), indent=1))


if __name__ == '__main__':
    main()
