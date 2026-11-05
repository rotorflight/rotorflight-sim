"""
Closed-loop identification of what the open-loop fits could not see: replay the real
flight's sticks through the SITL with the real config, for a grid of model parameters,
and score each run against the real flight (compare_replay.py metrics).

    FIT_DUMP=<cli dump> python -I scripts/sweep_replay.py <sitl exe> <flight> <sticks.csv> <flown.txt> <out_dir> NAME=v1,v2 [NAME=...]

FIT_DUMP is the heli's CLI dump (heli_bridge.py --dump). Decoding the SITL's blackbox needs
Node.js and the blackbox-decode skill (see common.py).

Runs one SITL at a time (fixed ports), ~1.5 min per case.
"""
import itertools
import os
import subprocess
import sys

sys.path.insert(0, 'scripts')
import compare_replay
from common import SKILL

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, '..', '..', 'heli')
DUMP = os.environ.get('FIT_DUMP') or sys.exit('set FIT_DUMP to the CLI dump')
# decoder: the Blackbox Explorer parser of the blackbox-decode skill
BBL_JS = os.path.join(SKILL, 'bbl.js')
KEYS = ('roll_err_rms', 'pitch_err_rms', 'yaw_err_rms', 'roll_4_8Hz', 'pitch_4_8Hz',
        'roll_peak_hz', 'pitch_peak_hz', 'tail', 'throttle')


def run(exe, flight, sticks, flown, out, params):
    tag = '_'.join('%s%s' % (k, v) for k, v in params)
    bbl = os.path.join(out, 'sim_%s.bbl' % tag)
    npz = bbl[:-4] + '.0.npy'
    if not os.path.exists(npz):
        cmd = [sys.executable, os.path.join(SIM, 'heli_bridge.py'), exe, '--model', 'stratos200',
               '--dump', DUMP, '--cli-file', flown, '--replay', sticks, '--bbl', bbl]
        for k, v in params:
            if k == 'L':                      # both axes' flap stiffness at once
                cmd += ['--set', 'Lb1s=%s' % v, '--set', 'Ma1s=%s' % v]
            else:
                cmd += ['--set', '%s=%s' % (k, v)]
        with open(bbl[:-4] + '.log', 'w') as lf:
            subprocess.run(cmd, cwd=SIM, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
        subprocess.run(['node', BBL_JS, 'decode', bbl, '-o', bbl[:-4]], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=600)
    sys.argv = ['compare_replay.py', flight, npz, tag]
    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        R, S = compare_replay.main()
    return tag, R, S


def main():
    exe, flight, sticks, flown, out = sys.argv[1:6]
    sticks, flown, out = (os.path.abspath(x) for x in (sticks, flown, out))
    os.makedirs(out, exist_ok=True)
    grid = []
    for spec in sys.argv[6:]:
        k, vs = spec.split('=')
        grid.append([(k, v) for v in vs.split(',')])
    rows = []
    for params in itertools.product(*grid):
        tag, R, S = run(exe, flight, sticks, flown, out, params)
        rows.append((tag, S))
        print('%-32s ' % tag + ' '.join('%s %.2f' % (k, S[k]) for k in KEYS), flush=True)
    print('%-32s ' % 'REAL' + ' '.join('%s %.2f' % (k, R[k]) for k in KEYS), flush=True)


main()
