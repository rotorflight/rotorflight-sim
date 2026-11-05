"""Closed-loop sweep of the M7R flap/body mode on the cruise-profile replay (real 242-318 s).

    python -I scripts/m7r_sweep.py <sitl elf> <decoded log .npy> <inputs dir> <out dir> [NAME=v1,v2 ...]

<inputs dir> holds flown_m7r_p1.txt and sticks_m7r_p1.csv (scripts/m7r_inputs.py). The
replay holds height with --hold-alt 10; both logs are scored over the instants the sim
flies above 1 m. LAG is the sim - real offset of that replay (compare_replay.align).
"""
import itertools, os, subprocess, sys
import numpy as np
from scipy.signal import welch

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, '..', '..', 'heli')
sys.path.insert(0, HERE)
from common import SKILL, bblog as _bblog
bblog = _bblog()

LAG, T0, T1 = 219.27, 242.0, 318.0
real = None
INPUTS = OUT = None


def metrics(lg, off, alt=None):
    t = lg.t + off
    m = (t >= T0) & (t <= T1)
    if alt is not None:
        m &= np.interp(t, alt[0] + LAG, alt[1], left=0, right=0) > 1.0
    fs = 1 / np.median(np.diff(lg.t))
    out = {}
    for ax, n in ((0, 'roll'), (1, 'pitch'), (2, 'yaw')):
        e = (np.asarray(lg['setpoint[%d]' % ax], float) - np.asarray(lg['gyroADC[%d]' % ax], float))[m]
        out[n + '_err'] = np.sqrt(np.mean(e ** 2))
        f, P = welch(e - e.mean(), fs=fs, nperseg=1024)
        b = (f >= 1.5) & (f < 3.0)
        out[n + '_err_1.5-3Hz'] = np.sqrt(P[b].sum() * (f[1] - f[0]))
    out['yaw_mix'] = np.median(np.asarray(lg['mixer[2]'], float)[m]) / 10
    out['throttle'] = np.median(np.asarray(lg['motor[0]'], float)[m]) / 10
    return out


def run(elf, params):
    tag = '_'.join('%s%s' % kv for kv in params) or 'default'
    out = OUT
    os.makedirs(out, exist_ok=True)
    bbl = os.path.join(out, 'sim_%s.bbl' % tag)
    npy = bbl[:-4] + '.0.npy'
    if not os.path.exists(npy):
        cmd = [sys.executable, os.path.join(SIM, 'heli_bridge.py'), elf, '--model', 'm7r', '--governor', '1500',
               '--cli-file', os.path.join(INPUTS, 'flown_m7r_p1.txt'), '--replay', os.path.join(INPUTS, 'sticks_m7r_p1.csv'),
               '--bbl', bbl, '--hold-alt', '10', '--log', bbl[:-4] + '.csv']
        for k, v in params:
            if k == 'L':
                cmd += ['--set', 'Lb1s=%s' % v, '--set', 'Ma1s=%s' % v]
            else:
                cmd += ['--set', '%s=%s' % (k, v)]
        with open(bbl[:-4] + '.log', 'w') as lf:
            subprocess.run(cmd, cwd=SIM, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
        subprocess.run(['node', os.path.join(SKILL, 'bbl.js'), 'decode', bbl, '-o', bbl[:-4]],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
    a = np.genfromtxt(bbl[:-4] + '.csv', delimiter=',', names=True)
    alt = (a['t'], a['alt_m'])
    return tag, metrics(bblog.load(npy), LAG, alt), alt


def main():
    global real, INPUTS, OUT
    elf = os.path.abspath(sys.argv[1])
    real = bblog.load(sys.argv[2])
    INPUTS, OUT = os.path.abspath(sys.argv[3]), os.path.abspath(sys.argv[4])
    grid = [[(k, v) for v in vs.split(',')] for k, vs in (s.split('=') for s in sys.argv[5:])]
    for params in itertools.product(*grid):
        tag, S, alt = run(elf, params)
        # the real flight over the same instants (sim above 1 m)
        R = metrics(real, 0.0, alt)
        keys = list(S)
        print('%-22s ' % tag + ' '.join('%s %.1f/%.1f' % (k, S[k], R[k]) for k in keys), flush=True)
    print('(sim/real, both over the instants the sim flies above 1 m)')


main()
