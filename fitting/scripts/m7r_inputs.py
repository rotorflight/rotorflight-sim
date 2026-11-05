"""
Replay inputs for the M7R cruise profile (1500 rpm, real 238-322 s), without a CLI dump:

    python -I scripts/m7r_inputs.py <decoded log .npy> <out dir>

flown_m7r_p1.txt - the settings the log header carries (PID, governor, yaw and coupling
    gains, rates), the collective rate that reproduces the log's collective setpoint
    (2.5 x the stick: 122 against the SITL's default 100 -> 2.04) and the headspeed.
    The header holds the profile active when logging started; the cruise profile flies
    on it again from 237 s.
sticks_m7r_p1.csv - the recorded sticks, after a 16 s lead: 6 s throttle off, then the
    flight's throttle with neutral sticks while the governor spools up on the ground
    (arming mid-flight with the throttle up leaves the rotor stopped).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, 'scripts')
from common import bblog as _bblog

T0, T1, LEAD = 238.0, 322.0, 16.0


def main():
    npy, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    h = json.load(open(npy[:-4] + '.json', encoding='utf-8'))['rawHeader']
    num = lambda k: [int(x) for x in h[k].split(',')]
    cmds = []
    for axis in ('roll', 'pitch', 'yaw'):
        P, I, D, F, B = num(axis + 'PID')
        cmds += ['set %s_%s_gain = %d' % (axis, k, v) for k, v in zip('pidfb', (P, I, D, F, B))]
    cmds += ['set %s_response = %d' % (a, v) for a, v in zip(('roll', 'pitch', 'yaw'), num('response_time'))]
    cmds += ['set gov_%s = %d' % (k, v) for k, v in zip(('p_gain', 'i_gain', 'd_gain', 'f_gain', 'gain'), num('govPID'))]
    cw, ccw = num('yaw_stop_gain')
    cmds += ['set yaw_cw_stop_gain = %d' % cw, 'set yaw_ccw_stop_gain = %d' % ccw]
    pc, cff, colff = num('yaw_precomp')
    cmds += ['set yaw_precomp_cutoff = %d' % pc, 'set yaw_cyclic_ff_gain = %d' % cff, 'set yaw_collective_ff_gain = %d' % colff]
    cg, cr, cc = num('cyclic_coupling')
    cmds += ['set cyclic_cross_coupling_gain = %d' % cg, 'set cyclic_cross_coupling_ratio = %d' % cr,
             'set cyclic_cross_coupling_cutoff = %d' % cc]
    cmds.append('set rates_type = %s' % ['NONE', 'BETAFLIGHT', 'RACEFLIGHT', 'KISS', 'ACTUAL', 'QUICK', 'ROTORFLIGHT'][int(h['rates_type'])])
    for axis, rr, ex, sr in zip(('roll', 'pitch', 'yaw'), num('rc_rates'), num('rc_expo'), num('rates')):
        cmds += ['set %s_rc_rate = %d' % (axis, rr), 'set %s_expo = %d' % (axis, ex), 'set %s_srate = %d' % (axis, sr)]
    cmds += ['set gov_headspeed = 1500', 'set collective_rc_rate = 122']
    open(os.path.join(out, 'flown_m7r_p1.txt'), 'w').write('\n'.join(cmds) + '\n')

    lg = _bblog().load(npy)
    t = lg.t
    tt = np.arange(T0, T1, 0.02)
    cols = [np.interp(tt, t, np.asarray(lg['rcCommand[%d]' % i], float)) for i in range(5)]
    thr = float(np.median(cols[4]))
    rows = [(s, 0, 0, 0, -165, 0 if s < T0 - 10 else thr) for s in np.arange(T0 - LEAD, T0, 0.02)]
    rows += list(zip(tt, *cols))
    with open(os.path.join(out, 'sticks_m7r_p1.csv'), 'w') as f:
        f.write('t,roll,pitch,yaw,coll,thr\n')
        for r in rows:
            f.write('%.3f,%d,%d,%d,%d,%d\n' % (r[0], *(int(round(v)) for v in r[1:])))
    print('%d commands, %d stick rows -> %s' % (len(cmds), len(rows), out))


main()
