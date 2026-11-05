"""
CLI commands for the settings a flight was actually flown with, from its blackbox header
(the CLI dump predates the tuning flights). Applied after the dump, on the dump's active
profile and rate profile.

    python -I scripts/header_overrides.py <flight> <out.txt>
"""
import sys
import numpy as np

sys.path.insert(0, 'scripts')
from common import Log


def main():
    L = Log(sys.argv[1])
    h = {k.strip(): v.strip() for k, v in L.hdr.items()}
    num = lambda k: [int(x) for x in h[k].split(',')]
    cmds = []
    for axis in ('roll', 'pitch', 'yaw'):
        P, I, D, F, B = num(axis + 'PID')
        cmds += ['set %s_p_gain = %d' % (axis, P), 'set %s_i_gain = %d' % (axis, I),
                 'set %s_d_gain = %d' % (axis, D), 'set %s_f_gain = %d' % (axis, F),
                 'set %s_b_gain = %d' % (axis, B)]
    for axis, v in zip(('roll', 'pitch', 'yaw'), num('response_time')):
        cmds.append('set %s_response = %d' % (axis, v))
    gp, gi, gd, gf, gg = num('govPID')
    cmds += ['set gov_p_gain = %d' % gp, 'set gov_i_gain = %d' % gi, 'set gov_d_gain = %d' % gd,
             'set gov_f_gain = %d' % gf, 'set gov_gain = %d' % gg]
    target = L.c('govTarget')
    cmds.append('set gov_headspeed = %d' % round(np.median(target[target > 0]), -1))
    open(sys.argv[2], 'w').write('\n'.join(cmds) + '\n')
    print('\n'.join(cmds))


main()
