"""
Export a flight's pilot sticks for the SITL replay (heli/heli_bridge.py --replay).

    python -I scripts/export_sticks.py <flight> <t0> <t1> <out.csv>

Columns t, roll, pitch, yaw, coll, thr in rcCommand units (roll/pitch/yaw/collective
+-500 around centre, throttle 0..1000), sampled at 50 Hz like an RC link.
"""
import sys
import numpy as np

sys.path.insert(0, 'scripts')
from common import Log


def main():
    name, t0, t1, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    L = Log(name)
    t = np.arange(t0, t1, 0.02)
    cols = [np.interp(t, L.t, L.c('rcCommand[%d]' % i)) for i in range(5)]
    with open(out, 'w') as f:
        f.write('t,roll,pitch,yaw,coll,thr\n')
        for row in zip(t, *cols):
            f.write('%.3f,%.0f,%.0f,%.0f,%.0f,%.0f\n' % row)
    print('%d rows, %.1f s -> %s' % (len(t), t1 - t0, out))


main()
