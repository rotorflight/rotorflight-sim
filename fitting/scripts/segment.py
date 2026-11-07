"""
Segmentation of a flight log: what the heli was doing, sample by sample, so each fit
uses the parts it can identify.

    python -I scripts/segment.py helis/<heli>/heli.json      # seconds per label and log

Labels (exclusive, in this order of precedence):
  ground        not airborne (the firmware's AIRBORNE_STATE events)
  autorotation  airborne with the governor in AUTOROTATION or BAILOUT
  inverted      |roll| or |pitch| beyond 120 deg
  aerobatic     airborne with a stick-driven rate (1 s mean |setpoint|) above 100 deg/s
  hover         calm and holding height: 1 s mean |vario| < 0.5 m/s (baro), pitch within
                5 deg, roll within 5 deg of the heli's calm-air roll trim (the tail rotor's
                side thrust needs a few degrees), 1 s mean |setpoint| < 15 deg/s, for 1.5 s
                at least
  cruise        the rest of the airborne time (forward flight, gentle manoeuvres)

Without airspeed, hover and slow forward flight at constant height differ only in the
attitude, so short hovers can be missed and slow drifts counted. The baro altitude is
taken relative to the ground before the first take-off; hover below `ige_m` (default
1.5 m) is flagged as in ground effect - it needs clearly less collective there.
"""
import json
import os
import sys

import numpy as np
from scipy.ndimage import uniform_filter1d

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402

LABELS = ('ground', 'autorotation', 'inverted', 'aerobatic', 'hover', 'cruise')


def airborne(log):
    air = np.zeros(len(log.t), bool)
    t0 = log.c('time')[0]
    start = None
    for tu, typ, d in log.events:
        if typ not in (52, 'AIRBORNE_STATE') or tu is None:
            continue
        ts = (tu - t0) / 1e6
        if d and d[0] == 1:
            start = ts
        elif start is not None:
            air |= (log.t > start + 1) & (log.t < ts - 1)
            start = None
    if start is not None:
        air |= log.t > start + 1
    return air


def governor_states(log):
    """Governor state per sample, from the GOVERNOR_STATE events."""
    st = np.zeros(len(log.t), int)
    t0 = log.c('time')[0]
    for tu, typ, d in log.events:
        if typ in (50, 'GOVERNOR_STATE') and tu is not None and d:
            st[log.t >= (tu - t0) / 1e6] = d[0]
    return st


def segment(log, ige_m=1.5):
    """Returns (labels: dict label -> bool mask, alt: baro height above the take-off
    ground in m, ige: bool mask of hover in ground effect)."""
    fs = 1.0 / log.dt
    sm = lambda x, s=1.0: uniform_filter1d(x, max(1, int(s * fs)))
    air = airborne(log)
    gov = governor_states(log)
    roll, pitch = log.c('attitude[0]') / 10, log.c('attitude[1]') / 10
    rs, ps = sm(roll), sm(pitch)
    vario = sm(log.c('vario') / 100) if 'vario' in log.N else np.zeros(len(air))
    alt = log.c('altitude') / 100 if 'altitude' in log.N else np.zeros(len(air))
    first = np.flatnonzero(air)
    pre = (log.t > log.t[first[0]] - 3) & (log.t < log.t[first[0]] - 1) if len(first) else ~air
    alt = alt - (np.median(alt[pre]) if pre.any() else 0.0)
    sp = np.max([sm(np.abs(log.c('setpoint[%d]' % i))) for i in range(3)], axis=0)

    lab = {}
    lab['ground'] = ~air
    lab['autorotation'] = air & np.isin(gov, (7, 8))
    rest = air & ~lab['autorotation']
    lab['inverted'] = rest & ((np.abs(roll) > 120) | (np.abs(pitch) > 120))
    rest &= ~lab['inverted']
    lab['aerobatic'] = rest & (sp > 100)
    rest &= ~lab['aerobatic']
    calm = rest & (sp < 15) & (np.abs(ps) < 5) & (np.abs(rs) < 15)
    trim = float(np.median(rs[calm])) if calm.any() else 0.0
    hov = calm & (np.abs(vario) < 0.5) & (np.abs(rs - trim) < 5)
    keep = np.zeros_like(hov)
    idx = np.flatnonzero(hov)
    if len(idx):
        for seg in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
            if len(seg) >= 1.5 * fs:
                keep[seg] = True
    lab['hover'] = keep
    lab['cruise'] = rest & ~keep
    return lab, alt, keep & (alt < ige_m)


def main():
    heli = json.load(open(sys.argv[1]))
    print('%-28s ' % 'log' + ' '.join('%12s' % k for k in LABELS) + '   hover in ground effect')
    for entry in heli['logs']:
        name = os.path.splitext(entry['file'])[0]
        try:
            log = Log(name)
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
            continue
        lab, alt, ige = segment(log)
        fs = 1.0 / log.dt
        print('%-28s ' % name + ' '.join('%10.1f s' % (lab[k].sum() / fs) for k in LABELS)
              + '   %.1f s' % (ige.sum() / fs))


if __name__ == '__main__':
    main()
