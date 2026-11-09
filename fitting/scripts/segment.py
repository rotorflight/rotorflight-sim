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

Overlays (inputs(), not exclusive): spoolup, doublet (sharp stick inputs) and sweep
(frequency sweeps on roll or pitch) - what excites the rotor law, the flap/body mode and
the frequency response.

Without airspeed, hover and slow forward flight at constant height differ only in the
attitude, so short hovers can be missed and slow drifts counted. The baro altitude is
taken relative to the ground before the first take-off; hover below `ige_m` (default
1.5 m) is flagged as in ground effect - it needs clearly less collective there. A log
without a baro (no altitude field, or one that never moves) finds hover by attitude and
sticks alone, and cannot tell its height: nothing is flagged as in ground effect, and
slow level flight can count as hover.
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
    baro = has_baro(log)
    vario = sm(log.c('vario') / 100) if baro and 'vario' in log.N else np.zeros(len(air))
    alt = log.c('altitude') / 100 if baro else np.zeros(len(air))
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
    # without a baro the height is unknown: no hover is flagged as in ground effect
    return lab, alt, keep & (alt < ige_m) if baro else np.zeros_like(keep)


def has_baro(log):
    """The log has a baro altitude that moves (not a flight controller without one)."""
    return 'altitude' in log.N and np.ptp(log.c('altitude')) > 100


INPUTS = ('spoolup', 'doublet', 'sweep')


def inputs(log):
    """Overlay masks (not exclusive, unlike the labels) of what excites what:
      spoolup   the motor powered and the rotor below 90 % of its later speed, accelerating
                (> 3 %/s): what identifies the rotor law's motor torque against inertia
      doublet   0.3 s before to 1.2 s after a sharp roll or pitch input (setpoint swinging
                by > 150 deg/s within 0.3 s, as fit_flap.py): flap/body dynamics
      sweep     a roll or pitch stick oscillating (1 s RMS setpoint > 20 deg/s) whose
                frequency (zero crossings in 4 s windows) rises 3x or more over 10 s or
                longer: the frequency response
    """
    fs = 1.0 / log.dt
    n = len(log.t)
    out = {}
    # spool-up
    hs = uniform_filter1d(log.c('headspeed'), max(1, int(0.5 * fs))) if 'headspeed' in log.N else np.zeros(n)
    top = np.percentile(hs, 95) if hs.any() else 0.0
    rising = np.gradient(hs, log.t) > 0.03 * max(top, 1.0)
    powered = log.c('motor[0]') > 50 if 'motor[0]' in log.N else np.zeros(n, bool)
    out['spoolup'] = powered & rising & (hs < 0.9 * top) & (hs > 0.05 * top)
    # doublets
    d = np.zeros(n, bool)
    k = max(1, int(0.3 * fs))
    for ax in (0, 1):
        sp = log.c('setpoint[%d]' % ax)
        jump = np.flatnonzero(np.abs(sp[k:] - sp[:-k]) > 150)
        last = -10 ** 9
        for i in jump:
            if i - last > fs:            # one per second at most
                d[max(0, i - int(0.3 * fs)):i + int(1.2 * fs)] = True
                last = i
    out['doublet'] = d
    # sweeps: zero-crossing rate of the setpoint in 4 s windows, stepped by 1 s
    sw = np.zeros(n, bool)
    win, step = int(4 * fs), int(fs)
    for ax in (0, 1):
        sp = log.c('setpoint[%d]' % ax)
        rows = []
        for a in range(0, n - win, step):
            x = sp[a:a + win] - np.mean(sp[a:a + win])
            rms = np.sqrt(np.mean(x ** 2))
            zc = np.count_nonzero(np.diff(np.signbit(x))) / 8.0      # Hz
            rows.append((a, rms > 20, zc, rms))
        run = []
        for a, active, zc, rms in rows + [(None, False, 0.0, 0.0)]:
            if active:
                run.append((a, zc, rms))
                continue
            if run:
                # without the edges, whose windows overlap the stick at rest
                med = np.median([r for _, _, r in run])
                while run and run[-1][2] < 0.8 * med:
                    run.pop()
                while run and run[0][2] < 0.8 * med:
                    run.pop(0)
            if len(run) >= 7 and run[-1][1] >= 3 * max(run[0][1], 0.1):
                zs = [z for _, z, _ in run]
                # mostly rising: a sweep, not stirring
                if np.corrcoef(np.arange(len(zs)), zs)[0, 1] > 0.8:
                    sw[run[0][0]:run[-1][0] + win] = True
            run = []
    out['sweep'] = sw
    return out


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
        inp = inputs(log)
        print('%-28s ' % name + ' '.join('%10.1f s' % (lab[k].sum() / fs) for k in LABELS)
              + '   %.1f s' % (ige.sum() / fs)
              + '   | ' + ', '.join('%s %.1f s' % (k, inp[k].sum() / fs) for k in INPUTS))


if __name__ == '__main__':
    main()
