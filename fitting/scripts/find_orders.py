"""
Tail and motor gear ratios from the vibration: lines in the unfiltered gyro locked to the
rotor at non-integer orders.

    python -I scripts/find_orders.py helis/<heli>/heli.json [--write]

A line from the tail rotor or the motor turns with the rotor through its gearing, so it
sits at the same rotor order at every headspeed - a frame resonance stays at its frequency
and moves through the orders. Per headspeed level (steady 4 s windows, airborne) the gyroRAW
spectrum is taken in rotor orders and normalised by its median; the levels are combined by
their geometric mean, so only lines present at the same order at every level stand out.
Peaks between orders 2.5 and 20 that are not within 3 % of an integer (main rotor
harmonics) are candidates. The strongest in 3-6.5 is the tail rotor (servo tails,
typical 4-5.5), the strongest in 6.5-20 that is not a multiple of the tail's or a sideband
of one with the main rotor (a tail +- 1-2 orders) the motor (geared helis; direct drive
has none). A tail ratio that is an integer (the M7R's 5.0) sits on a main rotor harmonic
and is not found; a weak motor line is not found either - then nothing is written. --write puts them in the heli file
(tail.gear_ratio, drive.gear_ratio, with "gear_ratio_from"), unless it has them already.
Needs two headspeed levels or more to tell locked lines from resonances.
"""
import json
import os
import sys

import numpy as np
from scipy.signal import welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402
from logstore import save  # noqa: E402
from vibration import WIN, airborne  # noqa: E402

GRID = np.linspace(1.0, 20.0, 7601)       # rotor orders, 0.0025 steps


def order_spectra(logs):
    """Per headspeed level (100 rpm): median of the normalised order spectra of all axes."""
    per = {}
    for log in logs:
        if 'gyroRAW[0]' not in log.N:
            continue
        fs = 1.0 / log.dt
        hs = log.c('headspeed')
        air = airborne(log)
        g = [log.c('gyroRAW[%d]' % i) for i in range(3)]
        n = int(WIN * fs)
        for i0 in range(0, len(hs) - n, n // 2):
            sl = slice(i0, i0 + n)
            h = hs[sl]
            m = h.mean()
            if m < 500 or h.std() > 0.005 * m or (air is not None and not air[sl].all()):
                continue
            f0 = m / 60
            for ax in range(3):
                f, P = welch(g[ax][sl] - g[ax][sl].mean(), fs=fs, nperseg=int(2 * fs))
                o = f / f0
                ok = (o <= GRID[-1]) & (f < 0.45 * fs)
                if o[ok][-1] < 10:
                    continue
                s = np.interp(GRID, o[ok], P[ok], right=np.nan)
                per.setdefault(int(round(m, -2)), []).append(s / np.nanmedian(s))
    return {lv: np.nanmedian(np.array(v), axis=0) for lv, v in per.items() if len(v) >= 6}


def candidates(spec):
    """Peaks of the combined spectrum: (order, strength) strongest first, non-integer only."""
    levels = sorted(spec)
    logs_ = np.log(np.array([spec[lv] for lv in levels]))
    comb = np.exp(np.nanmean(logs_, axis=0))          # geometric mean over the levels
    worst = np.exp(np.nanmin(logs_, axis=0))          # and at the weakest level
    out = []
    for i in range(5, len(GRID) - 5):
        if not np.isfinite(comb[i]) or comb[i] < comb[i - 5:i + 6].max():
            continue
        o = GRID[i]
        if abs(o - round(o)) < 0.03 * o or o < 2.5:
            continue
        if comb[i] > 4.0 and worst[i] > 2.0:
            out.append((round(float(o), 3), round(float(comb[i]), 1)))
    return sorted(out, key=lambda x: -x[1])


def assign(cands):
    tail = next((o for o, _ in cands if 3.0 <= o <= 6.5), None)
    motor = None
    for o, _ in cands:
        if not 6.5 <= o <= 20:
            continue
        # a tail harmonic, or a sideband of one with the main rotor (a tail + b: the M5's
        # 10.5 = 2 x 4.75 + 1 is not its motor)
        if tail and any(abs(o - (a * tail + b)) < 0.01 * o for a in (1, 2, 3) for b in (-2, -1, 0, 1, 2)):
            continue
        motor = o
        break
    return tail, motor


def main():
    heli_path = sys.argv[1]
    heli = json.load(open(heli_path))
    logs = []
    for e in heli['logs']:
        name = e['file'][:-4] if e['file'].endswith('.bbl') else e['file']
        try:
            logs.append(Log(name))
        except SystemExit as err:
            print('skip %s: %s' % (name, err))
    spec = order_spectra(logs)
    print('headspeed levels: %s' % ', '.join('%d' % lv for lv in sorted(spec)))
    if len(spec) < 2:
        raise SystemExit('needs two headspeed levels or more')
    cands = candidates(spec)
    print('locked non-integer lines (order, strength over the median): %s' % (cands[:8] or 'none'))
    tail, motor = assign(cands)
    print('tail rotor: %s, motor: %s' % (tail, motor))
    if '--write' in sys.argv:
        changed = False
        if tail and (heli.get('tail') or {}).get('type') != 'motor' and not (heli.get('tail') or {}).get('gear_ratio'):
            heli.setdefault('tail', {}).update(gear_ratio=tail, gear_ratio_from='tail 1x order in gyroRAW (find_orders.py)')
            changed = True
        if motor and not (heli.get('drive') or {}).get('gear_ratio'):
            heli.setdefault('drive', {}).update(gear_ratio=motor, gear_ratio_from='motor 1x order in gyroRAW (find_orders.py)')
            changed = True
        if changed:
            save(heli, heli_path)
            print('->', heli_path)


if __name__ == '__main__':
    main()
