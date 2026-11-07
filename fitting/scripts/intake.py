"""
Log intake for a heli: decode its blackbox files, check what each log can feed, and add
the logs to the heli file.

    python -I scripts/intake.py helis/<heli>/heli.json <log.bbl> [...] [--dry-run]

Each file is decoded with the blackbox-decode skill's bbl.js into FIT_DATA_DIR (one
<stem>.<n>.npy per log in the file; logs already listed in the heli file, or the same
log in another file, are skipped). Every log is then checked against what the fits need,
and the report says what is missing and how to get it:

  rate          blackbox rate: 1 kHz for the vibration (main 4x, tail and motor lines),
                at least 250 Hz for the rate gains
  gyroRAW       the unfiltered gyro (vibration.py)
  headspeed     a rotor speed source (rotor law, per-headspeed fits, vibration orders)
  power         Vbat and Ibat (rotor law, pack)
  baro          altitude and vario (segmentation: hover, height)
  airborne      AIRBORNE_STATE events (all fits use airborne time)
  gains         PID gains in the header (closed-loop replay)
  hover         hover out of ground effect per headspeed, 5 s for the hover stage; two
                headspeeds for the thrust (inflow) law
  sharp inputs  stick doublets on roll and pitch (control power, flap time constant)
  autorotation  an airborne autorotation (validates the inflow model); needs the
                headspeed from the rotor side to mean anything after the motor stops
  gps           GPS speed (forward-flight derivatives)

See ../CONTRIBUTING_LOGS.md for how to fly and log a flight for the sim.
"""
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import DATA, SKILL, Log  # noqa: E402
from segment import segment  # noqa: E402

HOVER_GATE_S = 5.0


def decode(bbl):
    """Decode every log of a file into DATA; returns their names (<stem>.<n>)."""
    os.makedirs(DATA, exist_ok=True)
    stem = os.path.splitext(os.path.basename(bbl))[0]
    node = os.environ.get('NODE', 'node')
    js = os.path.join(SKILL, 'bbl.js')
    info = json.loads(subprocess.run([node, js, 'info', bbl], check=True, capture_output=True,
                                     text=True).stdout)
    out = os.path.join(DATA, stem)
    subprocess.run([node, js, 'decode', bbl, '-o', out], check=True, capture_output=True)
    names = []
    for lg in info['logs']:
        if 'error' in lg:
            print('  %s log %s: %s' % (stem, lg['log'], lg['error']))
            continue
        names.append('%s.%d' % (stem, lg['log']))
    return names


def fingerprint(log):
    """Same flight in another file (or a copied file): start time, length and first frames."""
    h = hashlib.md5()
    h.update(('%s %d' % (log.hdr.get('Log start datetime', ''), len(log.t))).encode())
    h.update(np.ascontiguousarray(log.a[:200]).tobytes())
    return h.hexdigest()


def sharp_inputs(log):
    """Stick doublets: roll or pitch setpoint swinging by more than 150 deg/s within 0.3 s."""
    fs = 1.0 / log.dt
    n = max(1, int(0.3 * fs))
    count = 0
    for ax in (0, 1):
        sp = log.c('setpoint[%d]' % ax)
        i = 0
        while i + n < len(sp):
            if abs(sp[i + n] - sp[i]) > 150:
                count += 1
                i += int(1.0 * fs)            # one per second at most
            else:
                i += n // 2 or 1
    return count


def check(log):
    fs = 1.0 / log.dt
    has = lambda n: n in log.N
    nonzero = lambda n: has(n) and np.any(np.abs(log.c(n)) > 0)
    r = {'rate_hz': round(fs)}
    r['gyroRAW'] = nonzero('gyroRAW[0]')
    r['headspeed'] = nonzero('headspeed')
    r['power'] = nonzero('Vbat') and nonzero('Ibat')
    r['baro'] = has('altitude') and np.ptp(log.c('altitude')) > 100
    r['airborne'] = any(typ in (52, 'AIRBORNE_STATE') for _, typ, _ in log.events)
    r['gains'] = 'rollPID' in log.hdr
    r['gps'] = any(n.lower().startswith('gps') for n in log.N)
    lab, alt, ige = segment(log)
    tg = np.round(log.c('govTarget') / 100) * 100 if has('govTarget') else np.zeros(len(log.t))
    oge = lab['hover'] & ~ige
    r['hover_s'] = {'%d' % lv: round(float((oge & (tg == lv)).sum() / fs), 1)
                    for lv in sorted(set(tg[oge])) if lv > 0}
    r['air_s'] = round(float((~lab['ground']).sum() / fs), 1)
    r['autorotation_s'] = round(float((lab['autorotation'] & (alt > 5)).sum() / fs), 1)
    r['sharp_inputs'] = sharp_inputs(log)
    r['headspeeds'] = sorted(int(x) for x in set(tg[~lab['ground']]) if x > 0)
    return r


def advice(r):
    """What a log cannot feed, and what would fix it."""
    out = []
    if r['rate_hz'] < 250:
        out.append('rate %d Hz: too low for the rate gains - log at 1 kHz' % r['rate_hz'])
    elif r['rate_hz'] < 900:
        out.append('rate %d Hz: vibration only below %d Hz - log at 1 kHz' % (r['rate_hz'], r['rate_hz'] // 2))
    if not r['gyroRAW']:
        out.append('no gyroRAW: no vibration fit (Rotorflight 4.6+ logs it; blackbox_log_gyro_raw = ON)')
    if not r['headspeed']:
        out.append('no headspeed: no rotor law, no vibration orders (rpm sensor or ESC telemetry)')
    if not r['power']:
        out.append('no Vbat/Ibat: no rotor law in the volts form')
    if not r['baro']:
        out.append('no baro altitude: no hover segmentation')
    if not r['airborne']:
        out.append('no AIRBORNE_STATE events: every fit needs the airborne time')
    if not r['gains']:
        out.append('no PID gains in the header: no closed-loop replay')
    ok = [k for k, v in r['hover_s'].items() if v >= HOVER_GATE_S]
    if len(ok) < 2:
        out.append('hover out of ground effect >= %.0f s at %s headspeed(s) - the thrust law needs two: hover '
                   '10 s at 2 m+ on each headspeed' % (HOVER_GATE_S, len(ok)))
    if r['sharp_inputs'] < 4:
        out.append('%d sharp roll/pitch inputs - fly doublets for control power and flap time constant'
                   % r['sharp_inputs'])
    if r['autorotation_s'] < 3:
        out.append('no airborne autorotation (with a rotor-side rpm sensor it validates the inflow model)')
    if not r['gps']:
        out.append('no GPS speed: no forward-flight derivatives')
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    dry = '--dry-run' in sys.argv
    heli_path, files = args[0], args[1:]
    heli = json.load(open(heli_path))
    known = {os.path.splitext(e['file'])[0] for e in heli['logs']}
    seen = {}
    for name in known:
        try:
            seen[fingerprint(Log(name))] = name
        except SystemExit:
            pass
    added = []
    for bbl in files:
        print('== %s' % bbl)
        for name in decode(bbl):
            if name in known:
                print('  %s: already in %s' % (name, heli_path))
                continue
            log = Log(name)
            fp = fingerprint(log)
            if fp in seen:
                print('  %s: the same log as %s, skipped' % (name, seen[fp]))
                continue
            seen[fp] = name
            r = check(log)
            print('  %s: %.0f s airborne, %d Hz, headspeeds %s, hover (out of ground effect) %s s, '
                  '%d sharp inputs, autorotation %.0f s'
                  % (name, r['air_s'], r['rate_hz'], r['headspeeds'] or '-', r['hover_s'] or '-',
                     r['sharp_inputs'], r['autorotation_s']))
            for line in advice(r):
                print('    - ' + line)
            if r['air_s'] < 5:
                print('    (no flight: not added)')
                continue
            added.append({'file': name, 'headspeed': r['headspeeds'], 'intake': {
                k: r[k] for k in ('rate_hz', 'gyroRAW', 'power', 'baro', 'hover_s', 'sharp_inputs', 'autorotation_s')}})
    if added and not dry:
        heli['logs'] += added
        with open(heli_path, 'w') as f:
            json.dump(heli, f, indent=2)
            f.write('\n')
        print('%d log(s) added to %s' % (len(added), heli_path))


if __name__ == '__main__':
    main()
