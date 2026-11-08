"""
Setup fingerprint of a flight log: what of the heli's mechanics and configuration the log
says, so logs are pooled only within one setup.

    python -I scripts/fingerprint.py helis/<heli>/heli.json     # setup per log, and groups

A fit pools logs that share a fingerprint. What goes in (from the blackbox header and the
logged fields):

  craft        Craft name (a different name usually means another heli or a moved FC)
  firmware     Rotorflight version, major.minor (mixer and logging units can change)
  board        flight controller board
  tail         motor (a motor[1] field that moves) or servo
  collective   collectiveRange (the collective's mixer range, swash setup)
  esc          motor protocol and bidirectional DShot

Not in the key, so they do not split a heli: the PID gains, rates and governor settings
(tuning changes the controller, not the plant - the replays take them from each log's
header), the headspeed (fitted per headspeed), and the pack (cell count is estimated from
the voltage only, a partly charged pack would move it).

A changed mechanic the header cannot see (blades, gear, dampers) needs a "group" on the
log entries in the heli file: an explicit group always wins over the fingerprint.
"""
import hashlib
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Log  # noqa: E402

KEY = ('craft', 'firmware', 'board', 'tail', 'collective', 'esc')


def setup(log):
    """The setup a log was flown with: the KEY fields, plus informational ones (cells,
    gains) that do not split a heli."""
    h = {k.strip(): str(v).strip() for k, v in log.hdr.items()}
    fw = h.get('Firmware revision', '')
    m = re.search(r'(\d+)\.(\d+)', fw)
    tail = 'servo'
    if 'motor[1]' in log.N and np.ptp(log.c('motor[1]')) > 50:
        tail = 'motor'
    s = {'craft': h.get('Craft name', ''),
         'firmware': '%s.%s' % m.groups() if m else fw,
         'board': h.get('Board information', ''),
         'tail': tail,
         'collective': h.get('collectiveRange', ''),
         'esc': '%s/%s' % (h.get('motor_pwm_protocol', ''), h.get('dshot_bidir', ''))}
    info = {'firmware_revision': fw, 'rollPID': h.get('rollPID', ''), 'pitchPID': h.get('pitchPID', ''),
            'yawPID': h.get('yawPID', ''), 'log_start': h.get('Log start datetime', '')}
    if 'Vbat' in log.N:
        v = log.c('Vbat') / 100
        v = v[v > 1]
        if len(v):
            # charged packs rest at 4.1-4.2 V per cell; a guess, not part of the key
            info['cells_estimate'] = int(np.ceil(np.percentile(v, 99) / 4.22))
    return s, info


def group_id(s):
    """A short stable id for a setup (its KEY fields)."""
    return hashlib.sha1(json.dumps([s.get(k, '') for k in KEY]).encode()).hexdigest()[:8]


def differences(a, b):
    return ['%s %r -> %r' % (k, a.get(k), b.get(k)) for k in KEY if a.get(k) != b.get(k)]


def main():
    heli = json.load(open(sys.argv[1]))
    groups = {}
    for entry in heli['logs']:
        name = os.path.splitext(entry['file'])[0]
        try:
            log = Log(name)
        except SystemExit as e:
            print('skip %s: %s' % (name, e))
            continue
        s, info = setup(log)
        gid = entry.get('group') or group_id(s)
        groups.setdefault(gid, (s, []))[1].append(name)
        print('%-34s %s  %s  cells~%s' % (name, gid, ' | '.join(s[k] for k in KEY), info.get('cells_estimate')))
    print()
    ref = None
    for gid, (s, names) in groups.items():
        print('group %s: %d logs%s' % (gid, len(names), ('  (vs first: %s)' % '; '.join(differences(ref, s))) if ref else ''))
        ref = ref or s


if __name__ == '__main__':
    main()
