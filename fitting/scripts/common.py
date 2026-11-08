"""
Shared log loading for the fitting scripts.

Logs are read with bblog.py of the blackbox-decode skill
(https://github.com/rotorflight/rotorflight-agent-skills, skills/blackbox-decode): either
the .npy/.json that its bbl.js writes or an .npz from its pure-Python fallback decoder.
The skill's scripts folder is looked up in BLACKBOX_DECODE_DIR, else in a
rotorflight-agent-skills checkout next to this repository.

Flights are named by file stem and looked up in FIT_DATA_DIR (default data/split, relative
to the fitting folder the scripts run from; several folders separated by os.pathsep):
<name>.npz, <name>.0.npy or <name>.npy. New decodes go to the first folder.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SKILL = os.environ.get('BLACKBOX_DECODE_DIR') or os.path.join(
    HERE, '..', '..', '..', 'rotorflight-agent-skills', 'skills', 'blackbox-decode', 'scripts')
DATA_DIRS = [d for d in os.environ.get('FIT_DATA_DIR', os.path.join('data', 'split')).split(os.pathsep) if d]
DATA = DATA_DIRS[0]

# The Stratos 200 / Bo105 flights the preset was fitted from (2026-10-08): airborne, in acro.
# Replace for another heli's logs.
FLIGHTS = ['20261008_160146_11', '20261008_160146_12', '20261008_172721_00',
           '20261008_174938_00', '20261008_180641_00', '20261008_182311_01']


def bblog():
    sys.path.insert(0, SKILL)
    try:
        import bblog as mod
    except ImportError:
        raise SystemExit('bblog.py not found: set BLACKBOX_DECODE_DIR to the blackbox-decode '
                         'skill\'s scripts folder (tried %s)' % os.path.abspath(SKILL))
    return mod


def find(name):
    cands = [name]
    for d in DATA_DIRS:
        cands += [os.path.join(d, name + '.npz'), os.path.join(d, name + '.0.npy'), os.path.join(d, name + '.npy')]
    for cand in cands:
        if cand.endswith(('.npz', '.npy')) and os.path.exists(cand):
            return cand
    raise SystemExit('no decoded log for %s in %s' % (name, os.pathsep.join(DATA_DIRS)))


def has_log(name):
    try:
        find(name)
        return True
    except SystemExit:
        return False


class Log:
    """A decoded flight: columns by name (c), time from the first frame (t), header (hdr) and
    events as (time_us, type, data) tuples."""

    def __init__(self, name):
        lg = bblog().load(find(name))
        self.name = name
        self.a = lg.data
        self.N = list(lg.names)
        raw = lg.meta.get('rawHeader')
        if isinstance(raw, dict):
            self.hdr = {k: str(v) for k, v in raw.items()}
        else:
            self.hdr = {k: (','.join(str(x) for x in v) if isinstance(v, list) else str(v))
                        for k, v in lg.header.items()}
        self.events = []
        for e in lg.events:
            d = e.get('data') or {}
            vals = d['raw'] if 'raw' in d else [v for k, v in d.items() if k != 'label']
            self.events.append((e.get('time'), e.get('type'), vals))
        self.t = (self.c('time') - self.c('time')[0]) / 1e6
        self.dt = float(np.median(np.diff(self.t)))

    def c(self, n):
        return self.a[:, self.N.index(n)].astype(float)
