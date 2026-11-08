"""
The log manifest of a heli file, and the private store the logs live in.

    python -I scripts/logstore.py check helis/<heli>/heli.json      # decoded? in the store?
    python -I scripts/logstore.py backfill helis/<heli>/heli.json   # add source + sha256
    python -I scripts/logstore.py decode helis/<heli>/heli.json     # decode what is missing

Logs stay out of the repository (a GPS track holds where someone flies, and not every
owner agrees to publish). The heli file is the manifest instead: each log entry names its
source file and the file's SHA-256, and says whether the owner agreed to share it:

  {"file": "<stem>.<n>",          the decoded log (FIT_DATA_DIR)
   "source": "<stem>.bbl",        the blackbox file it came from
   "sha256": "...",               of the source file: the same bytes, the same fit
   "consent": "private",          private (default) | public
   "group": "<setup id>",         fingerprint.py (or set by hand)
   "role": "fit" | "validate"}    train.py: validate logs are held out of the fits

FIT_LOG_STORE names the folders the source files are found in (os.pathsep separated,
searched recursively, matched by hash - renamed files are found too). An index of the
hashes is kept beside the decoded logs (FIT_DATA_DIR/.logstore.json) so only new or
changed files are hashed again.
"""
import hashlib
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import DATA, SKILL, has_log  # noqa: E402

STORE = [d for d in os.environ.get('FIT_LOG_STORE', '').split(os.pathsep) if d]
INDEX = os.path.join(DATA, '.logstore.json')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def index():
    """sha256 -> path of every .bbl in the store."""
    try:
        cache = json.load(open(INDEX))
    except (OSError, ValueError):
        cache = {}
    out, fresh = {}, {}
    for root in STORE:
        for dp, _, files in os.walk(root):
            for fn in files:
                if not fn.lower().endswith(('.bbl', '.bfl')):
                    continue
                p = os.path.abspath(os.path.join(dp, fn))
                st = os.stat(p)
                key = '%s|%d|%d' % (p, st.st_size, int(st.st_mtime))
                h = cache.get(key) or sha256(p)
                fresh[key] = h
                out.setdefault(h, p)
    if fresh != cache and os.path.isdir(DATA):
        with open(INDEX, 'w') as f:
            json.dump(fresh, f, indent=0)
    return out


def find_file(name):
    """Any file of the store by name (a CLI dump), or None."""
    want = os.path.basename(name).lower()
    for root in STORE:
        for dp, _, files in os.walk(root):
            for fn in files:
                if fn.lower() == want:
                    return os.path.abspath(os.path.join(dp, fn))
    return None


def source_name(entry):
    """The source file an entry came from: its "source", else <stem>.bbl from the decoded
    name (<stem>.<n>, or <stem> for a file with one log)."""
    if entry.get('source'):
        return entry['source']
    name = os.path.splitext(entry['file'])[0] if entry['file'].endswith('.bbl') else entry['file']
    stem, _, n = name.rpartition('.')
    return (stem if n.isdigit() and stem else name) + '.bbl'


def log_index(entry):
    name = entry['file'][:-4] if entry['file'].endswith('.bbl') else entry['file']
    stem, _, n = name.rpartition('.')
    return int(n) if n.isdigit() and stem else 0


def locate(entry, idx=None):
    """The source file of an entry in the store: by hash, else by name (None if missing)."""
    idx = index() if idx is None else idx
    if entry.get('sha256') in idx:
        return idx[entry['sha256']]
    want = source_name(entry).lower()
    for p in idx.values():
        if os.path.basename(p).lower() == want:
            return p
    return None


def decoded_name(entry):
    return entry['file'][:-4] if entry['file'].endswith('.bbl') else entry['file']


def decode(path):
    """Decode a source file into DATA with the skill's bbl.js."""
    os.makedirs(DATA, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path))[0]
    js = os.path.join(SKILL, 'bbl.js')
    subprocess.run([os.environ.get('NODE', 'node'), js, 'decode', path, '-o', os.path.join(DATA, stem)],
                   check=True, capture_output=True)


def save(heli, path):
    """Write a heli file: serialised first, then replaced in one step (a failure leaves the
    old file)."""
    text = json.dumps(heli, indent=2) + '\n'
    tmp = path + '.tmp'
    with open(tmp, 'w', newline='\n') as f:
        f.write(text)
    os.replace(tmp, path)


def main():
    cmd, path = sys.argv[1], sys.argv[2]
    heli = json.load(open(path))
    idx = index()
    changed = False
    for e in heli['logs']:
        src = locate(e, idx)
        name = decoded_name(e)
        ok_hash = src is not None and (not e.get('sha256') or sha256(src) == e['sha256'])
        if cmd == 'check':
            print('%-36s decoded %-3s source %s%s' % (name, 'yes' if has_log(name) else 'NO',
                                                      src or 'NOT IN STORE', '' if ok_hash or src is None else
                                                      '  (hash differs from the manifest!)'))
        elif cmd == 'backfill' and src is not None:
            h = sha256(src)
            if e.get('sha256') not in (None, h):
                print('%s: the store file %s does not match the manifest hash - left alone' % (name, src))
                continue
            if e.get('sha256') != h or 'source' not in e:
                e['source'], e['sha256'] = os.path.basename(src), h
                e.setdefault('consent', 'private')
                changed = True
        elif cmd == 'decode' and not has_log(name):
            if src is None:
                print('%s: not in the store (FIT_LOG_STORE)' % name)
                continue
            print('decode %s' % src)
            decode(src)
    if changed:
        save(heli, path)
        print('manifest updated:', path)


if __name__ == '__main__':
    main()
