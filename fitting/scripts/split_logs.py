"""Split multi-log BBL files at their 'H Product:' headers; skip duplicates (same bytes).

    python -I scripts/split_logs.py data/split <log.bbl> [...]

Output: <date>_<time>_<n>.bbl per log (the RTFL_BLACKBOX_LOG_<craft>_ prefix dropped).
"""
import hashlib, os, re, sys
out = sys.argv[1]
seen = {}
for path in sys.argv[2:]:
    raw = open(path, 'rb').read()
    marks = []
    i = raw.find(b'H Product:')
    while i >= 0:
        marks.append(i)
        i = raw.find(b'H Product:', i + 1)
    base = re.sub(r'^RTFL_BLACKBOX_LOG_.*?_(?=\d{8}_)', '', os.path.basename(path)).rsplit('.', 1)[0]
    for k, start in enumerate(marks):
        seg = raw[start:marks[k + 1] if k + 1 < len(marks) else len(raw)]
        h = hashlib.md5(seg[:200000]).hexdigest()
        name = '%s_%02d.bbl' % (base, k)
        if h in seen:
            print('%s = %s (duplicate, skipped)' % (name, seen[h])); continue
        seen[h] = name
        open(os.path.join(out, name), 'wb').write(seg)
        print('%s %8d bytes' % (name, len(seg)))
