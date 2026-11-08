"""
Delete-a-group jackknife for the fit stages: how much a fitted number depends on which
part of the data it saw.

The airborne time of all logs is cut into blocks (20 s), and the blocks are dealt round
robin into K folds. Each fold is left out once and the stages fitted on the rest; the
spread of the K refits gives the standard error of each number:

    se = sqrt((K - 1) / K * sum((x_i - mean)^2))

Blocks rather than single samples, because neighbouring samples are not independent (a
hover, a manoeuvre); round robin rather than one contiguous stretch per fold, so every fold
sees every flight and the refits do not drop whole headspeeds. A number that one fold
cannot fit at all (a hover gate that fails without it, a headspeed whose data falls into
one fold) is reported as unstable: the data that identifies it is too thin.
"""
import numpy as np

BLOCK_S = 20.0


def folds(prep, K=8, block_s=BLOCK_S):
    """K keep-lists (one bool mask per log) for fit_heli.fit, each leaving one fold out."""
    marks, n = [], 0
    for p in prep:
        lg = p['log']
        blk = np.floor(lg.t / block_s).astype(int)
        marks.append((blk + n) % K)
        n += int(blk[-1]) + 1
    return [[m != k for m in marks] for k in range(K)]


def leaves(d, prefix=''):
    """Numeric leaves of a nested dict/list as {'a/b/0': value}."""
    out = {}
    items = d.items() if isinstance(d, dict) else enumerate(d) if isinstance(d, list) else ()
    for k, v in items:
        path = '%s/%s' % (prefix, k) if prefix else str(k)
        if isinstance(v, bool) or v is None or isinstance(v, str):
            continue
        if isinstance(v, (int, float)):
            out[path] = float(v)
        else:
            out.update(leaves(v, path))
    return out


def spread(full, reps):
    """Standard errors of the numeric leaves of full, from the K refits reps. Returns
    {path: se} and the paths that a refit could not produce (unstable)."""
    f = leaves(full)
    rs = [leaves(r) for r in reps]
    K = len(rs)
    se, unstable = {}, []
    for path in f:
        xs = [r[path] for r in rs if path in r]
        if len(xs) < K:
            unstable.append(path)
            continue
        x = np.array(xs)
        se[path] = float(np.sqrt((K - 1) / K * np.sum((x - x.mean()) ** 2)))
    return se, unstable


def rel(se, full, path):
    """Relative standard error of a leaf (inf when unknown or unstable)."""
    v = leaves(full).get(path)
    if v is None or path not in se or v == 0:
        return float('inf')
    return se[path] / abs(v)
