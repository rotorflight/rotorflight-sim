"""
Replay check of a trained heli against its prior, into its training report.

    python -I scripts/replay_report.py <rotorflight_SITL> helis/<heli>/heli.json [...]

For each heli with a model.json and held-out logs: replay_check.compare with the prior
(model_params.prior, at the nominal headspeed train.py used) as the old model and model.json
as the new one. The result goes into train.json ("replay") and TRAINING.md (its "Replay
check" section), and replay.json beside them. Slow: two SITL replays per held-out log (at
most two logs), each as long as the flight up to its scored stretch.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import model_params as mp  # noqa: E402
import replay_check  # noqa: E402
import train  # noqa: E402


def report(elf, heli_path, say=print):
    folder = os.path.dirname(os.path.abspath(heli_path))
    heli = json.load(open(heli_path))
    mpath, tpath = os.path.join(folder, 'model.json'), os.path.join(folder, 'train.json')
    if not os.path.exists(mpath) or not os.path.exists(tpath):
        say('%s: no trained model' % heli['name'])
        return None
    model = json.load(open(mpath))
    rep = json.load(open(tpath))
    group = model['provenance']['group']
    val = [e for e in heli['logs'] if e.get('group') == group and e.get('role') == 'validate']
    if not val:
        say('%s: no held-out logs' % heli['name'])
        return None
    prior, _ = mp.prior(heli, model['args'].get('pack_cells'))
    if not (heli.get('preset') or '').lower() in mp.MODELS:
        prior['args']['nominal_rpm'] = model['args']['nominal_rpm']
    new = {k: model[k] for k in ('base', 'args', 'attrs')}
    out = replay_check.compare(elf, heli, prior, new, val, say=say)
    out['old_is'] = 'prior'
    out['model_trained'] = model['provenance'].get('trained')
    out['model_args'] = model['args']
    out['model_attrs'] = model['attrs']
    json.dump(out, open(os.path.join(folder, 'replay.json'), 'w'), indent=1)
    rep['replay'] = out
    json.dump(rep, open(tpath, 'w'), indent=1, default=str)
    train.write_markdown(rep, os.path.join(folder, 'TRAINING.md'))
    say('%s: replay %s (prior %s, trained %s)' % (heli['name'], out['verdict'], out.get('old'), out.get('new')))
    return out


def main():
    elf = sys.argv[1]
    for path in sys.argv[2:]:
        report(elf, path)


if __name__ == '__main__':
    main()
