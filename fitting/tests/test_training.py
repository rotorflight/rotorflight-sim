"""
Tests of the training pipeline on synthetic flights (synthetic.py): a heli of known
parameters is flown, logged and fitted, and the numbers must come back. Needs numpy and
scipy; about five minutes (most of it flying the synthetic logs):

    python -m unittest discover -s fitting/tests -v
"""
import copy
import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import fit_flap  # noqa: E402
import fit_heli  # noqa: E402
import holdout  # noqa: E402
import jackknife  # noqa: E402
import model_params as mp  # noqa: E402
import train  # noqa: E402
from fingerprint import group_id, setup  # noqa: E402
from synthetic import TRUTH, fly  # noqa: E402

SHORT = dict(rpms=(1800, 2100), hover_s=9.0, noise_s=20.0, doublets=6)


def perturbed():
    """Another heli: every fitted number off by 20-50 %."""
    p = copy.deepcopy(TRUTH)
    a, g = p['args'], p['attrs']
    a['rate_gain_roll'] *= 1.3
    a['rate_gain_pitch'] *= 0.75
    a['Lb1s'] *= 1.6
    a['Ma1s'] *= 0.6
    a['tf_roll'] *= 1.5
    a['yaw_trim'] *= 1.5
    a['hover_power_w'] *= 1.5
    g['motor_fit'] = [g['motor_fit'][0] * 1.25] + g['motor_fit'][1:]
    g['thrust_inflow'] = [g['thrust_inflow'][0] * 0.75, g['thrust_inflow'][1]]
    g['Nped'] *= 1.4
    return p


class Training(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.fit_logs = [fly(name='a%d' % i, seed=i, **SHORT) for i in (1, 2, 3)]
        cls.val_logs = [fly(name='v1', seed=11, **SHORT)]
        cls.other_logs = [fly(perturbed(), name='b%d' % i, seed=20 + i, **SHORT) for i in (1, 2)]
        cls.by_name = {lg.name: lg for lg in cls.fit_logs + cls.val_logs + cls.other_logs}
        cls.prep = fit_heli.prepare(cls.fit_logs)
        cls.fit = fit_heli.fit(cls.prep, verbose=False)
        cls.gid = group_id(setup(cls.fit_logs[0])[0])

    # ---- the stages -----------------------------------------------------------------------
    def test_rotor_law(self):
        A, B = self.fit['motor_fit'][:2]
        tA, tB = TRUTH['attrs']['motor_fit'][:2]
        self.assertGreater(self.fit['motor_fit_r2'], 0.95)
        self.assertAlmostEqual(A / tA, 1.0, delta=0.08)
        self.assertAlmostEqual(B / tB, 1.0, delta=0.08)

    def test_rate_gains(self):
        # the spectral gain fits the flap/body magnitude: the plain median of |G| read
        # 15-25 % high on this heli (its mode sits in the band)
        for rpm in (1800, 2100):
            for ax in ('roll', 'pitch'):
                with self.subTest(rpm=rpm, axis=ax):
                    g = self.fit['rate_gains']['%d' % rpm][ax]['gain']
                    self.assertAlmostEqual(g / mp.rate_gain(TRUTH, ax, rpm), 1.0, delta=0.04)

    def test_hover_collective(self):
        for rpm in (1800, 2100):
            with self.subTest(rpm=rpm):
                h = self.fit['hover']['%d' % rpm]
                self.assertEqual(h['gate'], 'ok')
                # the height hold moves the collective a little around the trim
                self.assertAlmostEqual(h['collective'], mp.hover_collective(TRUTH, rpm), delta=0.012)

    def test_yaw(self):
        for rpm in (1800, 2100):
            with self.subTest(rpm=rpm):
                y = self.fit['yaw']['%d' % rpm]
                self.assertAlmostEqual(y['N_ped'] / (TRUTH['attrs']['Nped'] * (rpm / 1800) ** 2), 1.0, delta=0.05)

    def test_flap(self):
        per, _, dt = fit_flap.collect(self.fit_logs)
        for (axn, lv), wins in per.items():
            ax = 'roll' if axn == 0 else 'pitch'
            with self.subTest(axis=ax, rpm=lv):
                f = fit_flap.fit(wins, dt)
                K, L, tf = mp.flap_at(TRUTH, ax, lv)
                self.assertAlmostEqual(f['K'] / K, 1.0, delta=0.04)
                self.assertAlmostEqual(f['L'] / L, 1.0, delta=0.12)
                self.assertAlmostEqual(f['tf'] / tf, 1.0, delta=0.12)

    # ---- uncertainty, held-out scoring, split ---------------------------------------
    def test_jackknife(self):
        folds = jackknife.folds(self.prep, 4)
        for li in range(len(self.prep)):
            left_out = sum((~f[li]).astype(int) for f in folds)
            self.assertTrue(np.all(left_out == 1))          # every sample left out exactly once
        reps = [fit_heli.fit(self.prep, keep=k, verbose=False) for k in folds]
        se, unstable = jackknife.spread(self.fit, reps)
        self.assertLess(se['rate_gains/1800/roll/gain'] / self.fit['rate_gains']['1800']['roll']['gain'], 0.05)
        self.assertLess(se['motor_fit/0'] / self.fit['motor_fit'][0], 0.10)

    def test_holdout_prefers_the_truth(self):
        m = holdout.measured(fit_heli.prepare(self.val_logs))
        good, bad = holdout.score(TRUTH, m), holdout.score(perturbed(), m)
        for st in ('rotor', 'thrust', 'rate', 'flap', 'yaw'):
            with self.subTest(stage=st):
                self.assertIsNotNone(good[st][0], good[st][1])
                self.assertLess(good[st][0], bad[st][0])

    def test_roles_are_stable(self):
        names = [{'file': 'log%d.0' % i} for i in range(400)]
        roles = [train.role_for(e) for e in names]
        self.assertEqual(roles, [train.role_for(e) for e in names])
        self.assertAlmostEqual(roles.count('validate') / len(roles), 0.25, delta=0.06)

    def test_setup_groups(self):
        lg = self.fit_logs[0]
        s, _ = setup(lg)
        moved = dict(lg.hdr, **{'Craft name': 'Other'})
        lg2 = copy.copy(lg)
        lg2.hdr = moved
        self.assertEqual(group_id(s), group_id(setup(self.fit_logs[1])[0]))
        self.assertNotEqual(group_id(s), group_id(setup(lg2)[0]))

    # ---- the whole loop -------------------------------------------------------------------
    def run_train(self, fit_names, val_names, model=None, folds=4):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d)
        logs = [{'file': n, 'group': self.gid, 'role': 'fit', 'start': '2026-11-08T12:%02d' % i,
                 'intake': {'air_s': 60.0}} for i, n in enumerate(fit_names)]
        logs += [{'file': n, 'group': self.gid, 'role': 'validate', 'start': '2026-11-08T13:%02d' % i,
                  'intake': {'air_s': 60.0}} for i, n in enumerate(val_names)]
        heli = {'name': 'Synth 700', 'size_class': 700, 'tail': {'type': 'servo'}, 'preset': None,
                'groups': {self.gid: setup(self.fit_logs[0])[0]}, 'logs': logs}
        path = os.path.join(d, 'heli.json')
        with open(path, 'w') as f:
            json.dump(heli, f)
        if model:
            with open(os.path.join(d, 'model.json'), 'w') as f:
                json.dump(dict(model, provenance={'group': self.gid, 'trained': 'earlier', 'params': {}}), f)
        with mock.patch.object(fit_heli, 'load_logs', lambda names: [self.by_name[n] for n in names]):
            rep = train.train(path, folds=folds, say=lambda *a: None)
        mpath = os.path.join(d, 'model.json')
        return rep, (json.load(open(mpath)) if os.path.exists(mpath) else None)

    def test_train_from_a_size_prior(self):
        rep, model = self.run_train(['a1', 'a2', 'a3'], ['v1'])
        self.assertTrue(rep['changed'])
        # compared at the same rpm: the model's nominal is the most flown headspeed
        for rpm in (1800, 2100):
            for ax in ('roll', 'pitch'):
                with self.subTest(rpm=rpm, axis=ax):
                    K, L, tf = mp.flap_at(model, ax, rpm)
                    tK, tL, ttf = mp.flap_at(TRUTH, ax, rpm)
                    self.assertAlmostEqual(K / tK, 1.0, delta=0.05)
                    self.assertAlmostEqual(L / tL, 1.0, delta=0.15)
                    self.assertAlmostEqual(tf / ttf, 1.0, delta=0.15)
            self.assertAlmostEqual(mp.hover_collective(model, rpm), mp.hover_collective(TRUTH, rpm), delta=0.012)
        for st, v in rep['scores']['new'].items():
            if v is not None and rep['scores']['prior'][st] is not None:
                self.assertLessEqual(v, rep['scores']['prior'][st] * 1.02 + 1e-9, st)
        self.assertTrue(all(s.startswith(('fit', 'size prior', 'most flown', 'estimated', 'heli file'))
                            for s in model['provenance']['params'].values()))

    def test_train_never_takes_a_worse_model(self):
        # logs of another heli against held-out logs of the trained one: every proposal
        # scores worse there and must be refused
        truth = {k: TRUTH[k] for k in ('base', 'args', 'attrs')}
        rep, model = self.run_train(['b1', 'b2'], ['v1'], model=truth)
        self.assertFalse(rep['changed'], json.dumps({s: r['decision'] for s, r in rep['stages'].items()}, indent=1))
        self.assertEqual(model['args'], TRUTH['args'])


if __name__ == '__main__':
    unittest.main()
