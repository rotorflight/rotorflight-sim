"""Tests of intake.py's log checks that need no decoded log (fast):

    python -m unittest discover -s fitting/tests -v
"""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
from intake import gps_check  # noqa: E402


class Frames:
    """The GPS frames as intake reads them: columns by name, and .data for the count."""

    def __init__(self, t_s, sats, speed):
        self.cols = {'time': np.asarray(t_s, float) * 1e6, 'GPS_numSat': np.asarray(sats),
                     'GPS_speed_mps': np.asarray(speed, float)}
        self.data = np.zeros((len(t_s), 3))

    def __getitem__(self, name):
        return self.cols[name]


class GpsCheck(unittest.TestCase):

    def test_one_frame(self):
        r = gps_check(Frames([0.0], [10], [5.0]))
        self.assertEqual(r, {'gps': False, 'gps_moving_s': 0.0})

    def test_uneven_frames(self):
        # 20 s moving at 1 Hz, then 200 frames at 10 Hz without a fix: the 0.1 s median
        # interval must not shrink the moving time
        t = list(np.arange(20.0)) + list(20.0 + 0.1 * np.arange(200))
        sats = [10] * 20 + [3] * 200
        speed = [5.0] * 20 + [0.0] * 200
        r = gps_check(Frames(t, sats, speed))
        self.assertTrue(r['gps'])
        self.assertAlmostEqual(r['gps_moving_s'], 20.0)

    def test_gap_is_capped(self):
        # a 60 s gap in the frames counts for 2 s at most
        r = gps_check(Frames([0.0, 1.0, 61.0, 62.0], [10] * 4, [5.0] * 4))
        self.assertFalse(r['gps'])
        self.assertAlmostEqual(r['gps_moving_s'], 1.0 + 2.0 + 1.0 + 1.0)


if __name__ == '__main__':
    unittest.main()
