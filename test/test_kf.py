# -*- coding: utf-8 -*-
"""
Unit test for Kalman Filter
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import sys
import unittest

import numpy as np

# temporary solution for relative imports in case EIdrive is not installed
# if EIdrive is installed, no need to use the following line
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

import mocked_carla as mcarla
from EIdrive.core.sensing.localization.kalman_filter import KalmanFilter
from EIdrive.core.sensing.localization.convert_coordinate import convert_geo_to_esu


class testKalmanFilter(unittest.TestCase):
    def setUp(self):
        self.dt = 0.25
        self.kf = KalmanFilter(self.dt)
        self.kf.initiate_kf(10, 10, 90, 20)

    def test_parameters(self):
        assert (hasattr(self.kf, 'Q') and
                self.kf.Q.shape == (4, 4))
        assert (hasattr(self.kf, 'R') and
                self.kf.R.shape == (3, 3))
        assert (hasattr(self.kf, 'time_step') and
                self.kf.time_step == self.dt)
        assert (hasattr(self.kf, 'xEst') and
                self.kf.xEst.shape == (4, 1))
        assert (hasattr(self.kf, 'PEst') and
                self.kf.PEst.shape == (4, 4))

    def test_run_step(self):
        assert isinstance(self.kf.run_kf_filter(10, 10, 10, 10, 3)[0], float)
        assert isinstance(self.kf.run_kf_filter(10, 10, 10, 10, 3)[1], float)
        assert isinstance(self.kf.run_kf_filter(10, 10, 10, 10, 3)[2], float)
        assert isinstance(self.kf.run_kf_filter(10, 10, 10, 10, 3)[3], float)

    def test_geo_to_transform(self):
        assert isinstance(convert_geo_to_esu(100, 70, 10, 10, 10, 10)[0], float)
        assert isinstance(convert_geo_to_esu(100, 70, 10, 10, 10, 10)[1], float)
        assert isinstance(convert_geo_to_esu(100, 70, 10.0, 10, 10, 10.0)[2], float)


if __name__ == '__main__':
    unittest.main()