# -*- coding: utf-8 -*-
"""
Unit test for
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

from EIdrive.core.plan.drive_profile_plotting import *


class TestDriveProfile(unittest.TestCase):
    def setUp(self):
        self.mock_list = [[23, 25, 25, 44, 66], [44, 55, 25, 22, 33]]

    def test_sub_plot(self):
        assert draw_sub_plot(self.mock_list, self.mock_list, self.mock_list, self.mock_list, self.mock_list)