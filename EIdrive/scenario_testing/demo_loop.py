# -*- coding: utf-8 -*-
# Author: Wei Shao <phdweishao@gmail.com>
# License: TDG-Attribution-NonCommercial-NoDistribute

import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from EIdrive.scenario_testing.utils.yaml_utils import load_yaml
from EIdrive.core.common.misc import get_speed
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import time

from pynput import keyboard

player_ids = []


def run_scenario(opt, scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None

    try:
        # create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.edge,
                                                   opt.version,
                                                   town='town06',
                                                   cav_world=cav_world)
        if opt.record:
            scenario_manager.client.start_recorder("demo_loop.log", True)

        single_cav_list = scenario_manager.create_vehicle_manager(application=['single'])

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='demo',
                              current_time='')
        spectator = scenario_manager.world.get_spectator()

        # run steps
        t = 0
        kl = KeyListener()
        kl.start()

        spec_controller = SpectatorController(spectator)

        while True:
            ## pause and exit
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                continue

            scenario_manager.tick()

            spec_controller.bird_view_following(single_cav_list[0].vehicle.get_transform())

            # run step
            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

            t = t + 1

            if 3000 == t:
                sys.exit(0)

    finally:
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.close()
        print("Destroyed scenario_manager")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
