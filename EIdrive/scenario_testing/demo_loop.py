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


def run_scenario(scenario_params):
    scenario_runner = None
    cav_world = None
    gameworld = None

    try:
        # create CAV world
        cav_world = CavWorld()
        # create scenario manager
        gameworld = sim_api.GameWorld(scenario_params,
                                             town='town06',
                                             cav_world=cav_world)
        if scenario_params.common_params.record:
            gameworld.client.start_recorder("demo_loop.log", True)

        single_cav_list = gameworld.create_vehicle_agent(application=['single'], data_dump=False)

        # create evaluation manager
        eval_manager = \
            EvaluationManager(gameworld.cav_world,
                              script_name='demo',
                              current_time='')
        spectator = gameworld.world.get_spectator()

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

            gameworld.tick(single_cav_list)

            spec_controller.bird_view_following(single_cav_list[0].vehicle.get_transform())

            t = t + 1

            if 3000 == t:
                sys.exit(0)

    finally:
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if gameworld is not None:
            gameworld.close()
        print("Destroyed gameworld")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
