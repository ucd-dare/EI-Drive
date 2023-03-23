import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from EIdrive.core.common.misc import get_speed
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import time


def run_scenario(opt, scenario_params):
    try:
        # create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.edge,
                                                   opt.version,
                                                   town='Town06',
                                                   cav_world=cav_world)
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        traffic_manager = \
            scenario_manager.create_traffic_carla()
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='mpc_dynamics_training',
                              current_time=scenario_params['current_time'])
        spectator = scenario_manager.world.get_spectator()

        kl = KeyListener()
        kl.start()

        while True:
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                time.sleep(0.5)
                continue

            spectator_pos = single_cav_list[0].vehicle.get_transform()
            spectator_pos.location.z += 30
            spectator_pos.rotation.pitch = -90
            spectator.set_transform(spectator_pos)
            scenario_manager.tick()
            # print(get_speed(single_cav_list[0].vehicle))
            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        # eval_manager.evaluate()
        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()