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

# def get_latency(single_cav_list):
#     latencies = []
#     points = []
#     for vm in single_cav_list:
#         loc_vehicle = vm.vehicle.get_transform()
#         x = loc_vehicle.location.x
#         y = loc_vehicle.location.y
#         points.append([x, y])
#     for i, point_a in enumerate(points):
#         for j, point_b in enumerate(points):
#             if i > j:
#                 latency = math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_a[1]) ** 2)
#                 latencies.append(latency)
#     return latencies


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # if not opt.edge:
        #     config_yaml['sensing']['edge'] = not config_yaml['sensing']['edge']

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

        spectator = scenario_manager.world.get_spectator()
        spectator.set_transform(carla.Transform(
          carla.Location(
              x=107, y=133, z=75),
          carla.Rotation(
              pitch=-
              90)))

        t = 0
        kl = KeyListener()
        kl.start()
        
        player = None
        leading = None
        
        while player is None or leading is None:
            print("Waiting for vehicles...")
            time.sleep(1)
            possible_vehicles =  scenario_manager.world.get_actors().filter('vehicle.*')
            print(len(possible_vehicles))
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    player = vehicle
                if vehicle.attributes['role_name'] == 'scenario':
                    print("Leadning vehicle found")
                    leading = vehicle
        
        print(leading.get_transform().location)

        spec_controller = SpectatorController(spectator)
        
        while True:
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                continue
            scenario_manager.tick()

            # run step
            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                
            control = single_cav.run_step()
            player.apply_control(control)
            single_cav.vehicle.apply_control(control)
            spec_controller.bird_view_following(player.get_transform())

            # draw figures
            t = t + 1

    finally:

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()