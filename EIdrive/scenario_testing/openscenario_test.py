import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
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


def run_scenario(opt, scenario_params):
    # create CAV world
    cav_world = CavWorld(opt.apply_ml)
    # create scenario manager
    scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                opt.apply_ml,
                                                opt.edge,
                                                opt.version,
                                                town='Town01',
                                                cav_world=cav_world)
    world = scenario_manager.world;
    spectator = world.get_spectator()

    try:
        kl = KeyListener()
        kl.start()

        spec_controller = SpectatorController(spectator)

        player = None
        leading = None

        while player is None or leading is None:
            print("Waiting for vehicles...")
            time.sleep(1)
            possible_vehicles = world.get_actors().filter('vehicle.*')
            print(len(possible_vehicles))
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    player = vehicle
                if vehicle.attributes['role_name'] == 'scenario':
                    print("Leading vehicle found")
                    leading = vehicle
        t = 0
        while True:
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                time.sleep(0.5)
                continue

            scenario_manager.tick()
            spec_controller.bird_view_following(
                player.get_transform(), altitude=75)
            control = leading.get_control()

            if t < 50:
                control = carla.VehicleControl(throttle=0.30000, steer=0, brake=0.000000, hand_brake=False,
                                               reverse=False, manual_gear_shift=False, gear=0)

            player.apply_control(control)
            time.sleep(0.03)
            t = t + 1

    except Exception as error:
        # logging.exception(error)
        print("Error caught", error)
        
    finally:
        print("Destroying cav_world...")
        if cav_world is not None:
            cav_world.destroy()
        if scenario_manager is not None:
            scenario_manager.destroyActors()
            scenario_manager.close()
        print("Destroying scenario_runner...")
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner