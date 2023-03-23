# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController

import psutil
import time
from multiprocessing import Process

import scenario_runner as sr

def exec_scenario_runner(scenario_params):
    # Initialize and execute scenario-runner
    print(scenario_params.scenario)
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario)
    scenario_runner.run()
    scenario_runner.destroy()


def exec_control(scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None
    client = None

    try:
        client_host = scenario_params.world.client_host
        client_port = scenario_params.world.client_port
        client = carla.Client(client_host, client_port)
        world = client.get_world()

        print("Controller prepared")
        player = None
        leading = None

        while player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    player = vehicle
                if vehicle.attributes['role_name'] == 'scenario':
                    print("Leading vehicle found")
                    leading = vehicle
                
        spectator = player.get_world().get_spectator()
        spec_controller = SpectatorController(spectator)
        
        timestep = 0
        while True:
            world.tick()
            spec_controller.bird_view_following(
                player.get_transform(), altitude=75)
            
            control = leading.get_control()
            distance = leading.get_location().distance(player.get_location())
            velocity = math.sqrt(player.get_velocity().x ** 2 + player.get_velocity().y ** 2)
            if timestep < 50:
                control = carla.VehicleControl(throttle=0.50000, steer=0, brake=0.000000, hand_brake=False,
                                            reverse=False, manual_gear_shift=False, gear=0)
            elif distance < 1.5 * velocity:
                control = carla.VehicleControl(throttle=0, steer=0, brake=1.0, hand_brake=True,
                                            reverse=False, manual_gear_shift=False, gear=0)
            

            player.apply_control(control)
            timestep += 1
            time.sleep(0.05)
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


def run_scenario(opt, scenario_params):
    try:
        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # Create scenario manager
        sim_api.ScenarioManager(scenario_params,
                                opt.apply_ml,
                                opt.edge,
                                opt.version,
                                town=scenario_params.scenario.town,
                                cav_world=cav_world)
        print("Senario manager created")

        # Create a background process to init and execute scenario runner
        sr_process = Process(target=exec_scenario_runner, args=(scenario_params, ))
        # Create a parallel process to init and run EI-Drive
        ei_process = Process(target=exec_control, args=(scenario_params, ))

        sr_process.start()
        ei_process.start()
        
        key_listener = KeyListener()
        key_listener.start()
        
        while True:
            if key_listener.keys['esc']:
                sr_process.kill()
                ei_process.kill()
                # Terminate the main process
                break
            if key_listener.keys['p']:
                psutil.Process(sr_process.pid).suspend()
                psutil.Process(ei_process.pid).suspend()
            if not key_listener.keys['p']:
                psutil.Process(sr_process.pid).resume()
                psutil.Process(ei_process.pid).resume()

    except Exception as e:
        print("Error: {}".format(e))
