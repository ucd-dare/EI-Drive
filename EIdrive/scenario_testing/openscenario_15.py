# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
from carla.libcarla import VehicleControl

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener

import time
from multiprocessing import Process
import psutil

import scenario_runner as sr


def exec_scenario_runner(scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_scenario(scenario_params):
    scenario_runner = None
    cav_world = None
    gameworld = None

    try:
        # Create CAV world
        cav_world = CavWorld()
        # Create scenario manager
        gameworld = sim_api.GameWorld(scenario_params,
                                             scenario_params.common_params.version,
                                             town=scenario_params.scenario.scenario_runner.town,
                                             cav_world=cav_world)

        # Create a background process to init and execute scenario runner
        sr_process = Process(target=exec_scenario_runner,
                             args=(scenario_params,))
        sr_process.start()

        key_listener = KeyListener()
        key_listener.start()

        world = gameworld.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')

        single_cav_list = gameworld.create_vehicle_agent_from_scenario_runner(
            vehicle=ego_vehicle,
        )

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 100
        spectator_bird_pitch = -90

        while True:
            if key_listener.keys['esc']:
                sr_process.kill()
                # Terminate the main process
                return
            if key_listener.keys['p']:
                psutil.Process(sr_process.pid).suspend()
                continue
            if not key_listener.keys['p']:
                psutil.Process(sr_process.pid).resume()

            gameworld.tick(single_cav_list)
            ego_cav = single_cav_list[0].vehicle

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            time.sleep(0.01)
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

