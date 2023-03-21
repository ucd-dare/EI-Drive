import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.utils.yaml_utils import load_yaml
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController

import time
from multiprocessing import Process
import logging

import scenario_runner as sr


class SRArguments:
    additionalScenario = ''
    agent = None
    agentConfig = ''
    configFile = ''
    debug = False
    file = False
    host = '127.0.0.1'
    json = False
    junit = False
    list = False
    openscenario = None
    penscenarioparams = None
    output = False
    outputDir = ''
    port = '2000'
    randomize = False
    record = ''
    reloadWorld = False
    repetitions = 1
    route = None
    scenario = 'FollowLeadingVehicle_1'
    sync = False
    timeout = 20.0
    trafficManagerPort = '8000'
    trafficManagerSeed = '0'
    waitForEgo = False

    def __init__(self):
        pass


def exec_sr():
    # Initialize and execute scenario-runner
    scenario_runner = sr.ScenarioRunner(SRArguments())
    scenario_runner.run()


def exec_ei(opt, config_yaml):
    # Initialize and execute EI-Driver
    scenario_params = load_yaml(config_yaml)
    # Create CAV world
    cav_world = CavWorld(opt.apply_ml)
    # Create scenario manager
    scenario_manager = sim_api.ScenarioManager(scenario_params,
                                               opt.apply_ml,
                                               opt.edge,
                                               opt.version,
                                               town='Town01',
                                               cav_world=cav_world)
    logging.debug("Senario manager created")
    world = scenario_manager.world
    logging.debug("World obtained", world)
    spectator = world.get_spectator()
    spec_controller = SpectatorController(spectator)

    player = None

    while player is None:
        print("Waiting for the ego vehicle...")
        time.sleep(1)
        possible_vehicles = world.get_actors().filter('vehicle.*')
        for vehicle in possible_vehicles:
            if vehicle.attributes['role_name'] == 'hero':
                player = vehicle
                print("Ego vehicle found")
                break
    key_listener = KeyListener()
    key_listener.start()
    timestep = 0
    while True:
        if key_listener.keys['esc']:
            break
        if key_listener.keys['p']:
            time.sleep(0.5)
            continue

        scenario_manager.tick()
        spec_controller.bird_view_following(
            player.get_transform(), altitude=75)

        if timestep < 50:
            control = carla.VehicleControl(throttle=0.50000, steer=0, brake=0.000000, hand_brake=False,
                                           reverse=False, manual_gear_shift=False, gear=0)

        player.apply_control(control)
        timestep += 1
        time.sleep(0.03)


def run_scenario(opt, config_yaml):
    scenario_runner = None
    cav_world = None
    scenario_manager = None

    try:
        # Create a background process to init and execute scenario runner
        sr_process = Process(target=exec_sr, args=())
        # Create a parallel process to init and run EI-Drive
        ei_process = Process(target=exec_ei, args=(opt, config_yaml, ))

        sr_process.start()
        ei_process.start()

        sr_process.join()
        ei_process.join()

    except Exception as e:
        print("Error: {}".format(e))

    finally:
        print("Cleaning up...")
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
