"""
The script is used for multiple destination test.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
import sys


player_ids = []


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town06')

        if scenario_params.common_params.record:
            gameworld.client.start_recorder("demo_loop.log", True)

        vehicle_list = gameworld.create_vehicle_agent()

        spectator = gameworld.world.get_spectator()

        # Keyboard listener
        t = 0
        kl = KeyListener()
        kl.start()

        spec_controller = SpectatorController(spectator)

        while True:
            # Pause and exit
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                continue

            gameworld.tick(vehicle_list)

            spec_controller.bird_view_following(vehicle_list[0].vehicle.get_transform())

            t = t + 1
            if 3000 == t:
                sys.exit(0)

    finally:
        if gameworld is not None:
            gameworld.close()
        print("Destroyed gameworld")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
