"""
The script is used for testing the sensor, perception, and planning modules of EI Drive with a Red Light scenario.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
import sys
import carla


player_ids = []

def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town03')

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
            gameworld.tick()

            for vehicle_agent in vehicle_list:
                vehicle_agent.update_info()
                sim_api.gamemap_visualize(vehicle_agent)
                control = sim_api.calculate_control(vehicle_agent)
                vehicle_agent.vehicle.apply_control(control)


                traffic_light = vehicle_agent.vehicle.get_traffic_light()

                if t < 300 and traffic_light is not None:
                    traffic_light.set_state(carla.TrafficLightState.Red)
                if t > 300 and traffic_light is not None:
                    traffic_light.set_state(carla.TrafficLightState.Green)
                    
            
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
