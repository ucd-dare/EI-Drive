"""
The script is used for the eighth cooperative perception test.

The ego vehicle is approachign an intersection. Travel buses partially block its view of an incoming vehicle.
A spectator vehicle and two RSUs provides additional info to the ego vehicle.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
from EIdrive.scenario_testing.utils.perception_utils import manage_bbx_list, ClientSideBoundingBoxes, \
    PygameCamera, perception_assisted_control, VIEW_WIDTH, VIEW_HEIGHT
from EIdrive.scenario_testing.utils.display_utils import display_latency, display_rsu
import sys
import pygame

player_ids = []


def customized_bp(world):
    """
    Provide customized vehicle blueprints.
    """
    vehicle_blueprints = []
    vehicle_model = 'vehicle.lincoln.mkz_2020'
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 0')
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '255, 0, 0')
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 255')
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint = world.get_blueprint_library().find('vehicle.mitsubishi.fusorosa')
    vehicle_blueprints.append(vehicle_blueprint)
    
    return vehicle_blueprints


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    control_tick = None

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town06')

        text_viz = scenario_params['scenario']['text_viz'] \
            if 'text_viz' in scenario_params['scenario'] else True

        pygame.init()
        gameDisplay = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('CARLA Visualization')
        pygame_clock = pygame.time.Clock()

        vehicle_blueprints = customized_bp(gameworld.world)
        vehicle_list = gameworld.create_vehicle_agent(vehicle_blueprints)
        rsu_list = gameworld.create_rsu()

        # Spawn pygame camera
        cam = PygameCamera(gameworld.world, vehicle_list[0].vehicle, gameDisplay)
        cam.set_synchronous_mode(True)

        # Draw the position of RSU
        rsu_locations = display_rsu(rsu_list, gameworld)        

        # Create background traffic
        traffic_manager, flow_list = gameworld.create_traffic_flow()

        # Set vehicle stop mode
        stopped_vehicles = [1, 3,4]
        for i in stopped_vehicles:
            vehicle_list[i].stop_mode = True

        # Keyboard listener
        t = 0
        kl = KeyListener()
        kl.start()

        spectator = gameworld.world.get_spectator()
        spec_controller = SpectatorController(spectator)

        pedestrians = gameworld.world.get_actors().filter('walker.*')
        perception_box = [[-11.3, -8.1 ], [49, 54]]
        bbx_visualizer = ClientSideBoundingBoxes(vehicle_list, rsu_list, pedestrians, rsu_locations, perception_box)

        while True:

            # Pause and exit
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                continue

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    cam.destroy()

            # Tick and update info
            gameworld.tick()
            cam.capture = True
            pygame_clock.tick_busy_loop(60)
            cam.render(cam.display)

            display_latency(vehicle_list, rsu_list, gameDisplay, VIEW_WIDTH)

            for vehicle_agent in vehicle_list:
                vehicle_agent.update_info()
                sim_api.gamemap_visualize(vehicle_agent)

            for rsu in rsu_list:
                rsu.update_info()

            bbx_list = manage_bbx_list(vehicle_list, rsu_list)

            # Visualize the bounding box
            vehicles = gameworld.world.get_actors().filter('vehicle.*')
            control_tick_temp = bbx_visualizer.VisualizeBBX(cam, vehicles, bbx_list, t, text_viz)
            if control_tick_temp is not None:
                control_tick = control_tick_temp

            spec_controller.bird_view_following(vehicle_list[0].vehicle.get_transform(), altitude=50)

            pygame.display.flip()

            # Apply control to vehicles
            
            for vehicle_agent in vehicle_list:
                control = sim_api.calculate_control(vehicle_agent)

                # Applies additional control on the ego vehicle
                if vehicle_agent == vehicle_list[0]:
                    control = perception_assisted_control(control, t, control_tick)
                
                vehicle_agent.vehicle.apply_control(control)

            t = t + 1
            if 3000 == t:
                sys.exit(0)

    finally:
        if gameworld is not None:
            gameworld.close()
        print("Destroyed gameworld")

        for v in vehicle_list:
            v.destroy()
        for rsu in rsu_list:
            rsu.destroy()
        for v in flow_list:
            v.destroy()

        cam.destroy()
        pygame.quit()

        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")