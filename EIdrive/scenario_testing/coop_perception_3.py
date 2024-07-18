"""
The script is used for the first cooperative perception test.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
from EIdrive.scenario_testing.utils.perception_utils import merge_bbx_list, visualize_bbx_by_open3d, \
    ClientSideBoundingBoxes, PygameCamera, perception_assisted_control, VIEW_WIDTH, VIEW_HEIGHT, VIEW_FOV
from EIdrive.core.sensing.perception.dynamic_obstacle import BoundingBox
import open3d as o3d
import numpy as np
import sys
import carla
import pygame
from EIdrive.imagePrinter import save_pygame

player_ids = []


def customized_bp(world):
    """
    Provide customized vehicle blueprints.
    """
    vehicle_blueprints = []
    vehicle_model = 'vehicle.lincoln.mkz_2020'
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 0') #Ego
    vehicle_blueprints.append(vehicle_blueprint) 

    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 255') # Coop
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '255, 0, 0') # Blocker 1
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    vehicle_blueprints.append(vehicle_blueprint)

    vehicle_blueprint = world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    vehicle_blueprints.append(vehicle_blueprint) # Blocker 3

    return vehicle_blueprints


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    ground_truth_bbx = False  # When it is true, show the ground truth bbx
    coop_perception = False  # With the help of RSU

    control_tick = None

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town04')

        pygame.init()
        gameDisplay = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('CARLA Visualization')
        pygame_clock = pygame.time.Clock()

        # if scenario_params.common_params.record:
        #     gameworld.client.start_recorder("coop_perception.log", True)

        vehicle_blueprints = customized_bp(gameworld.world)
        vehicle_list = gameworld.create_vehicle_agent(vehicle_blueprints)
        rsu_list = gameworld.create_rsu()
        #pedestrian_list = gameworld.create_pedestrian()

        # Spawn pygame camera
        cam = PygameCamera(gameworld.world, vehicle_list[0].vehicle, gameDisplay)
        cam.set_synchronous_mode(True)

        # Draw the position of RSU
        rsu_locations = []
        for rsu in rsu_list:
            location = carla.Location(*rsu.perception.global_position[:3])
            rsu_locations.append(location)
            if coop_perception:
                color = carla.Color(0, 255, 0)
            else:
                color = carla.Color(255, 0, 0)
            gameworld.world.debug.draw_point(location, size=0.5, color=color, life_time=-1.0)

        # Set vehicle stop mode
        vehicle_list[1].stop_mode = True
        #vehicle_list[2].stop_mode = True
        #vehicle_list[3].stop_mode = True
        #vehicle_list[4].stop_mode = True
        #vehicle_list[5].stop_mode = True

        # Keyboard listener
        t = 0
        kl = KeyListener()
        kl.start()

        spectator = gameworld.world.get_spectator()
        spec_controller = SpectatorController(spectator)

        vehicles = gameworld.world.get_actors().filter('vehicle.*')

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

            for vehicle_agent in vehicle_list:
                vehicle_agent.update_info()
                sim_api.gamemap_visualize(vehicle_agent)

            for rsu in rsu_list:
                rsu.update_info()

            bbx_list = []
            true_extent = []
            true_transform = []

            # Add detection result to the list
            if vehicle_list[0].detected_objects['vehicles']:
                bbx = vehicle_list[0].detected_objects['vehicles'][0].bounding_box
                bbx_list.append(bbx)
            if vehicle_list[1].detected_objects['vehicles'] and coop_perception:
                bbx = vehicle_list[1].detected_objects['vehicles'][0].bounding_box
                bbx_list.append(bbx)

            for v in vehicle_list:
                true_extent.append(v.vehicle.bounding_box.extent)
                true_transform.append(v.vehicle.get_transform())

            # Data fusion result
            bbx_list = merge_bbx_list(bbx_list)

            # Visualize the bounding box
            perception_box = [[148, 157],[-11,-7]]
            control_tick_temp = ClientSideBoundingBoxes.VisualizeBBX(ground_truth_bbx, coop_perception, cam, vehicles, bbx_list, vehicle_list, rsu_locations, t, perception_box)
            if control_tick_temp is not None:
                control_tick = control_tick_temp

            spec_controller.bird_view_following(vehicle_list[0].vehicle.get_transform(), altitude=50)

            save_pygame(gameDisplay, t)
            pygame.display.flip()

            # Apply control to vehicles
            
            for vehicle_agent in vehicle_list[:1]:
                control = sim_api.calculate_control(vehicle_agent)

                # Applies additional control on the ego vehicle
                if vehicle_agent == vehicle_list[0]:
                    if not (perception_box[0][0] <= vehicle_agent.vehicle.get_location().x <= perception_box[0][1] and perception_box[1][0] <= vehicle_agent.vehicle.get_location().y <= (perception_box[1][1]+3)):
                        control = perception_assisted_control(control, t, control_tick)
                
                vehicle_agent.vehicle.apply_control(control)

            #for pedestrian in pedestrian_list:
            #    control = carla.WalkerControl()
            #    #control.speed = 1.9
            #    pedestrian.apply_control(control)

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

        cam.destroy()
        pygame.quit()

        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")