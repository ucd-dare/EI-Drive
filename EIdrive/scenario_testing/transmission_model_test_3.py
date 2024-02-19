"""
The script is used for transmission model test. It is designed for transmission latency.
There will be a firetruck blocks the view.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
from EIdrive.scenario_testing.utils.perception_utils import merge_bbx_list, visualize_bbx_by_open3d, \
    ClientSideBoundingBoxes, PygameCamera, VIEW_WIDTH, VIEW_HEIGHT, VIEW_FOV
from EIdrive.core.sensing.perception.dynamic_obstacle import BoundingBox
from collections import deque
import open3d as o3d
import numpy as np
import sys
import carla
import pygame
import math

player_ids = []


def customized_bp(world):
    """
    Provide customized vehicle blueprints.
    """
    # Vehicle_0 on bottom, 1 on left, 2 is firetruck, 3 on top.
    vehicle_blueprints = []
    vehicle_model = 'vehicle.lincoln.mkz_2020'
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 0')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '255, 0, 0')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint = world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    vehicle_blueprints.append(vehicle_blueprint)
    return vehicle_blueprints


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    coop_perception = True  # With the help of RSU

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town03')

        pygame.init()
        gameDisplay = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('CARLA Visualization')
        pygame_clock = pygame.time.Clock()

        font_size = 80
        font = pygame.font.SysFont(None, font_size)
        text_color = (0, 255, 0)  # Green color

        right_margin = 1500
        top_margin = 200

        if scenario_params.common_params.record:
            gameworld.client.start_recorder("coop_perception.log", True)

        vehicle_blueprints = customized_bp(gameworld.world)
        vehicle_list = gameworld.create_vehicle_agent(vehicle_blueprints)
        rsu_list = gameworld.create_rsu()

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
        vehicle_list[2].stop_mode = True

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

            # Render text
            vehicle_latency = vehicle_list[0].perception.transmission_latency_in_sec
            rsu_latency = rsu_list[0].perception.transmission_latency_in_sec
            rsu_text = font.render(f"RSU latency is {rsu_latency}", True, text_color)
            v_text = font.render(f"Vehicle latency is {vehicle_latency}", True, text_color)
            first_line_rect = rsu_text.get_rect()
            second_line_rect = v_text.get_rect()
            rsu_text_position = VIEW_WIDTH - right_margin, top_margin
            gap = 20
            v_text_position = (VIEW_WIDTH - right_margin,
                            top_margin + first_line_rect.height + gap)
            gameDisplay.blit(rsu_text, rsu_text_position)
            gameDisplay.blit(v_text, v_text_position)

            for index, vehicle_agent in enumerate(vehicle_list):
                if index == 0 and t < 160:
                    vehicle_agent.update_info(1)
                elif index == 0 and t >= 160:
                    vehicle_agent.update_info(0.2)
                else:
                    vehicle_agent.update_info()
                sim_api.gamemap_visualize(vehicle_agent)

            for rsu in rsu_list:
                if t < 160:
                    rsu.update_info(1)
                else:
                    rsu.update_info(0.2)

            bbx_list = []
            true_extent = []
            true_transform = []

            # Add detection result to the list
            if vehicle_list[0].detected_objects['vehicles']:
                bbx = vehicle_list[0].detected_objects['vehicles'][0].bounding_box
                bbx_list.append(bbx)
            if rsu_list[0].detected_objects['vehicles'] and coop_perception:
                bbx = rsu_list[0].detected_objects['vehicles'][0].bounding_box
                bbx_list.append(bbx)

            for v in vehicle_list:
                true_extent.append(v.vehicle.bounding_box.extent)
                true_transform.append(v.vehicle.get_transform())

            # Data fusion result
            bbx_list = merge_bbx_list(bbx_list)

            # Visualize the bbx
            if coop_perception:
                # Visualize data fusion result
                ClientSideBoundingBoxes.draw_only_bbx(cam.display, bbx_list, vehicle_list[0].vehicle,
                                                      cam.camera_actor.calibration, cam.camera_actor, rsu_locations)
            else:
                # Visualize perception result only from ego vehicle
                ClientSideBoundingBoxes.draw_only_bbx(cam.display, bbx_list, vehicle_list[0].vehicle,
                                                      cam.camera_actor.calibration, cam.camera_actor)

            spec_controller.bird_view_following(vehicle_list[0].vehicle.get_transform(), altitude=50)

            pygame.display.flip()

            # Apply control to vehicles
            for vehicle_agent in vehicle_list:
                control = sim_api.calculate_control(vehicle_agent)
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

        cam.destroy()
        pygame.quit()

        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
