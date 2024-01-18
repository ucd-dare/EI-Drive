"""
The script is used for transmission model test. It is designed for transmission delay.
There will be a firetruck blocks the view.
"""

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener
from EIdrive.scenario_testing.utils.spectator_api import SpectatorController
from EIdrive.scenario_testing.utils.perception_utils import merge_bbx_list, visualize_bbx_by_open3d, \
    ClientSideBoundingBoxes, PygameCamera, VIEW_WIDTH, VIEW_HEIGHT, VIEW_FOV
from EIdrive.core.sensing.perception.dynamic_obstacle import BoundingBox
import open3d as o3d
import numpy as np
import sys
import carla
import pygame
import weakref

player_ids = []


def customized_bp(world):
    """
    Provide customized vehicle blueprints.
    """
    # Vehicle_0 is ego, 1 on left, 2 and 3 is firetruck.
    vehicle_blueprints = []
    vehicle_model = 'vehicle.lincoln.mkz_2020'
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 0')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint.set_attribute('color', '0, 0, 255')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint = world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    vehicle_blueprints.append(vehicle_blueprint)
    vehicle_blueprint = world.get_blueprint_library().find('vehicle.carlamotors.firetruck')
    vehicle_blueprints.append(vehicle_blueprint)
    return vehicle_blueprints


def corner_matrix_transform(corners):
    new = np.zeros((8, 3))
    new[2, :] = corners[0, :]
    new[7, :] = corners[1, :]
    new[1, :] = corners[2, :]
    new[0, :] = corners[3, :]
    new[5, :] = corners[4, :]
    new[4, :] = corners[5, :]
    new[6, :] = corners[6, :]
    new[3, :] = corners[7, :]
    return new


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    coop_perception = True  # With the help of RSU

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town06')

        pygame.init()
        gameDisplay = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption('CARLA Visualization')
        pygame_clock = pygame.time.Clock()

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

            for vehicle_agent in vehicle_list:
                vehicle_agent.update_info()
                sim_api.gamemap_visualize(vehicle_agent)

            for rsu in rsu_list:
                rsu.update_info()

            bbx_list = []

            # Get the ground truth bbx as the perception result from RSU
            rsu_bbx = ClientSideBoundingBoxes._create_bb_points(vehicle_list[1].vehicle)
            location = vehicle_list[1].vehicle.get_transform().location
            rsu_bbx = rsu_bbx[:, :-1]
            rsu_bbx[:, 0] += location.x
            rsu_bbx[:, 1] += location.y
            rsu_bbx[:, 2] += location.z
            rsu_bbx = corner_matrix_transform(rsu_bbx)
            bbx_list.append(rsu_bbx)

            # Get the ground truth bbx as the perception result from ego vehicle
            ego_bbx = ClientSideBoundingBoxes._create_bb_points(vehicle_list[0].vehicle)
            location = vehicle_list[0].vehicle.get_transform().location
            ego_bbx = ego_bbx[:, :-1]
            ego_bbx[:, 0] += location.x
            ego_bbx[:, 1] += location.y
            ego_bbx[:, 2] += location.z
            ego_bbx = corner_matrix_transform(ego_bbx)
            bbx_list.append(ego_bbx)

            # Data fusion
            # bbx_list = merge_bbx_list(bbx_list)

            # Visualize the bbx
            if coop_perception:
                # Visualize data fusion result
                ClientSideBoundingBoxes.draw_only_bbx(cam.display, bbx_list, vehicle_list[0].vehicle,
                                                      cam.camera_actor.calibration, cam.camera_actor, rsu_locations,
                                                      corner_form=True)

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
