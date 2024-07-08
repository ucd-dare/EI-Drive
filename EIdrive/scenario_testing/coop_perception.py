"""
The script is used for cooperative perception test.
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
    vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
    vehicle_blueprint.set_attribute('color', '0, 0, 255')
    vehicle_blueprints.append(vehicle_blueprint)
    return vehicle_blueprints


def run_scenario(scenario_params):
    scenario_runner = None
    gameworld = None

    ground_truth_bbx = False  # When it is true, show the ground truth bbx
    coop_perception = False  # With the help of RSU

    trigger_tick = None

    try:
        # Create game world
        gameworld = sim_api.GameWorld(scenario_params, map_name='town03')

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
            if ground_truth_bbx:
                # Get z-coordinate of the base of the ego vehicle's bounding box
                ego_vehicle_bbox = ClientSideBoundingBoxes.get_bounding_box(cam.vehicle, cam.camera_actor)
                ego_base_z = ego_vehicle_bbox[0, 2] - cam.vehicle.bounding_box.extent.z

                in_sight_vehicles = [vehicle for vehicle in vehicles if
                                    vehicle.id != cam.vehicle.id and cam.is_in_sight(vehicle, cam.camera_actor)]
                bounding_boxes = [(vehicle.id, ClientSideBoundingBoxes.get_bounding_box(vehicle, cam.camera_actor)) for
                                vehicle in in_sight_vehicles]
                bounding_boxes = [(vehicle_id, bbox) for vehicle_id, bbox in bounding_boxes if
                                bbox[4, 2] <= ego_base_z + 50]

                ego_vehicle_position = (int(ego_vehicle_bbox[0, 0]), int(ego_vehicle_bbox[0, 1]))

                vehicle_info = {}
                for vehicle in in_sight_vehicles:
                    vehicle_info[vehicle.id] = {
                        'location': (vehicle.get_location().x, vehicle.get_location().y),
                        'speed': 3.6 * vehicle.get_velocity().length()  # convert m/s to km/h
                    }

                ClientSideBoundingBoxes.draw_ground_truth_bbx(cam.display, bounding_boxes, ego_vehicle_position,
                                                            vehicle_info, ego_bbox=ego_vehicle_bbox,
                                                            line_between_vehicle=False)
            elif coop_perception:
                # Visualize data fusion result
                ClientSideBoundingBoxes.draw_only_bbx(cam.display, bbx_list, vehicle_list[0].vehicle,
                                                    cam.camera_actor.calibration, cam.camera_actor, rsu_locations)
                
            else:
                # Visualize perception result only from ego vehicle
                ClientSideBoundingBoxes.draw_only_bbx(cam.display, bbx_list, vehicle_list[0].vehicle,
                                                    cam.camera_actor.calibration, cam.camera_actor)
                
            # If a bounding box is in the trigger area, set the current time as the starting point for a brake
            for bbox in bbx_list:
                corners = bbox.corners
                corners = np.vstack((corners.T, np.ones(corners.shape[0])))
                center_x = np.mean(corners[0, :])
                center_y = np.mean(corners[1, :])
                if -77 <= center_x <= -73 and -133 <= center_y <= -125:
                    trigger_tick = t

            spec_controller.bird_view_following(vehicle_list[0].vehicle.get_transform(), altitude=50)

            pygame.display.flip()

            # Apply control to vehicles
            
            for vehicle_agent in vehicle_list:
                control = sim_api.calculate_control(vehicle_agent)
                
                # Brakes when a bounding box is in the trigger area and for 25 ticks after the bounding box leaves the area
                if trigger_tick is not None and t - trigger_tick < 35 and vehicle_agent == vehicle_list[0]:
                    control.brake = 0.05
                    control.throttle = 0

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