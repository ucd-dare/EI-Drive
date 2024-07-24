"""
This file includes GameWorld, which describes the whole environment in simulation. Also, there are some functions
for script visualization and calculation.
"""

import math
import random
import sys
import json
import cv2
from random import shuffle
import carla
import numpy as np
from omegaconf import OmegaConf
from collections import deque

from EIdrive.core.basic.vehicle_agent import VehicleAgent
from EIdrive.core.basic.rsu import RSU
from EIdrive.scenario_testing.utils.customized_map import create_customized_world, bcolors
from EIdrive.core.basic.ml_model import MLModel
import EIdrive.imagePrinter as IP

def filter_blueprint_lib(blueprint_library):
    """
    Remove the non-standard vehicles from the CARLA blueprint collection.

    Parameters
    ----------
    blueprint_library : carla.blueprint_library
        The collection of all available vehicle models.

    Returns
    -------
    blueprints : list
        The list of suitable blueprints for vehicles.
    """

    blueprints = [
        blueprint_library.find('vehicle.audi.a2'),
        blueprint_library.find('vehicle.audi.tt'),
        blueprint_library.find('vehicle.dodge.charger_police'),
        blueprint_library.find('vehicle.dodge.charger_police_2020'),
        blueprint_library.find('vehicle.dodge.charger_2020'),
        blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
        blueprint_library.find('vehicle.chevrolet.impala'),
        blueprint_library.find('vehicle.mini.cooper_s'),
        blueprint_library.find('vehicle.audi.etron'),
        blueprint_library.find('vehicle.mercedes.coupe'),
        blueprint_library.find('vehicle.mercedes.coupe_2020'),
        blueprint_library.find('vehicle.bmw.grandtourer'),
        blueprint_library.find('vehicle.toyota.prius'),
        blueprint_library.find('vehicle.citroen.c3'),
        blueprint_library.find('vehicle.ford.mustang'),
        blueprint_library.find('vehicle.tesla.model3'),
        blueprint_library.find('vehicle.lincoln.mkz_2017'),
        blueprint_library.find('vehicle.lincoln.mkz_2020'),
        blueprint_library.find('vehicle.seat.leon'),
        blueprint_library.find('vehicle.nissan.patrol'),
        blueprint_library.find('vehicle.nissan.micra'),
    ]

    return blueprints


def multi_class_vehicle_blueprint_filter(label, blueprint_library, blueprint_dict):
    """
    Fetch a list of blueprints whose class matches the provided label.

    Parameters
    ----------
    label : str
        Provided blueprint.

    blueprint_library : carla.blueprint_library
        The collection of all available vehicle models.

    blueprint_dict : dict
        Dictionary of {blueprint name: blueprint class}.

    Returns
    -------
    blueprints : list
        List of blueprints whose class matches with the provided label.

    """
    blueprints = [blueprint_library.find(k) for k, v in blueprint_dict.items() if v["class"] == label]
    return blueprints


def gamemap_visualize(vehicle):
    """
    Visualize the gamemap.

    Parameters
    ----------
    vehicle : carla.vehicle
        The carla vehicle agent.

    """
    if not vehicle.gamemap.activate:
        return
    vehicle.gamemap.render_static_agents()
    vehicle.gamemap.render_dynamic_agents()
    if vehicle.gamemap.visualize:
        cv2.imshow('the bev map of agent %s' % vehicle.gamemap.agent_id, vehicle.gamemap.vis_bev)
        cv2.waitKey(1)


def calculate_control(vehicle):
    """
    Calculate the vehicle control based on trajectory and speed buffer. If it is manual control, then the trajectory
    is given by provided files.

    Parameters
    ----------
    vehicle : carla.vehicle
        The carla vehicle agent.

    Returns
    -------

    """
    trajectory_buffer = deque()
    target_speed = deque()

    if hasattr(vehicle, 'is_manually') and vehicle.is_manually:
        for i in range(vehicle.manual_horizon):
            if vehicle.tick + i <= vehicle.df.shape[0] - 1:
                tem_pos = carla.Transform(carla.Location(
                    x=vehicle.df.iloc[vehicle.tick + i][vehicle.car_id * 4],
                    y=vehicle.df.iloc[vehicle.tick + i][vehicle.car_id * 4 + 1],
                    z=vehicle.df.iloc[vehicle.tick + i][vehicle.car_id * 4 + 2]))
                tem_speed = vehicle.df.iloc[vehicle.tick + i][vehicle.car_id * 4 + 3]
            else:
                tem_pos = carla.Transform(carla.Location(
                    x=vehicle.df.iloc[-1][vehicle.car_id * 4],
                    y=vehicle.df.iloc[-1][vehicle.car_id * 4 + 1],
                    z=vehicle.df.iloc[-1][vehicle.car_id * 4 + 2]))
                tem_speed = vehicle.df.iloc[-1][vehicle.car_id * 4 + 3]
            trajectory_buffer.append(tem_pos)  # here the trajectory_buffer is Transform class
            target_speed.append(tem_speed)

    elif not vehicle.stop_mode:
        target_speed, trajectory_buffer = vehicle.agent_behavior.rule_based_trajectory_planning(target_speed)
    else:
        # When the stop model is True, stop planning
        target_speed = None
        trajectory_buffer = None

    control = vehicle.agent_behavior.vehicle_control(target_speed, trajectory_buffer)

    if vehicle.stop_mode:
        control.brake = 1.0

    vehicle.tick = vehicle.tick + 1

    return control


class GameWorld:
    """
    The class that contains map, vehicle, and all simulation config. All the agents run under the frame of GameWorld.

    Parameters
    ----------
    scenario_params : dict
        All simulation configurations.

    xodr_path : str
        The xodr file path to the customized map.

    map_name : str
        Map name if not using customized map, eg. 'Town06'.

    Attributes
    ----------
    client : carla.client
        The client that connects to carla server.

    world : carla.world
        Carla simulation server.

    origin_settings : dict
        The origin setting of the simulation server.

    ml_model : EIdrive object
        ML model object.

    carla_map : carla.map
        Carla map.

    """

    def __init__(self, scenario_params,
                 edge=False,
                 xodr_path=None,
                 map_name=None):
        self.scenario_params = scenario_params
        self.carla_version = scenario_params.common_params.version

        world_config = scenario_params['world']

        # Set random seed
        if 'seed' in world_config:
            np.random.seed(world_config['seed'])
            random.seed(world_config['seed'])

        self.client = carla.Client('localhost', world_config['client_port'])
        self.client.set_timeout(10.0)

        if xodr_path:
            self.world = create_customized_world(xodr_path, self.client)
        elif map_name:
            try:
                self.world = self.client.load_world(map_name)
            except RuntimeError:
                print(
                    f"{bcolors.FAIL} %s is not present in your CARLA repository! "
                    f"Ensure all maps are available in your CARLA repository!"
                    f"{bcolors.ENDC}" % map_name)
        else:
            self.world = self.client.get_world()

        if not self.world:
            sys.exit('World creating failed')

        self.origin_settings = self.world.get_settings()
        new_settings = self.world.get_settings()

        new_settings.no_rendering_mode = world_config['no_rendering_mode']

        if world_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = world_config['fixed_delta_seconds']
        else:
            sys.exit('ERROR: Current version only supports sync simulation mode')

        self.world.apply_settings(new_settings)

        # Weather settings
        weather = self.set_weather(world_config['weather'])
        self.world.set_weather(weather)

        # Configuring blueprint probabilities. It is used to create multiple background traffic.
        self.multi_blueprint_used = scenario_params["blueprint"][
            'use_multi_class_bp'] if 'blueprint' in scenario_params else False
        if self.multi_blueprint_used:
            with open(scenario_params['blueprint']['bp_meta_path']) as f:
                self.blueprint_data = json.load(f)
            self.bp_class_sample_prob = scenario_params['blueprint']['bp_class_sample_prob']

            # Normalize the probability
            self.bp_class_sample_prob = {
                k: v / sum(self.bp_class_sample_prob.values()) for k, v in
                self.bp_class_sample_prob.items()}

        try:
            apply_ml = scenario_params.vehicle_perception.perception.apply_ml is True
        except AttributeError:
            apply_ml = False

        self.ml_model = MLModel(apply_ml=apply_ml)
        self.carla_map = self.world.get_map()
        self.edge = edge

    @staticmethod
    def set_weather(weather_config):
        """
        Set the weather in simulation.

        Parameters
        ----------
        weather_config : dict
            The configuration for weather.

        Returns
        -------
        weather :
        The CARLA weather setting.
        """
        weather = carla.WeatherParameters(
            sun_altitude_angle=weather_config['sun_altitude_angle'],
            cloudiness=weather_config['cloudiness'],
            precipitation=weather_config['precipitation'],
            precipitation_deposits=weather_config['precipitation_deposits'],
            wind_intensity=weather_config['wind_intensity'],
            fog_density=weather_config['fog_density'],
            fog_distance=weather_config['fog_distance'],
            fog_falloff=weather_config['fog_falloff'],
            wetness=weather_config['wetness']
        )
        return weather

    def create_vehicle_agent(self, vehicle_blueprints='default'):
        """
        Create a list of vehicle.

        Returns
        -------
        vehicle_list : list
            A list of all vehicle agents.

        """
        print('Creating vehicles.')

        # Use default vehicle settings if vehicle blueprints are not given
        if vehicle_blueprints == 'default':
            vehicle_blueprints = []
            vehicle_model = 'vehicle.lincoln.mkz_2020'
            # vehicle_blueprint = self.world.get_blueprint_library().find(vehicle_model)
            for i in range(len(self.scenario_params['scenario']['vehicle_list'])):
                print(i)
                vehicle_blueprint = self.world.get_blueprint_library().find(vehicle_model)
                if i % 3 == 0:
                    vehicle_blueprint.set_attribute('color', '0, 0, 255')
                elif i % 3 == 1:
                    vehicle_blueprint.set_attribute('color', '255, 0, 0')
                elif i % 3 == 2:
                    vehicle_blueprint.set_attribute('color', '0, 255, 0')
                vehicle_blueprints.append(vehicle_blueprint)

        vehicle_list = []

        for i, vehicle_config in enumerate(self.scenario_params['scenario']['vehicle_list']):
            vehicle_config = OmegaConf.merge(self.scenario_params['vehicle_perception'], vehicle_config)
            vehicle_config = OmegaConf.merge(self.scenario_params['vehicle_localization'], vehicle_config)
            vehicle_config = OmegaConf.merge(self.scenario_params['behavior'], vehicle_config)
            vehicle_config = OmegaConf.merge(self.scenario_params['game_map'], vehicle_config)
            vehicle_config = OmegaConf.merge(self.scenario_params['controller'], vehicle_config)
            vehicle_config = OmegaConf.merge(self.scenario_params['scenario'], vehicle_config)

            spawn_transform = carla.Transform(
                carla.Location(
                    x=vehicle_config['spawn_position'][0],
                    y=vehicle_config['spawn_position'][1],
                    z=vehicle_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=vehicle_config['spawn_position'][5],
                    yaw=vehicle_config['spawn_position'][4],
                    roll=vehicle_config['spawn_position'][3]))

            vehicle = self.world.spawn_actor(vehicle_blueprints[i], spawn_transform)

            if 'id' in vehicle_config:
                vehicle_config['perception']['vid'] = vehicle_config['id']

            # Create vehicle agent.
            vehicle_agent = VehicleAgent(vehicle, vehicle_config, self.edge, self.carla_map, self.ml_model)

            self.world.tick()

            destinations = []
            for destination in vehicle_config['destination']:
                location = carla.Location(x=destination[0],
                                          y=destination[1],
                                          z=destination[2])
                destinations.append(location)

            vehicle_agent.update_info()
            vehicle_agent.set_local_planner(vehicle_agent.vehicle.get_location(), destinations[-1], clean=True)
            vehicle_list.append(vehicle_agent)

        return vehicle_list

    def create_vehicle_agent_from_scenario_runner(self, vehicle):
        """
        Create vehicle agent with a loaded ego vehicle from SR.
        Different from the create_vehicle_manager API creating Carla vehicle from scratch,
        SR creates on its own only supports 'single' vehicle.

        Parameters
        ----------
        vehicle:
            The Carla ego vehicle created by ScenarioRunner.

        Returns
        -------
        vehicle_list : list
            A list contains the vehicle agent derived from the ego vehicle.
        """
        vehicle_config = self.scenario_params['scenario']['vehicle_list']
        if len(vehicle_config) != 1:
            raise ValueError('Only support one ego vehicle for ScenarioRunner')

        vehicle_config = vehicle_config[0]
        vehicle_config = OmegaConf.merge(self.scenario_params['vehicle_perception'], vehicle_config)
        vehicle_config = OmegaConf.merge(self.scenario_params['vehicle_localization'], vehicle_config)
        vehicle_config = OmegaConf.merge(self.scenario_params['behavior'], vehicle_config)
        vehicle_config = OmegaConf.merge(self.scenario_params['game_map'], vehicle_config)
        vehicle_config = OmegaConf.merge(self.scenario_params['controller'], vehicle_config)
        vehicle_agent = VehicleAgent(vehicle, vehicle_config, False, self.carla_map, self.ml_model)

        self.world.tick()

        destinations = []
        for destination in vehicle_config['destination']:
            location = carla.Location(x=destination[0],
                                      y=destination[1],
                                      z=destination[2])
            destinations.append(location)

        vehicle_agent.update_info()
        vehicle_agent.set_local_planner(
            vehicle_agent.vehicle.get_location(),
            destinations[-1],
            clean=True)

        return [vehicle_agent]

    def create_rsu(self):
        """
        Create a list of RSU.

        Returns
        -------
        rsu_list : list
            A list of all RSU.
            
        """
        print('Creating RSU.')
        rsu_list = []
        if 'rsu_list' not in self.scenario_params['scenario']:
            return rsu_list

        for i, rsu_config in enumerate(self.scenario_params['scenario']['rsu_list']):
            rsu_config = OmegaConf.merge(self.scenario_params['vehicle_perception'], rsu_config)
            rsu_config = OmegaConf.merge(self.scenario_params['vehicle_localization'], rsu_config)

            rsu_manager = RSU(self.world, rsu_config,
                              self.carla_map,
                              self.ml_model
                              )

            rsu_list.append(rsu_manager)

        return rsu_list
    
    def create_pedestrian(self):
        """
        Create a list of pedestrian.

        Returns
        -------
        pedestrian_list : list
            A list of all pedestrian agents.

        """
        print('Creating pedestrians.')

        pedestrian_list = []
        if 'pedestrian_list' not in self.scenario_params['scenario']:
            return pedestrian_list

        for i, pedestrian_config in enumerate(self.scenario_params['scenario']['pedestrian_list']):

            spawn_transform = carla.Transform(
                carla.Location(
                    x=pedestrian_config['spawn_position'][0],
                    y=pedestrian_config['spawn_position'][1],
                    z=pedestrian_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=pedestrian_config['spawn_position'][5],
                    yaw=pedestrian_config['spawn_position'][4],
                    roll=pedestrian_config['spawn_position'][3]))

            pedestrian = self.world.spawn_actor(self.world.get_blueprint_library().find('walker.pedestrian.0001'), spawn_transform)
            pedestrian_list.append(pedestrian)

        return pedestrian_list

    def create_vehicles_by_list(self, traffic_manager, traffic_config, background_traffic):
        """
        Spawn the background traffic vehicles according to the provided list.

        Parameters
        ----------
        traffic_manager : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        background_traffic : list
            The list of all background traffic vehicle.

        Returns
        -------
        background_traffic : list
            Update background traffic list.
        """

        blueprint_library = self.world.get_blueprint_library()
        if not self.multi_blueprint_used:
            ego_vehicle_random_list = filter_blueprint_lib(blueprint_library)
        else:
            label_list = list(self.bp_class_sample_prob.keys())
            prob = [self.bp_class_sample_prob[label] for label in label_list]

        # Default color
        color = '0, 255, 0'
        default_model = 'vehicle.lincoln.mkz2017' if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'
        ego_vehicle_blueprint = blueprint_library.find(default_model)

        for i, vehicle_config in enumerate(traffic_config['vehicle_list']):
            spawn_transform = carla.Transform(
                carla.Location(
                    x=vehicle_config['spawn_position'][0],
                    y=vehicle_config['spawn_position'][1],
                    z=vehicle_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=vehicle_config['spawn_position'][5],
                    yaw=vehicle_config['spawn_position'][4],
                    roll=vehicle_config['spawn_position'][3]))

            if not traffic_config['random']:
                ego_vehicle_blueprint.set_attribute('color', color)

            else:
                # Sample an ego vehicle blueprint from a list with given probability
                if self.multi_blueprint_used:
                    label = np.random.choice(label_list, p=prob)
                    # Given the label (class), find all associated blueprints in CARLA
                    ego_vehicle_random_list = multi_class_vehicle_blueprint_filter(
                        label, blueprint_library, self.blueprint_data)
                ego_vehicle_blueprint = random.choice(ego_vehicle_random_list)

                if ego_vehicle_blueprint.has_attribute("color"):
                    color = random.choice(ego_vehicle_blueprint.get_attribute('color').recommended_values)
                    ego_vehicle_blueprint.set_attribute('color', color)

            vehicle = self.world.spawn_actor(ego_vehicle_blueprint, spawn_transform)
            vehicle.set_autopilot(True, 8000)

            if 'vehicle_speed_perc' in vehicle_config:
                traffic_manager.vehicle_percentage_speed_difference(
                    vehicle, vehicle_config['vehicle_speed_perc'])
            traffic_manager.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            background_traffic.append(vehicle)

        return background_traffic

    def create_vehicle_by_range(self, traffic_manager, traffic_config, background_traffic):
        """
        Spawn the background traffic vehicles by a provided range.

        Parameters
        ----------
        traffic_manager : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        background_traffic : list
            The list of all background traffic vehicles.

        Returns
        -------
        background_traffic : list
            Update traffic list.
        """
        blueprint_library = self.world.get_blueprint_library()
        if not self.multi_blueprint_used:
            ego_vehicle_random_list = filter_blueprint_lib(blueprint_library)
        else:
            label_list = list(self.bp_class_sample_prob.keys())
            prob = [self.bp_class_sample_prob[label] for label in label_list]

        # Default color
        color = '0, 255, 0'
        default_model = 'vehicle.lincoln.mkz2017' if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'
        ego_vehicle_blueprint = blueprint_library.find(default_model)

        spawn_ranges = traffic_config['range']
        spawn_set = set()
        spawn_number = 0

        for spawn_range in spawn_ranges:
            spawn_number += spawn_range[6]
            x_min, x_max, y_min, y_max = \
                math.floor(spawn_range[0]), math.ceil(spawn_range[1]), \
                    math.floor(spawn_range[2]), math.ceil(spawn_range[3])

            for x in range(x_min, x_max, int(spawn_range[4])):
                for y in range(y_min, y_max, int(spawn_range[5])):
                    location = carla.Location(x=x, y=y, z=0.3)
                    waypoint = self.carla_map.get_waypoint(location).transform

                    spawn_set.add((waypoint.location.x,
                                   waypoint.location.y,
                                   waypoint.location.z,
                                   waypoint.rotation.roll,
                                   waypoint.rotation.yaw,
                                   waypoint.rotation.pitch))
        count = 0
        spawn_list = list(spawn_set)
        shuffle(spawn_list)

        while count < spawn_number:
            if len(spawn_list) == 0:
                break

            coordinates = spawn_list[0]
            spawn_list.pop(0)

            spawn_transform = carla.Transform(carla.Location(x=coordinates[0],
                                                             y=coordinates[1],
                                                             z=coordinates[2] + 0.3),
                                              carla.Rotation(
                                                  roll=coordinates[3],
                                                  yaw=coordinates[4],
                                                  pitch=coordinates[5]))
            if not traffic_config['random']:
                ego_vehicle_blueprint.set_attribute('color', color)

            else:
                # Sample an ego vehicle blueprint from a list with given probability
                if self.multi_blueprint_used:
                    label = np.random.choice(label_list, p=prob)
                    # Given the label (class), find all associated blueprints in CARLA
                    ego_vehicle_random_list = multi_class_vehicle_blueprint_filter(
                        label, blueprint_library, self.blueprint_data)
                ego_vehicle_blueprint = random.choice(ego_vehicle_random_list)
                if ego_vehicle_blueprint.has_attribute("color"):
                    color = random.choice(ego_vehicle_blueprint.get_attribute('color').recommended_values)
                    ego_vehicle_blueprint.set_attribute('color', color)

            vehicle = self.world.try_spawn_actor(ego_vehicle_blueprint, spawn_transform)

            if not vehicle:
                continue

            vehicle.set_autopilot(True, 8000)
            traffic_manager.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            if 'ignore_lights_percentage' in traffic_config:
                traffic_manager.ignore_lights_percentage(vehicle, traffic_config['ignore_lights_percentage'])

            # Slight difference in speed between vehicles
            traffic_manager.vehicle_percentage_speed_difference(
                vehicle,
                traffic_config['global_speed_perc'] + random.randint(-30, 30))

            background_traffic.append(vehicle)
            count += 1

        return background_traffic

    def create_traffic_flow(self):
        """
        Create traffic flow, including traffic manager and background traffic.

        Returns
        -------
        traffic_manager : carla.traffic_manager
            Carla traffic manager.

        background_traffic : list
            The list of all the background traffic vehicles.
        """
        print('Creating traffic flow.')
        traffic_config = self.scenario_params['traffic_manager']['carla_traffic_manager']
        traffic_manager = self.client.get_trafficmanager()

        traffic_manager.set_global_distance_to_leading_vehicle(traffic_config['global_distance'])
        traffic_manager.set_synchronous_mode(traffic_config['sync_mode'])
        traffic_manager.set_osm_mode(traffic_config['set_osm_mode'])
        traffic_manager.global_percentage_speed_difference(traffic_config['global_speed_perc'])

        background_traffic = []

        if isinstance(traffic_config['vehicle_list'], list):
            background_traffic = self.create_vehicles_by_list(traffic_manager, traffic_config, background_traffic)
        else:
            background_traffic = self.create_vehicle_by_range(traffic_manager, traffic_config, background_traffic)

        print('CARLA traffic flow has been created.')
        return traffic_manager, background_traffic

    def tick(self):
        """
        Tick the server.
            
        """
        self.world.tick()


    def destroy_actors(self):
        """
        Destroy all actors in the world.
        """

        actor_list = self.world.get_actors()
        for actor in actor_list:
            actor.destroy()

    def close(self):
        """
        Simulation close.
        """
        
        # restore to origin setting
        self.world.apply_settings(self.origin_settings)
