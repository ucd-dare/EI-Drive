"""
Class for vehicle and its modules.
"""

import uuid
from EIdrive.core.sensing.localization.localizer import Localizer
from EIdrive.core.sensing.perception.sensor_perception import Perception
from EIdrive.core.plan.agent_behavior import AgentBehavior
from EIdrive.core.map.game_map import GameMap
from collections import deque
import pandas as pd


class VehicleAgent(object):
    """
    A class represents EI-Drive vehicle. Based on Carla.Vehicle, the VehicleAgent includes several modules,
    such as localizer, perception, and planning.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle.

    config_yaml : dict
        The configuration of the vehicle.

    edge : bool
        Indicator of whether using edge computing.

    carla_map : carla.Map
        The CARLA map.

    ml_model : EIdrive object
        ML model object.

    Attributes
    ----------
    localizer : EIdrive object
        The localization module.

    perception : EIdrive object
        The perception module.

    agent_behavior : EIdrive object
        The behavior planning module

    """

    def __init__(self, vehicle, config_yaml, edge, carla_map, ml_model):

        # Initialize
        self.tick = 0
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map
        self.car_id = config_yaml['id'] if 'id' in config_yaml else None
        self.manual_horizon = config_yaml.manual_horizon if 'manual_horizon' in config_yaml else None
        self.stop_mode = False  # If the vehicle keep still
        self.detected_objects = None
        self.dt = config_yaml['controller']['args']['dt']
        self.detected_objects_queue = deque(maxlen=1000)

        # Split the config yaml file
        localization_config = config_yaml['localization']
        perception_config = config_yaml['perception']
        map_config = config_yaml['game_map']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']

        if edge:
            self.df = pd.read_csv('EIdrive/assets/DemoAsset/Edge.csv')
        else:
            self.df = pd.read_csv('EIdrive/assets/DemoAsset/No_Edge.csv')

        self.df_records = pd.DataFrame(columns=['x', 'y', 'id', 'tick'])

        # Localization module
        self.localizer = Localizer(vehicle, localization_config, carla_map)
        # Perception module
        self.perception = Perception(vehicle, perception_config, ml_model)
        # GameMap module
        self.gamemap = GameMap(vehicle, carla_map, map_config)
        # AgentBehavior module
        self.agent_behavior = AgentBehavior(vehicle, carla_map, behavior_config, control_config)
        # Manual control module
        if 'is_manually' in behavior_config:
            self.is_manually = behavior_config['is_manually']

    def set_local_planner(self, origin, destination, clean=False, end_reset=True):
        """
        Set local planner for the AgentBehavior.

        Parameters
        ----------
        origin : carla.location
            The start location for planning.

        destination : carla.location
            The destination for planning.

        clean : bool
            Whether clean the waypoint queue.

        end_reset : bool
            Whether reset the end location.
        """

        self.agent_behavior.set_local_planner(origin, destination, clean, end_reset)

    def update_info(self):
        """
        Implement localizer and perception. Also, update information for game map and behavior planner.
        """
        # Implement Localization
        self.localizer.localize()

        position = self.localizer.get_ego_pos()
        speed = self.localizer.get_ego_speed()

        # Implement individual object detection
        self.detected_objects = self.perception.object_detect(position)

        # Update the vehicle info for gamemap
        self.gamemap.set_center(position)

        # Update the vehicle info for behavior planner
        self.agent_behavior.update_information(position, speed, self.detected_objects)

    def destroy(self):
        """
        Destroy all the modules.
        """

        self.perception.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.gamemap.destroy()
