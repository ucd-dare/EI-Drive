# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
import uuid
import carla
from collections import deque

from EIdrive.core.sensing.localization.localization_manager \
    import Localizer
from EIdrive.core.sensing.perception.perception \
    import Perception
from EIdrive.core.plan.agent_behavior \
    import AgentBehavior
from EIdrive.core.map.game_map import GameMap
from EIdrive.core.common.data_dumper import DataDumper
from EIdrive.core.common.misc import draw_trajetory_points
import pandas as pd


class VehicleAgent(object):
    """
    A class manager to embed different modules with vehicle together.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config_yaml : dict
        The configuration dictionary of this CAV.

    application : list
        The delete_application category, currently support:['single','platoon'].

    carla_map : carla.Map
        The CARLA simulation map.

    cav_world : EIdrive object
        CAV World. This is used for V2X communication simulation.

    current_time : str
        Timestamp of the simulation beginning, used for data dumping.

    data_dumping : bool
        Indicates whether to dump sensor data during simulation.

    Attributes
    ----------
    localizer : EIdrive object
        The current localization manager.

    perception_manager : EIdrive object
        The current V2X perception manager.

    agent : EIdrive object
        The current carla agent that handles the basic behavior
         planning of ego vehicle.

    controller : EIdrive object
        The current control manager.

    data_dumper : EIdrive object
        Used for dumping sensor data.
    """

    def __init__(
            self,
            vehicle,
            config_yaml,
            application,
            edge,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=False):

        self.tick = 0

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map
        self.car_id = config_yaml['id'] if 'id' in config_yaml else None
        self.manual_horizon = config_yaml.manual_horizon if 'manual_horizon' in config_yaml else None

        # retrieve to configure for different modules
        localization_config = config_yaml['localization']
        perception_config = config_yaml['perception']
        map_config = config_yaml['game_map']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']

        if edge:
            self.df = pd.read_csv('Edge.csv')
        else:
            self.df = pd.read_csv('No_Edge.csv')

        self.df_records = pd.DataFrame(columns=['x', 'y', 'id', 'tick'])

        # localization module
        self.localizer = Localizer(
            vehicle, localization_config, carla_map)
        # perception module
        self.perception_manager = Perception(
            vehicle, perception_config, cav_world,
            data_dumping)
        # map manager
        self.gamemap = GameMap(vehicle,
                               carla_map,
                               map_config)

        self.agent = AgentBehavior(vehicle, carla_map, behavior_config, control_config)

        # Control module
        if 'is_manually' in behavior_config:
            self.is_manually = behavior_config['is_manually']

        if data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          vehicle.id,
                                          save_time=current_time)
        else:
            self.data_dumper = None

        cav_world.update_vehicle_manager(self)

    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True):
        """
        Set global route.

        Parameters
        ----------
        start_location : carla.location
            The CAV start location.

        end_location : carla.location
            The CAV destination.

        clean : bool
             Indicator of whether clean waypoint queue.

        end_reset : bool
            Indicator of whether reset the end location.
        """

        self.agent.set_local_planner(
            start_location, end_location, clean, end_reset)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        # update the ego pose for map manager
        self.gamemap.set_center(ego_pos)

        self.agent.update_information(ego_pos, ego_spd, objects)

    # def run_step(self, target_speed=None):
    #     """
    #     Execute one step of navigation.
    #     """
    #     # visualize the bev map if needed
    #     self.game_map.run_step()
    #
    #     if hasattr(self, 'is_manually') and self.is_manually:
    #         target_pos = deque()
    #         target_speed = deque()
    #         for i in range(5):  # config_yaml['prediction_horizon']
    #             if self.tick + i <= self.df.shape[0] - 1:
    #                 tem_pos = carla.Transform(carla.Location(
    #                     x=self.df.iloc[self.tick + i][self.car_id * 4],
    #                     y=self.df.iloc[self.tick + i][self.car_id * 4 + 1],
    #                     z=self.df.iloc[self.tick + i][self.car_id * 4 + 2]))
    #                 tem_speed = self.df.iloc[self.tick + i][self.car_id * 4 + 3]
    #             else:
    #                 tem_pos = carla.Transform(carla.Location(
    #                     x=self.df.iloc[-1][self.car_id * 4],
    #                     y=self.df.iloc[-1][self.car_id * 4 + 1],
    #                     z=self.df.iloc[-1][self.car_id * 4 + 2]))
    #                 tem_speed = self.df.iloc[-1][self.car_id * 4 + 3]
    #             target_pos.append(tem_pos)  # here the target_pos is Transform class
    #             target_speed.append(tem_speed)
    #
    #     else:
    #         target_speed, target_pos = self.agent.rule_based_trajectory_planning(
    #             target_speed)  # target_pos is trajectory buffer, target_pos[i][0].location is Location class
    #
    #     control = self.agent.vehicle_control(target_speed, target_pos)
    #
    #     # dump data
    #     if self.data_dumper:
    #         self.data_dumper.save_data(self.perception_manager,
    #                                    self.localizer,
    #                                    self.agent)
    #
    #     self.tick = self.tick + 1
    #
    #     return control

    def destroy(self):
        """
        Destroy the actor vehicle
        """

        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.gamemap.destroy()
