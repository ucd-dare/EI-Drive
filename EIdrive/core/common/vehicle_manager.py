# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import carla
from collections import deque

from EIdrive.core.actuation.control_manager \
    import ControlManager
from EIdrive.core.common.v2x_manager \
    import V2XManager
from EIdrive.core.sensing.localization.localization_manager \
    import LocalizationManager
from EIdrive.core.sensing.perception.perception_manager \
    import PerceptionManager
from EIdrive.core.plan.behavior_agent \
    import BehaviorAgent
from EIdrive.core.map.map_manager import MapManager
from EIdrive.core.common.data_dumper import DataDumper
from EIdrive.core.common.misc import draw_trajetory_points
import pandas as pd

# df = pd.read_csv('Edge.csv')
# df = pd.read_csv('No_Edge.csv')

class VehicleManager(object):
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
    v2x_manager : EIdrive object
        The current V2X manager.

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

        # retrieve to configure for different modules
        sensing_config = config_yaml['sensing']
        map_config = config_yaml['map_manager']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        if edge:
            self.df = pd.read_csv('Edge.csv')
        else:
            self.df = pd.read_csv('No_Edge.csv')

        self.df_records = pd.DataFrame(columns=['x', 'y', 'id', 'tick'])
        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], cav_world,
            data_dumping)
        # map manager
        self.map_manager = MapManager(vehicle,
                                      carla_map,
                                      map_config)

        self.agent = BehaviorAgent(vehicle, carla_map, behavior_config)

        # Control module
        self.controller = ControlManager(control_config)
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

        Returns
        -------
        """

        self.agent.set_destination(
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
        self.map_manager.update_information(ego_pos)

        # update ego position and speed to v2x manager,
        # and then v2x manager will search the nearby cavs
        self.v2x_manager.update_info(ego_pos, ego_spd)

        self.agent.update_information(ego_pos, ego_spd, objects)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        # visualize the bev map if needed
        self.map_manager.run_step()

        if self.is_manually:
            """
            # The original version
            target_pos = carla.Location(
                x=self.df.iloc[self.tick][self.car_id * 4], y=self.df.iloc[self.tick][self.car_id * 4 + 1],
                z=self.df.iloc[self.tick][self.car_id * 4 + 2])
            target_speed = self.df.iloc[self.tick][self.car_id * 4 + 3]
            """

            target_pos = deque()
            target_speed = deque()
            for i in range(5):   # config_yaml['prediction_horizon']
                if self.tick + i <= self.df.shape[0] - 1:
                    tem_pos = carla.Transform(carla.Location(
                        x=self.df.iloc[self.tick+i][self.car_id * 4],
                        y=self.df.iloc[self.tick+i][self.car_id * 4 + 1],
                        z=self.df.iloc[self.tick+i][self.car_id * 4 + 2]))
                    tem_speed = self.df.iloc[self.tick+i][self.car_id * 4 + 3]
                else:
                    tem_pos = carla.Transform(carla.Location(
                        x=self.df.iloc[-1][self.car_id * 4],
                        y=self.df.iloc[-1][self.car_id * 4 + 1],
                        z=self.df.iloc[-1][self.car_id * 4 + 2]))
                    tem_speed = self.df.iloc[-1][self.car_id * 4 + 3]
                target_pos.append(tem_pos)  # here the target_pos is Transform class
                target_speed.append(tem_speed)


        else:
            target_speed, target_pos = self.agent.run_step(target_speed)    # target_pos is trajectory buffer, target_pos[i][0].location is Location class

        # list_row = {"x": self.localizer.get_ego_pos().location.x, "y": self.localizer.get_ego_pos().location.y, "id": self.car_id, "tick": self.tick}
        # self.df_records = self.df_records.append(list_row, ignore_index=True)
        # if 199 == self.tick:
        #     self.df_records.to_csv(str(self.car_id) + "traj.csv", index=False)

        # localizer.get_ego_pos()
        # print(target_speed)
        # print(self.localizer.get_ego_spd())

        control = self.controller.run_step(target_speed, target_pos)

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        self.tick = self.tick + 1

        return control

    def destroy(self):
        """
        Destroy the actor vehicle
        """

        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.map_manager.destroy()
