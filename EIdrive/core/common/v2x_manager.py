# -*- coding: utf-8 -*-
# Removed platooning code
"""Communication manager for cooperation
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from collections import deque
import weakref

import carla
import numpy as np


from EIdrive.core.common.misc import compute_distance


class V2XManager(object):
    """
    V2X Manager for platooning, cooperative perception and so on.

    Parameters
    ----------
    cav_world : EIdrive object
        CAV world.

    config_yaml : dict
        The configuration dictionary of the v2x module.

    vid : str
        The corresponding vehicle manager's uuid.

    Attributes
    ----------
    _recieved_buffer : dict
        A buffer for receive data.

    cav_nearby : dict
        The dictionary that contains the cavs in the communication range.

    platooning_plugin : EIdrive object
        The platooning plugin for communication during platooning.

    ego_pos : carla.transform
        Ego position.

    ego_spd : float
        Ego speed(km/h).

    """

    def __init__(self, cav_world, config_yaml, vid):
        # if disabled, no cooperation will be operated
        self.cda_enabled = config_yaml['enabled']
        self.communication_range = config_yaml['communication_range']

        # found CAVs nearby
        self.cav_nearby = {}

        # used for cooperative perception.
        self._recieved_buffer = {}

        self.cav_world = weakref.ref(cav_world)()

        # ego position buffer. use deque so we can simulate lagging
        self.ego_pos = deque(maxlen=100)
        self.ego_spd = deque(maxlen=100)
        # used to exclude the cav self during searching
        self.vid = vid

        # check if lag or noise needed to be added during communication
        self.loc_noise = 0.0
        self.yaw_noise = 0.0
        self.speed_noise = 0.0
        self.lag = 0

        # Add noise to V2X communication if needed.
        if 'loc_noise' in config_yaml:
            self.loc_noise = config_yaml['loc_noise']
        if 'yaw_noise' in config_yaml:
            self.yaw_noise = config_yaml['yaw_noise']
        if 'speed_noise' in config_yaml:
            self.speed_noise = config_yaml['speed_noise']
        if 'lag' in config_yaml:
            self.lag = config_yaml['lag']

    def update_info(self, ego_pos, ego_spd):
        """
        Update all communication plugins with current localization info.
        """
        self.ego_pos.append(ego_pos)
        self.ego_spd.append(ego_spd)
        self.search()

    def get_ego_pos(self):
        """
        Add noise and lag to the current ego position and send to other CAVs.
        This is for simulate noise and lagging during communication.

        Returns
        -------
        processed_ego_pos : carla.Transform
            The ego position after adding noise and lagging.
        """
        if not self.ego_pos:
            return None

        # add lag
        ego_pos = self.ego_pos[0] if len(self.ego_pos) < self.lag else \
            self.ego_pos[np.random.randint(-1 - int(abs(self.lag)), 0)]

        x_noise = np.random.normal(0, self.loc_noise) + ego_pos.location.x
        y_noise = np.random.normal(0, self.loc_noise) + ego_pos.location.y
        z = ego_pos.location.z
        yaw_noise = np.random.normal(0, self.yaw_noise) + ego_pos.rotation.yaw

        noise_location = carla.Location(x=x_noise, y=y_noise, z=z)
        noise_rotation = carla.Rotation(pitch=0, yaw=yaw_noise, roll=0)

        processed_ego_pos = carla.Transform(noise_location, noise_rotation)

        return processed_ego_pos

    def get_ego_speed(self):
        """
        Add noise and lag to the current ego speed.

        Returns
        -------
        processed_ego_speed : float
            The ego speed after adding noise and lagging.
        """
        if not self.ego_spd:
            return None
        # add lag
        ego_speed = self.ego_spd[0] if len(self.ego_spd) < self.lag else \
            self.ego_spd[-1 - int(abs(self.lag))]
        processed_ego_speed = np.random.normal(0, self.speed_noise) + ego_speed

        return processed_ego_speed

    def search(self):
        """
        Search the CAVs nearby.
        """
        vehicle_manager_dict = self.cav_world.get_vehicle_managers()

        for vid, vm in vehicle_manager_dict.items():
            # avoid the Nonetype error at the first simulation step
            if not vm.v2x_manager.get_ego_pos():
                continue
            # avoid add itself as the cav nearby
            if vid == self.vid:
                continue
            distance = compute_distance(
                self.ego_pos[-1].location,
                vm.v2x_manager.get_ego_pos().location)

            if distance < self.communication_range:
                self.cav_nearby.update({vid: vm})
