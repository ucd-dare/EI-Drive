"""
This file includes all script for spectator monitoring, describing the motion of spectator.
"""

import carla
import numpy as np


class SpectatorController(object):
    """
    The controller that controls the moving and position of the spectator.

    Attributes
    ----------
    spectator : world object
        Spectator provides the video.

    """

    def __init__(self, spectator=None):
        """
        Initialize a spectator instance
        """
        self.spectator = spectator

    def bird_view_following(self, ego_vehicle_transform, altitude=80, bird_pitch=-90):
        """
        Set the spectator to follow a vehicle and provide bird view.

        Parameters
        ----------
        ego_vehicle_transform : transform
            The transform class contain all the position information of vehicle.

        """
        bird_view_ego_transform = carla.Transform()
        bird_view_ego_transform.location = ego_vehicle_transform.location
        bird_view_ego_transform.location.z = bird_view_ego_transform.location.z + altitude
        bird_view_ego_transform.rotation.pitch = bird_pitch
        self.spectator.set_transform(bird_view_ego_transform)

    def cal(self, x1, x2, t):
        return (x1 - x2) / t * 1.0

    def spectator_moving(self, ego_vehicle_transform, destination, remain_shift_ticks):
        """
        Shift the spectator from a position to another.

        Parameters
        ----------
        ego_vehicle_transform : transform
            The current position of vehicle.

        destination : transform
            The destination of shift.

        remain_shift_ticks : int
            Ticks left for the shift.

        """
        d = (destination.location.x - ego_vehicle_transform.location.x) / remain_shift_ticks
        ego_vehicle_transform.location.x = ego_vehicle_transform.location.x + d
        d = (destination.location.y - ego_vehicle_transform.location.y) / remain_shift_ticks
        ego_vehicle_transform.location.y = ego_vehicle_transform.location.y + d
        d = (destination.location.z - ego_vehicle_transform.location.z) / remain_shift_ticks
        ego_vehicle_transform.location.z = ego_vehicle_transform.location.z + d
        d = (destination.rotation.pitch - ego_vehicle_transform.rotation.pitch) / remain_shift_ticks
        ego_vehicle_transform.rotation.pitch = ego_vehicle_transform.rotation.pitch + d
        d = (destination.rotation.yaw - ego_vehicle_transform.rotation.yaw) / remain_shift_ticks
        ego_vehicle_transform.rotation.yaw = ego_vehicle_transform.rotation.yaw + d
        d = (destination.rotation.roll - ego_vehicle_transform.rotation.roll) / remain_shift_ticks
        ego_vehicle_transform.rotation.roll = ego_vehicle_transform.rotation.roll + d
        self.spectator.set_transform(ego_vehicle_transform)

    def full_bird_view(self, vehicle_list):
        """
        Adjust the height of spectator to include all vehicles in bird view.

        Parameters
        ----------
        vehicle_list : list
            The list of all vehicles.

        """
        vehicle_locations = np.array(
            [(vehicle.vehicle.get_transform().location.x, vehicle.vehicle.get_transform().location.y)
             for vehicle in vehicle_list])

        x = vehicle_locations[:, 0]
        y = vehicle_locations[:, 1]

        max_dx = max(x) - min(x)
        max_dy = max(y) - min(y)
        length = max(max_dx, max_dy)
        transform = carla.Transform(
            carla.Location(x=(max(x) + min(x)) / 2, y=(max(y) + min(y)) / 2, z=50 + length * 0.2),
            carla.Rotation(pitch=-90))
        self.spectator.set_transform(transform)
