"""
This module is used to check collision.
"""

import math
from math import sin, cos
from scipy import spatial

import carla
import numpy as np

from EIdrive.core.basic.auxiliary import distance_angle_to_target
from EIdrive.core.plan.cubic_spline import Spline2D


class CollisionDetector:
    """
    The default collision checker module.

    Parameters
    ----------
    time_ahead : float
        how many seconds we look ahead in advance for collision check.
    circle_radius : float
        The radius of the collision checking circle.
    circle_offsets : float
        The offset between collision checking circle and the trajectory point.
    """

    def __init__(self, time_ahead=1.2, circle_radius=1.0, circle_offsets=None):

        self.time_ahead = time_ahead
        self._circle_offsets = [-1.0,
                                0,
                                1.0] \
            if circle_offsets is None else circle_offsets
        self._circle_radius = circle_radius

    def check_adjacent_lane_collision(
            self, ego_loc, target_wpt, overtake):
        """
        Generates a straight path in the neighbouring lane for collision prediction during overtake or lane change
        scenarios.

        Parameters:
            ego_loc (carla.Location): The current location of the ego vehicle.
            target_wpt (carla.Waypoint): The farthest waypoint in the adjacent lane to be checked.
            overtake (bool): Flag indicating whether this is an overtaking scenario or a standard lane change.

        Returns:
            rx (list): The x coordinates of the collision detection line in the adjacent lane.
            ry (list): The y coordinates of the collision detection line in the adjacent lane.
            ryaw (list): The yaw angle of the collision detection line in the adjacent lane.
        """

        # Depending on the overtake situation, define the next target waypoint
        target_wpt_next = target_wpt.next(6)[0] if overtake else target_wpt

        # Calculate the distances from the ego location to the next target waypoint
        distance_x = target_wpt_next.transform.location.x - ego_loc.x
        distance_y = target_wpt_next.transform.location.y - ego_loc.y
        total_distance = np.hypot(distance_x, distance_y) + 3

        # Define the previous target waypoint based on the calculated total_distance
        target_wpt_prev = target_wpt.previous(total_distance)
        while len(target_wpt_prev) == 0:
            total_distance -= 2
            target_wpt_prev = target_wpt.previous(total_distance)

        target_wpt_prev = target_wpt_prev[0]
        target_wpt_mid = target_wpt_prev.next(total_distance / 2)[0]

        # Gather the x and y coordinates of the next, middle and previous waypoints
        waypoint_x_coords = [target_wpt_next.transform.location.x,
                             target_wpt_mid.transform.location.x,
                             target_wpt_prev.transform.location.x]
        waypoint_y_coords = [target_wpt_next.transform.location.y,
                             target_wpt_mid.transform.location.y,
                             target_wpt_prev.transform.location.y]

        increment_distance = 0.1

        # Create a 2D CubicSpline with the gathered x and y coordinates
        created_spline = Spline2D(waypoint_x_coords, waypoint_y_coords)
        incremental_s_values = np.arange(created_spline.arc_lengths[0], created_spline.arc_lengths[-1], increment_distance)

        debug_tmp = []

        # Calculate interpolation points and the corresponding yaw values
        rx, ry, ryaw = [], [], []
        for s_val in incremental_s_values:
            i_x, i_y = created_spline.get_position(s_val)
            rx.append(i_x)
            ry.append(i_y)
            ryaw.append(created_spline.get_yaw(s_val))
            debug_tmp.append(carla.Transform(carla.Location(i_x, i_y, 0)))

        return rx, ry, ryaw

    def circle_collision_check(
            self,
            route_x,
            route_y,
            route_yaw,
            potential_obstacle,
            velocity,
            carla_roadmap,
            adjacent_check=False):
        """
        Checks for potential collision hazards on the proposed route using
        circular collision checking mechanism.

        Parameters
            - adjacent_check (boolean): Indicates whether adjacent lane checking should be done.
              Note: The complete path should always be provided when performing an adjacent lane check.
            - velocity (float): The speed of the ego vehicle in meters per second.
            - route_yaw (float): A list of yaw angles.
            - route_x (list): A list of x coordinates.
            - route_y (list): A list of y coordinates.
            - potential_obstacle (carla.vehicle): The potential hazard vehicle on the path.

        Returns:
            - no_collision (boolean): Flag that indicates whether the current range is collision free.

        route_x
        """
        no_collision = True
        # Determine how far ahead to check, based on speed.
        # If the speed is very low, use a minimum threshold for the check distance
        check_distance = min(max(int(self.time_ahead * velocity / 0.1), 90),
                             len(route_x)) if not adjacent_check else len(route_x)

        potential_obstacle_location = potential_obstacle.get_location()
        potential_obstacle_yaw = carla_roadmap.get_waypoint(
            potential_obstacle_location).transform.rotation.yaw

        # The step is 0.1m, so we check every 10 points
        for index in range(0, check_distance, 10):
            point_x, point_y, yaw = route_x[index], route_y[index], route_yaw[index]

            circular_locations = np.zeros((len(self._circle_offsets), 2))
            circle_offsets_array = np.array(self._circle_offsets)
            circular_locations[:, 0] = point_x + circle_offsets_array * cos(yaw)
            circular_locations[:, 1] = point_y + circle_offsets_array * sin(yaw)

            # Calculate bounding box coordinates under world coordinate system
            adjusted_extent_x = potential_obstacle.bounding_box.extent.x * \
                                math.cos(math.radians(potential_obstacle_yaw))
            adjusted_extent_y = potential_obstacle.bounding_box.extent.y * \
                                math.sin(math.radians(potential_obstacle_yaw))

            # Compute the four corners of the bounding box
            potential_obstacle_bounding_box_array = \
                np.array([[potential_obstacle_location.x - adjusted_extent_x,
                           potential_obstacle_location.y - adjusted_extent_y],
                          [potential_obstacle_location.x - adjusted_extent_x,
                           potential_obstacle_location.y + adjusted_extent_y],
                          [potential_obstacle_location.x,
                           potential_obstacle_location.y],
                          [potential_obstacle_location.x + adjusted_extent_x,
                           potential_obstacle_location.y - adjusted_extent_y],
                          [potential_obstacle_location.x + adjusted_extent_x,
                           potential_obstacle_location.y + adjusted_extent_y]])

            # Compute the distances from the four corners of the vehicle to the trajectory point
            potential_collision_dists = spatial.distance.cdist(
                potential_obstacle_bounding_box_array, circular_locations)

            potential_collision_dists = np.subtract(potential_collision_dists, self._circle_radius)
            no_collision = no_collision and not np.any(potential_collision_dists < 0)

            if not no_collision:
                break

        return no_collision
