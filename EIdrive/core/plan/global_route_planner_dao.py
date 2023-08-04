# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides implementation for GlobalRoutePlannerDAO
"""

import numpy as np


class GlobalRoutePlannerDAO(object):
    """
    Serves as the data access layer, responsible for fetching data from the Carla server instance utilized by
    GlobalRoutePlanner. DAO is an abbreviation for Data Access Object.

    Parameters:
        -world_map : carla.world
            Represents the current Carla simulation world.
        -sampling_resolution : float
            Specifies the distance interval between sampled waypoints.
    """

    def __init__(self, world_map, sampling_resolution):

        self._sampling_resolution = sampling_resolution
        self._world_map = world_map

    def get_topology(self):
        """
        Accessor for topology. This function first gets the topology from the server, which is a list of road
        segments represented by pairs of waypoint objects. Then, it processes this list into a collection of
        dictionaries, each dictionary representing a road segment with the following key-value pairs

        :return topology_data:
            'entry'    - The waypoint at the starting point of the road segment.
            'entryxyz' - The (x,y,z) coordinates of the starting point of the road segment.
            'exit'     - The waypoint at the end point of the road segment.
            'exitxyz'  - The (x,y,z) coordinates of the end point of the road segment.
            'path'     - A list of waypoints that are 1m apart, representing the path from the entry to the exit point.
        """
        topology_data = []

        for segment in self._world_map.get_topology():
            # Unpack segment waypoints
            entry_waypoint, exit_waypoint = segment

            # Retrieve and round off waypoint locations
            entry_loc = np.round([entry_waypoint.transform.location.x,
                                  entry_waypoint.transform.location.y,
                                  entry_waypoint.transform.location.z], 0)
            exit_loc = np.round([exit_waypoint.transform.location.x,
                                 exit_waypoint.transform.location.y,
                                 exit_waypoint.transform.location.z], 0)

            # Initialize a dictionary for segment data
            segment_info = {
                'entry': entry_waypoint,
                'exit': exit_waypoint,
                'entryxyz': tuple(entry_loc),
                'exitxyz': tuple(exit_loc),
                'path': []
            }

            # If the distance between entry and exit points is larger than sampling resolution
            if entry_waypoint.transform.location.distance(exit_waypoint.transform.location) > self._sampling_resolution:
                waypoint = entry_waypoint.next(self._sampling_resolution)[0]

                # Collect waypoints until we reach the exit point
                while waypoint.transform.location.distance(
                        exit_waypoint.transform.location) > self._sampling_resolution:
                    segment_info['path'].append(waypoint)
                    waypoint = waypoint.next(self._sampling_resolution)[0]
            else:
                segment_info['path'].append(entry_waypoint.next(self._sampling_resolution)[0])

            # Append this segment's information to the topology data
            topology_data.append(segment_info)

        return topology_data

    def get_waypoint(self, location):
        """
        The function returns waypoint at the specific location.

        Args:
            -location (carla.location): Location of vehicle.
        Returns:
            -waypoint (carla.waypoint): Newly generated waypoint close
            to location.
        """
        waypoint = self._world_map.get_waypoint(location)
        return waypoint

    def get_resolution(self):
        """
        Return the sampling resolution.
        """
        return self._sampling_resolution
