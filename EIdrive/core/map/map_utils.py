"""
Minimap convert utilities.
"""

import carla
import numpy as np
from enum import IntEnum


def lateral_shift(transform, shift):
    transform.rotation.yaw += 90
    return transform.location + shift * transform.get_forward_vector()


def convert_locations_to_array(locations):
    """
    Transform a list of Carla locations into a numpy array.

    Parameters
    ----------
    locations : list
        Collection of Carla locations.

    Returns
    -------
    coordinates : np.array
        A numpy array with shape (N, 3) representing the X, Y, Z coordinates.
    """
    coordinates = np.zeros((len(locations), 3))
    for idx, loc in enumerate(locations):
        coordinates[idx, 0] = loc.x
        coordinates[idx, 1] = loc.y
        coordinates[idx, 2] = loc.z

    return coordinates


def waypoints_to_array(waypoints):
    """
    Convert a list of Carla waypoints to a numpy array.

    Parameters
    ----------
    waypoints : list
        Collection of Carla waypoints.

    Returns
    -------
    coordinates : np.array
        A numpy array with shape (N, 3) representing the X, Y, Z coordinates of the waypoints.
    """
    coordinates = np.zeros((len(waypoints), 3))
    for idx, waypoint in enumerate(waypoints):
        coordinates[idx, 0] = waypoint.transform.location.x
        coordinates[idx, 1] = waypoint.transform.location.y
        coordinates[idx, 2] = waypoint.transform.location.z

    return coordinates


def traffic_light_state_to_string(traffic_light_state):
    """
    Transform a carla.TrafficLightState into its string representation.

    Parameters
    ----------
    traffic_light_state : carla.TrafficLightState
        The state of a traffic light in the Carla environment.

    Returns
    -------
    state_representation : str
        String representation of the traffic light's state.
    """
    if traffic_light_state == carla.TrafficLightState.Red:
        return 'red'
    elif traffic_light_state == carla.TrafficLightState.Green:
        return 'green'
    elif traffic_light_state == carla.TrafficLightState.Yellow:
        return 'yellow'
    else:
        return 'normal'
