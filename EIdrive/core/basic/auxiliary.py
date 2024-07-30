"""
Auxiliary functions.
"""

import math
import numpy as np
import carla

def visualize_trajectory(world, waypoints,
                        color=carla.Color(255, 0, 0),
                        life_time=5, size=0.1):
    """
    Visualize a trajectory using a list of waypoints.

    Parameters
    ----------
    world : carla.world
        The carla world.

    waypoints : list
        List of trajectory to visualize.
        
    color : carla.Color
        The trajectory color. Default is red.
        
    life_time : int
        Number of waypoints being visualized.
    
    size : float
        Size of the visualized waypoint. Default is 0.1.

    """
    for wpt in waypoints:
        # Check the type of the waypoint and obtain its transform
        if isinstance(wpt, (tuple, list)):
            wpt = wpt[0]
        wpt_transform = wpt.transform if hasattr(wpt, 'is_junction') else wpt

        # Draw the waypoint in the world
        world.debug.draw_point(
            wpt_transform.location,
            size=size,
            color=color,
            life_time=life_time)

def calculate_vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2.

    Parameters
    ----------
    location_1 : carla.location
        Start location of the vector.

    location_2 : carla.location
        End location of the vector.
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]

def distance_angle_to_target(target_location, current_location, orientation):
    """
    Compute the relative distance and angle from the current location
    to a target location based on a given orientation.

    Parameters
    ----------
    target_location : carla.Location
        The target location.

    current_location : carla.Location
        The current location .

    orientation : carla.Rotation
        Orientation of the reference object.

    Returns
    -------
    distance : float
        The measured distance from current location to target location.

    angle : float)
        Angle (in degrees) between the agent's forward direction
        and the direction towards the target.
    """
    delta_vector = np.array([target_location.x - current_location.x,
                             target_location.y - current_location.y])
    distance = np.linalg.norm(delta_vector) + 1e-10

    forward_vector = np.array([math.cos(math.radians(orientation)),
                               math.sin(math.radians(orientation))])
    angle = math.degrees(
        math.acos(
            np.clip(
                np.dot(
                    forward_vector, delta_vector) / distance, -1., 1.)))

    return distance, angle

def distance_to_vehicle(waypoint, vehicle_transform):
    """
    Get the distance from a point to a vehicle

    Parameters
    ----------
    waypoint : carla.Waypoint
        The waypoint.

    vehicle_transform : carla.transform
        Vehicle transform.

    Returns
    -------
    distance : float
        The distance from a point to a vehicle.
    """
    location = vehicle_transform.location
    if hasattr(waypoint, 'is_junction'):
        x = waypoint.transform.location.x - location.x
        y = waypoint.transform.location.y - location.y
    else:
        x = waypoint.location.x - location.x
        y = waypoint.location.y - location.y

    distance = math.sqrt(x**2 + y**2)

    return distance


def calculate_distance(location_1, location_2):
    """
    Calculate the distance between two location.

    Parameters
    ----------
    location_1 : carla.Location
        Start point.

    location_2 : carla.Location
        End point.
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    norm = np.linalg.norm([x, y]) + np.finfo(float).eps
    return norm


def get_speed(vehicle, by_meters=False):
    """
    Get the speed of a vehicle in m/s or km/h.

    Parameters
    ----------
    by_meters : bool
        Choose the unit in m/s (True) or km/h (False).

    vehicle : carla.vehicle
        The vehicle.

    Returns
    -------
    speed : float
        The vehicle speed.
    """
    v = vehicle.get_velocity()
    v_by_meters = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
    return v_by_meters if by_meters else 3.6 * v_by_meters


def positive(num):
    """
    Return the given number if positive, else return 0
    """
    return num if num > 0.0 else 0.0
