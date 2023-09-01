"""
Class for static obstacle.
"""

import sys

import numpy as np
import carla


class BoundingBox:
    """
    Class representing a bounding box for obstacles or vehicles.

    Parameters
    ----------
    corners : np.ndarray
        The eight corners of the bounding box in a 3D space.
        Shape: (8, 3).

    Attributes
    ----------
    loc : carla.Location
        The central location of the object within the bounding box.

    size : carla.Vector3D
        Dimensions of the object, indicating its width, height, and depth.
    """

    def __init__(self, corners):
        self.loc = self.compute_location(corners)
        self.size = self.compute_extent(corners)

    @staticmethod
    def compute_location(corners):
        avg_x, avg_y, avg_z = np.mean(corners, axis=0)
        return carla.Location(x=avg_x, y=avg_y, z=avg_z)

    @staticmethod
    def compute_extent(corners):
        half_x = (np.max(corners[:, 0]) - np.min(corners[:, 0])) / 2
        half_y = (np.max(corners[:, 1]) - np.min(corners[:, 1])) / 2
        half_z = (np.max(corners[:, 2]) - np.min(corners[:, 2])) / 2
        return carla.Vector3D(x=half_x, y=half_y, z=half_z)


class StaticObstacle:
    """
    Representation for a static obstacle. Stop signs and traffic lights are treated uniformly under this class.

    Parameters
    ----------
    bounding_corners : np.ndarray
        The eight corners of the obstacle's bounding box in a 3D space.
        Shape: (8, 3).

    o3d_box : open3d.AlignedBoundingBox
        Open3D's bounding box representation primarily utilized for visualization purposes.

    Attributes
    ----------
    bbx : BoundingBox
        The bounding box encapsulating the static obstacle.
    """

    def __init__(self, bounding_corners, o3d_box):
        self.bbx = BoundingBox(bounding_corners)
        self.o3d_box = o3d_box


class TrafficLight:
    """
    Traffic light information is directly retrieved from the server and mapped to this class.

    Parameters
    ----------
    position : carla.Location
        Location of the traffic light in the simulation environment.

    state : carla.TrafficLightState
        Current operational state of the traffic light (e.g., red, green, yellow).
    """

    def __init__(self, position, state):
        self.position = position
        self.state = state

    def get_location(self):
        return self.position

    def get_state(self):
        return self.state
