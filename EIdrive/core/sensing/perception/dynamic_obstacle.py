"""
Class for object detection and save the obstacle.
"""

import sys

import carla
import numpy as np
import open3d as o3d

import EIdrive.core.sensing.perception.sensor_transformation as st


def is_vehicle_in_cococlass(label):
    """
    Determines if the given label corresponds to a vehicle class in the COCO dataset.

    Parameters
    ----------
    label : int
        The label of the detected object.

    Returns
    -------
    bool
        True if the label belongs to the vehicle class, otherwise False.
    """
    vehicle_class_array = np.array([1, 2, 3, 5, 7], dtype=np.int)
    return True if 0 in (label - vehicle_class_array) else False


class BoundingBox(object):
    """
    Represents a bounding box for an obstacle vehicle.

    Parameters
    ----------
    corners : np.ndarray
        Eight corners of the bounding box. Shape: (8, 3).

    Attributes
    ----------
    location : carla.Location
        The location of the object.
    extent : carla.Vector3D
        The dimensions of the object in three-dimensional space.
    """

    def __init__(self, corners):
        center_x = np.mean(corners[:, 0])
        center_y = np.mean(corners[:, 1])
        center_z = np.mean(corners[:, 2])

        extent_x = (np.max(corners[:, 0]) - np.min(corners[:, 0])) / 2
        extent_y = (np.max(corners[:, 1]) - np.min(corners[:, 1])) / 2
        extent_z = (np.max(corners[:, 2]) - np.min(corners[:, 2])) / 2

        self.location = carla.Location(x=center_x, y=center_y, z=center_z)
        self.extent = carla.Vector3D(x=extent_x, y=extent_y, z=extent_z)


class DynamicObstacle(object):
    """
    Represents an obstacle vehicle, aligning attributes with the carla.Vehicle class.

    Parameters
    ----------
    corners : np.ndarray
        Eight corners of the bounding box. Shape: (8, 3).
    o3d_bbx : open3d.AlignedBoundingBox
        Bounding box representation in Open3D, mainly for visualization purposes.
    vehicle : carla.Vehicle
        Associated carla.Vehicle instance.
    lidar : carla.sensor.Lidar
        The LiDAR sensor associated with the vehicle.
    sumo2carla_ids : dict
        Mapping dictionary for Sumo to Carla. Essential for co-simulation as it aids
        in reading vehicle speeds from the Sumo API (traci) when they can't be fetched
        from the Carla server.

    Attributes
    ----------
    bounding_box : BoundingBox
        Bounding box of the obstacle vehicle.
    location : carla.Location
        Location of the obstacle vehicle.
    velocity : carla.Vector3D
        Velocity of the obstacle vehicle.
    carla_id : int
        ID of the obstacle vehicle, aligned with the carla.Vehicle's ID. Defaults to -1
        if no matching Carla vehicle is found.
    """

    def __init__(self, corners, o3d_bbx, vehicle=None, lidar=None, sumo2carla_ids=None):

        if not vehicle:
            self.bounding_box = BoundingBox(corners)
            self.location = self.bounding_box.location
            self.transform = None
            self.o3d_bbx = o3d_bbx
            self.carla_id = -1
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
        else:
            if sumo2carla_ids is None:
                sumo2carla_ids = dict()
            self.set_vehicle(vehicle, lidar)

    def get_transform(self):
        """
        Get the transform of the object vehicle.
        """
        return self.transform

    def get_location(self):
        """
        Get the location of the object vehicle.
        """
        return self.location

    def get_velocity(self):
        """
        Get the velocity of the object vehicle.
        """
        return self.velocity

    def set_carla_id(self, id):
        """
        Set carla id according to the carla.vehicle.

        Parameters
        ----------
        id : int
            The id from the carla.vehicle.
        """
        self.carla_id = id

    def set_velocity(self, velocity):
        """
        Set the velocity of the vehicle.

        Parameters
        ----------
        velocity : carla.Vector3D
            The target velocity in 3d vector format.
        """
        self.velocity = velocity

    def set_vehicle(self, vehicle_obj, lidar_sensor):
        """
        Assign attributes from carla.Vehicle to DynamicObstacle.

        Parameters
        ----------
        vehicle_obj : carla.Vehicle
            The carla.Vehicle object.

        lidar_sensor : carla.sensor.lidar
            The lidar sensor, used to project world coordinates to sensor coordinates.

        sumo_carla_mapping : dict
            Mapping dictionary between Sumo and Carla IDs, used during co-simulation.
            Necessary since speed info of vehicles controlled by Sumo cannot be read from the Carla server.
        """
        self.location = vehicle_obj.get_location()
        self.transform = vehicle_obj.get_transform()
        self.bounding_box = vehicle_obj.bounding_box
        self.carla_id = vehicle_obj.id
        self.type_id = vehicle_obj.type_id
        self.color = vehicle_obj.attributes["color"] \
            if hasattr(vehicle_obj, "attributes") and "color" in vehicle_obj.attributes else None
        self.set_velocity(vehicle_obj.get_velocity())

        # Calculate min and max boundary
        min_boundary = np.array([self.location.x - self.bounding_box.extent.x,
                                 self.location.y - self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z
                                 - self.bounding_box.extent.z,
                                 1])
        max_boundary = np.array([self.location.x + self.bounding_box.extent.x,
                                 self.location.y + self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z
                                 + self.bounding_box.extent.z,
                                 1])
        min_boundary = min_boundary.reshape((4, 1))
        max_boundary = max_boundary.reshape((4, 1))
        stack_boundary = np.hstack((min_boundary, max_boundary))

        if lidar_sensor is None:
            return
        # Convert the boundary coordinates to the lidar sensor space
        stack_boundary_sensor_coords = st.world_to_sensor(stack_boundary, lidar_sensor.get_transform())
        # Convert unreal space to o3d space
        stack_boundary_sensor_coords[:1, :] = -stack_boundary_sensor_coords[:1, :]
        # (4, 2) -> (3, 2)
        stack_boundary_sensor_coords = stack_boundary_sensor_coords[:-1, :]

        min_boundary_sensor = np.min(stack_boundary_sensor_coords, axis=1)
        max_boundary_sensor = np.max(stack_boundary_sensor_coords, axis=1)

        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_boundary_sensor,
                                                   max_bound=max_boundary_sensor)
        aabb.color = (1, 0, 0)
        self.o3d_bbx = aabb
