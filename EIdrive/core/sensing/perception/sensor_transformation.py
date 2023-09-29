"""
The transformations between world and sensors.
"""

import numpy as np
from matplotlib import cm

import carla

VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


def get_camera_intrinsic_matrix(sensor_obj):
    """
    Get the camera intrinsic matrix.

    Parameters
    ----------
    sensor_obj : carla.sensor
        A Carla RGB camera sensor object.

    Returns
    -------
    intrinsic_matrix : np.ndarray
        The 2D intrinsic matrix.
    """
    view_width = int(sensor_obj.attributes['image_size_x'])
    view_height = int(sensor_obj.attributes['image_size_y'])
    view_fov = int(float(sensor_obj.attributes['fov']))

    intrinsic_matrix = np.identity(3)
    intrinsic_matrix[0, 2] = view_width / 2.0
    intrinsic_matrix[1, 2] = view_height / 2.0
    intrinsic_matrix[0, 0] = intrinsic_matrix[1, 1] = view_width / \
                                                      (2.0 * np.tan(view_fov * np.pi / 360.0))

    return intrinsic_matrix


def generate_bb_points(vehicle_obj):
    """
    Generate the vertices of a 3D bounding box based on the vehicle.

    Parameters
    ----------
    vehicle_obj : EIdrive object
        An instance of Opencda's DynamicObstacle with relevant attributes.

    Returns
    -------
    np.ndarray
        A 3D bounding box represented by its vertices. The array has a shape of (8, 4).
    """
    bbx = np.zeros((8, 4))
    extent = vehicle_obj.bounding_box.extent

    bbx[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
    bbx[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
    bbx[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
    bbx[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
    bbx[4, :] = np.array([extent.x, extent.y, extent.z, 1])
    bbx[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
    bbx[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
    bbx[7, :] = np.array([extent.x, -extent.y, extent.z, 1])

    return bbx


def x_to_world_transformation(transform_obj):
    """
    Computes the transformation matrix to convert from vehicle or sensor coordinates to world coordinates

    Parameters
    ----------
    transform_obj : carla.Transform
        The transform containing location and rotation information.

    Returns
    -------
    transformation_matrix : np.ndarray
        The transform containing location and rotation information.
    """
    rotation = transform_obj.rotation
    location = transform_obj.location

    # Calculate rotation matrix components
    yaw_rad = np.radians(rotation.yaw)
    pitch_rad = np.radians(rotation.pitch)
    roll_rad = np.radians(rotation.roll)

    # Calculate trigonometric values
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)

    transformation_matrix = np.identity(4)
    # Translation matrix
    transformation_matrix[:3, 3] = location.x, location.y, location.z

    # Rotation matrix
    rotation_matrix = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                                [-sp, cp * sr, cp * cr]])

    transformation_matrix[:3, :3] = rotation_matrix

    return transformation_matrix


def bbx_to_world(bounding_box_coords, obstacle_vehicle):
    """
    Compute the bounding box coordinates from vehicle coordinates to world coordinates.

    Parameters
    ----------
    bounding_box_coords : np.ndarray
        Bounding box coordinates with 8 vertices in the shape of (8, 4).
    obstacle_vehicle : EIdrive object
        DynamicObstacle object.

    Returns
    -------
    bb_world_coords : np.ndarray
        Bounding box coordinates in world coordinates.
    """

    # Transformation from bounding box to vehicle
    bb_to_vehicle_transform = carla.Transform(obstacle_vehicle.bounding_box.location)

    # Transformation matrix from bounding box to vehicle
    bb_to_vehicle_matrix = x_to_world_transformation(bb_to_vehicle_transform)

    # Transformation matrix from vehicle to world
    vehicle_to_world_matrix = x_to_world_transformation(obstacle_vehicle.get_transform())

    # Combined transformation matrix from bounding box to world
    bb_to_world_matrix = np.dot(vehicle_to_world_matrix, bb_to_vehicle_matrix)

    # Convert the 8 vertices from bounding box center to world coordinates
    bb_world_coords = np.dot(bb_to_world_matrix, np.transpose(bounding_box_coords))

    return bb_world_coords


def world_to_sensor(world_coords, sensor_transform):
    """
    Convert transform coordinates from world coordinates to sensor coordinates.

    Parameters
    ----------
    world_coords : np.ndarray
        Coordinates in world coordinates in the shape of (4, n).

    sensor_transform : carla.Transform
        Sensor's transform in the world.

    Returns
    -------
    sensor_coords : np.ndarray
        Coordinates in the sensor coordinates.
    """
    # Transformation matrix from world to sensor space
    sensor_world_matrix = x_to_world_transformation(sensor_transform)

    # Inverse transformation matrix from sensor to world space
    sensor_to_world_matrix = np.linalg.inv(sensor_world_matrix)

    # Transform coordinates from world to sensor reference
    sensor_coords = np.dot(sensor_to_world_matrix, world_coords)

    return sensor_coords


def sensor_to_world(sensor_coords, sensor_transform):
    """
    Convert transform coordinates from sensor coordinates to world coordinates.

    Parameters
    ----------
    sensor_coords : np.ndarray
        Coordinates in the sensor's coordinates.

    sensor_transform : carla.Transform
        Sensor's transform in the world.

    Returns
    -------
    world_coords : np.ndarray
        Coordinates in world coordinates.
    """
    # Transformation matrix from sensor to world space
    sensor_world_matrix = x_to_world_transformation(sensor_transform)

    # Project sensor coordinates to world reference
    world_coords = np.dot(sensor_world_matrix, sensor_coords)

    return world_coords


def vehicle_to_sensor(cords, vehicle, sensor_transform):
    """
    Convert transform coordinates from vehicle coordinates to sensor coordinates.

    Parameters
    ----------
    cords : np.ndarray
         Coordinates in vehicle coordinates in the shape of (n, 4).

    vehicle : EIdrive object
        Carla DynamicObstacle.

    sensor_transform : carla.Transform
        Sensor transform in the world.

    Returns
    -------
    sensor_cord : np.ndarray
        Coordinates in the sensor reference in the shape of (4, n)

    """
    world_cord = bbx_to_world(cords, vehicle)
    sensor_cord = world_to_sensor(world_cord, sensor_transform)

    return sensor_cord


def get_2d_bounding_box(vehicle, camera, sensor_transform):
    """
    Get vehicle's bounding box and convert it to 2D sensor image.

    Parameters
    ----------
    vehicle : carla.Vehicle
        Ego vehicle.

    camera : carla.sensor
        Carla RGB camera on vehicle.

    sensor_transform : carla.Transform
        Sensor's transform in the world.

    Returns
    -------
    camera_bbx_2d : np.ndarray
        Bounding box coordinates in the 2D sensor image.
    """
    # Get the camera intrinsic matrix
    camera_intrinsic_matrix = get_camera_intrinsic_matrix(camera)

    # Calculate bounding box points relative to the vehicle's center
    bounding_box_coords_rel = generate_bb_points(vehicle)

    # Transform bounding box coordinates to the sensor's coordinate system
    bb_coords_sensor = vehicle_to_sensor(bounding_box_coords_rel, vehicle, sensor_transform)[:3, :]

    # Rearrange bounding box coordinates for projection
    coords_y_minus_z_x = np.concatenate([bb_coords_sensor[1, :].reshape(1, 8),
                                         -bb_coords_sensor[2, :].reshape(1, 8),
                                         bb_coords_sensor[0, :].reshape(1, 8)])

    # Project bounding box coordinates to 2D sensor image
    projected_coords = np.dot(camera_intrinsic_matrix, coords_y_minus_z_x)
    projected_coords /= projected_coords[2]

    camera_bbx_2d = np.transpose(projected_coords[:2, :])

    return camera_bbx_2d


def draw_2d_bbx_from_3d(p3d_bb):
    """
    Draw 2d bounding box(4 vertices) from 3d bounding box(8 vertices). 2D
    bounding box is represented by two corner points.

    Parameters
    ----------
    p3d_bb : np.ndarray
        The 3d bounding box is going to project to 2d.

    Returns
    -------
    bbx_2d : np.ndarray
        Projected 2d bounding box.

    """
    min_x = np.amin(p3d_bb[:, 0])
    min_y = np.amin(p3d_bb[:, 1])
    max_x = np.amax(p3d_bb[:, 0])
    max_y = np.amax(p3d_bb[:, 1])
    bbx_2d = np.array([[min_x, min_y], [max_x, max_y]])
    return bbx_2d


def get_2d_bbx(vehicle, sensor, senosr_transform):
    """
    Get 2D bounding box.

    Parameters
    ----------
    vehicle : carla.Vehicle
        Ego vehicle.

    sensor : carla.sensor
        Carla sensor.

    senosr_transform : carla.Transform
        Sensor transform.

    Returns
    -------
    p2d_bb : np.ndarray
        2D bounding box.

    """
    p3d_bb = get_2d_bounding_box(vehicle, sensor, senosr_transform)
    p2d_bb = draw_2d_bbx_from_3d(p3d_bb)
    return p2d_bb


def convert_lidar_to_camera(lidar, camera, point_cloud, rgb_image):
    """
    Convert lidar points to camera space.

    Parameters
    ----------
    lidar : carla.sensor
        Lidar sensor.

    camera : carla.sensor
        RGB camera.

    point_cloud : np.ndarray
        Point cloud containing lidar points in the shape of (n, 4).

    rgb_image : np.ndarray
        RGB image from camera.

    Returns
    -------
    rgb_image : np.ndarray
        New RGB image with projected lidar points.

    points_2d : np.ndarray
        Point cloud projected to camera space.

    """

    # Extract intensity array and 3D points from point cloud
    intensity = np.array(point_cloud[:, 3])
    local_lidar_points = np.array(point_cloud[:, :3]).T

    # Convert local lidar points to homogeneous coordinates
    local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

    # Transform lidar points from lidar space to world space
    lidar_2_world = x_to_world_transformation(lidar.get_transform())
    world_points = np.dot(lidar_2_world, local_lidar_points)

    # Project lidar world points to camera space
    sensor_points = world_to_sensor(world_points, camera.get_transform())

    # Convert to camera coordinates
    point_in_camera_coords = np.array([
        sensor_points[1],
        sensor_points[2] * -1,
        sensor_points[0]])

    # Get camera intrinsic matrix
    K = get_camera_intrinsic_matrix(camera)

    # Project 3D points in camera space to image space
    x_coord = np.dot(K[0, :], point_in_camera_coords)
    y_coord = np.dot(K[1, :], point_in_camera_coords)
    z_coord = np.dot(K[2, :], point_in_camera_coords)

    # Normalize x, y, z
    normalized_x = x_coord / z_coord
    normalized_y = y_coord / z_coord

    points_2d = np.array([normalized_x, normalized_y, z_coord])

    image_w = int(camera.attributes['image_size_x'])
    image_h = int(camera.attributes['image_size_y'])

    # Filter out points outside the camera's field of view
    points_2d = points_2d.T
    intensity = intensity.T
    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
        (points_2d[:, 2] > 0.0)
    new_points_2d = points_2d[points_in_canvas_mask]
    new_intensity = intensity[points_in_canvas_mask]

    # Calculate integer screen coordinates
    u_coord = new_points_2d[:, 0].astype(np.int)
    v_coord = new_points_2d[:, 1].astype(np.int)

    # Adjust intensity for visualization
    new_intensity = 4 * new_intensity - 3
    color_map = np.array([
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 0]) * 255.0,
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 1]) * 255.0,
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 2]) * 255.0]). \
        astype(np.int).T

    for i in range(len(new_points_2d)):
        rgb_image[v_coord[i] - 1: v_coord[i] + 1,
        u_coord[i] - 1: u_coord[i] + 1] = color_map[i]

    return rgb_image, points_2d
