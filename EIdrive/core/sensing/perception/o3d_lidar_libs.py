"""
Utility functions for 3d lidar visualization and processing by utilizing open3d.
"""

import time

import open3d as o3d
import numpy as np

from matplotlib import cm
from scipy.stats import mode

import EIdrive.core.sensing.perception.sensor_transformation as st
from EIdrive.core.sensing.perception.obstacle_vehicle import \
    is_vehicle_in_cococlass, ObstacleVehicle
from EIdrive.core.sensing.perception.static_obstacle import StaticObstacle

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255),  # None
    (70, 70, 70),  # Building
    (100, 40, 40),  # Fences
    (55, 90, 80),  # Other
    (220, 20, 60),  # Pedestrian
    (153, 153, 153),  # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),  # Vehicle
    (102, 102, 156),  # Wall
    (220, 220, 0),  # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),  # Ground
    (150, 100, 100),  # Bridge
    (230, 150, 140),  # RailTrack
    (180, 165, 180),  # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160),  # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),  # Water
    (145, 170, 100),  # Terrain
]) / 255.0  # normalize each channel [0-1] since is what Open3D uses


def convert_raw_to_o3d_pointcloud(raw_data, point_cloud):
    """
    Encode the raw point cloud(np.array) to Open3d PointCloud object.

    Parameters
    ----------
    raw_data : np.ndarray
        Raw lidar points, (N, 4).

    point_cloud : o3d.PointCloud
        Open3d PointCloud.
    """

    # Extract intensity values and compute corresponding colors
    intensity = raw_data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Extract 3D data and adjust the y-coordinate for visualization
    xyz_data = np.array(raw_data[:, :-1], copy=True)
    xyz_data[:, 0] = -xyz_data[:, 0]  # Negate x-coordinate for correct visualization

    # Assign the processed data to the point cloud object
    point_cloud.points = o3d.utility.Vector3dVector(xyz_data)
    point_cloud.colors = o3d.utility.Vector3dVector(int_color)


def o3d_visualizer_init(actor_id):
    """
    Initialize the Open3D visualizer.

    Parameters
    ----------
    actor_id : int
        Ego vehicle's id.

    Returns
    -------
    visualizer : o3d.visualization.Visualizer
        Initialized Open3D visualizer.

    """
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=str(actor_id),
                             width=900,
                             height=600,
                             left=2380,
                             top=1470)
    render_option = visualizer.get_render_option()
    render_option.background_color = [0.05, 0.05, 0.05]
    render_option.point_size = 1
    render_option.show_coordinate_frame = True

    return visualizer


def visualize_point_cloud(visualizer, current_step, pcl, entity_data):
    """
    Visualize the point cloud at runtime.

    Parameters
    ----------
    visualizer : o3d.Visualizer
        Visualization interface.

    current_step : int
        Current step since simulation started.

    pcl : o3d.PointCloud
        Open3d point cloud.

    entity_data : dict
        The dictionary containing entities (e.g., vehicles).
    """

    if current_step == 2:
        visualizer.add_geometry(pcl)

    visualizer.update_geometry(pcl)

    def handle_entities(data):
        for key in data:
            if key != 'vehicles':
                continue
            for entity in data[key]:
                bounding_box = entity.o3d_bbx
                visualizer.add_geometry(bounding_box)

    handle_entities(entity_data)
    visualizer.poll_events()
    visualizer.update_renderer()
    time.sleep(0.001)

    def remove_geometries(data):
        for key in data:
            if key != 'vehicles':
                continue
            for entity in data[key]:
                bounding_box = entity.o3d_bbx
                visualizer.remove_geometry(bounding_box)

    remove_geometries(entity_data)


def camera_lidar_fusion_SSD(objects,
                            bounding_boxes,
                            raw_lidar_data,
                            lidar_in_camera_space,
                            lidar_sensor):
    """
    Utilize the 3D lidar points to extend the 2D bounding box from camera to 3D bounding box under world coordinates
    by SSD.

    Parameters
    ----------
    objects : dict
        The dictionary contains all object detection results.

    bounding_boxes : torch.Tensor
        Object detection bounding box at current photo from yolov5,
        shape (n, 5)->(n, [x1, y1, x2, y2, label])

    raw_lidar_data : np.ndarray
        Raw 3D lidar points in lidar coordinate system.

    lidar_in_camera_space : np.ndarray
        3D lidar points projected to the camera space.

    lidar_sensor : carla.sensor
        The lidar sensor.

    Returns
    -------
    detection_results : dict
        The updated detection dictionary that contains 3d bounding boxes.
    """

    bounding_boxes = np.array(bounding_boxes)
    for i in range(bounding_boxes.shape[0]):
        single_detection = bounding_boxes[i]
        x1, y1, x2, y2 = map(int, single_detection[:4])
        object_label = int(single_detection[4])

        points_within_box = \
            (lidar_in_camera_space[:, 0] > x1) & (lidar_in_camera_space[:, 0] < x2) & \
            (lidar_in_camera_space[:, 1] > y1) & (lidar_in_camera_space[:, 1] < y2) & \
            (lidar_in_camera_space[:, 2] > 0.0)
        filtered_points = raw_lidar_data[points_within_box][:, :-1]

        if filtered_points.shape[0] == 0:
            continue

        x_mode = mode(np.array(np.abs(filtered_points[:, 0]),
                               dtype=np.int), axis=0)[0][0]
        y_mode = mode(np.array(np.abs(filtered_points[:, 1]),
                               dtype=np.int), axis=0)[0][0]
        inlier_points_condition = (np.abs(filtered_points[:, 0]) > x_mode - 3) & \
                                  (np.abs(filtered_points[:, 0]) < x_mode + 3) & \
                                  (np.abs(filtered_points[:, 1]) > y_mode - 3) & \
                                  (np.abs(filtered_points[:, 1]) < y_mode + 3)
        inlier_points = filtered_points[inlier_points_condition]

        if inlier_points.shape[0] < 2:
            continue

        inlier_points[:, :1] = -inlier_points[:, :1]
        o3d_pcl = o3d.geometry.PointCloud()
        o3d_pcl.points = o3d.utility.Vector3dVector(inlier_points)
        bounding_box = o3d_pcl.get_axis_aligned_bounding_box()
        bounding_box.color = (0, 1, 0)

        box_corners = np.asarray(bounding_box.get_box_points())
        box_corners[:, :1] = -box_corners[:, :1]
        box_corners = box_corners.transpose()
        box_corners = np.r_[box_corners, [np.ones(box_corners.shape[1])]]
        world_corners = st.sensor_to_world(box_corners, lidar_sensor.get_transform())
        world_corners = world_corners.transpose()[:, :3]

        if is_vehicle_in_cococlass(object_label):
            vehicle_obstacle = ObstacleVehicle(world_corners, bounding_box)
            if 'vehicles' in objects:
                objects['vehicles'].append(vehicle_obstacle)
            else:
                objects['vehicles'] = [vehicle_obstacle]
        else:
            non_moving_obstacle = StaticObstacle(world_corners, bounding_box)
            if 'static' in objects:
                objects['static'].append(non_moving_obstacle)
            else:
                objects['static'] = [non_moving_obstacle]

    return objects


def camera_lidar_fusion_yolo(objects,
                             yolo_bbx,
                             raw_lidar_data,
                             lidar_in_camera_space,
                             lidar_sensor):
    """
    Convert 2D bounding boxes from camera images to 3D bounding boxes in world coordinates using 3D LIDAR points
    by yolov5.

    Parameters
    ----------
    objects : dict
        Dictionary containing all object detection results.

    yolo_bbx : torch.Tensor
        Bounding boxes detected in the current image using YOLOv5.
        Shape (n, 5) where n is the number of detected objects and
        each object is represented as [x1, y1, x2, y2, label].

    raw_lidar_data : np.ndarray
        Raw 3D LIDAR points in the LIDAR's coordinate system.

    lidar_in_camera_space : np.ndarray
        3D LIDAR points that have been projected onto the camera's coordinate space.

    lidar_sensor : carla.sensor
        The LIDAR sensor.

    Returns
    -------
    objects : dict
        Updated dictionary containing the original object detection results
        along with their corresponding 3D bounding boxes.
    """

    # Convert torch tensor to numpy array first
    if yolo_bbx.is_cuda:
        yolo_bbx = yolo_bbx.cpu().detach().numpy()
    else:
        yolo_bbx = yolo_bbx.detach().numpy()

    i = 0
    while i < yolo_bbx.shape[0]:
        detection = yolo_bbx[i]
        # 2D bbx coordinates
        x1, y1, x2, y2 = int(detection[0]), int(detection[1]), \
            int(detection[2]), int(detection[3])
        label = int(detection[5])

        # Choose the lidar points in the 2D yolo bounding box
        points_in_bbx = \
            (lidar_in_camera_space[:, 0] > x1) & (lidar_in_camera_space[:, 0] < x2) & \
            (lidar_in_camera_space[:, 1] > y1) & (lidar_in_camera_space[:, 1] < y2) & \
            (lidar_in_camera_space[:, 2] > 0.0)
        # Ignore intensity channel
        select_points = raw_lidar_data[points_in_bbx][:, :-1]

        if select_points.shape[0] == 0:
            i += 1
            continue

        # Filter out the outliers
        x_common = mode(np.array(np.abs(select_points[:, 0]),
                                 dtype=np.int), axis=0)[0][0]
        y_common = mode(np.array(np.abs(select_points[:, 1]),
                                 dtype=np.int), axis=0)[0][0]
        points_inlier = (np.abs(select_points[:, 0]) > x_common - 3) & \
                        (np.abs(select_points[:, 0]) < x_common + 3) & \
                        (np.abs(select_points[:, 1]) > y_common - 3) & \
                        (np.abs(select_points[:, 1]) < y_common + 3)
        select_points = select_points[points_inlier]

        if select_points.shape[0] < 2:
            i += 1
            continue

        # To visualize 3D lidar points in o3d visualizer, we need to revert the x coordinates
        select_points[:, :1] = -select_points[:, :1]

        # Create o3d.PointCloud object
        o3d_pointcloud = o3d.geometry.PointCloud()
        o3d_pointcloud.points = o3d.utility.Vector3dVector(select_points)
        # Add o3d bounding box
        aabb = o3d_pointcloud.get_axis_aligned_bounding_box()
        aabb.color = (0, 1, 0)

        # Get the eight corners of the bounding boxes.
        corner = np.asarray(aabb.get_box_points())
        # Convert back to unreal coordinate
        corner[:, :1] = -corner[:, :1]
        corner = corner.transpose()
        # Extend (3, 8) to (4, 8) for homogeneous transformation
        corner = np.r_[corner, [np.ones(corner.shape[1])]]
        # Project to world reference
        corner = st.sensor_to_world(corner, lidar_sensor.get_transform())
        corner = corner.transpose()[:, :3]

        if is_vehicle_in_cococlass(label):
            obstacle_vehicle = ObstacleVehicle(corner, aabb)
            if 'vehicles' in objects:
                objects['vehicles'].append(obstacle_vehicle)
            else:
                objects['vehicles'] = [obstacle_vehicle]
        # We regard other obstacles rather than vehicles as static class
        else:
            static_obstacle = StaticObstacle(corner, aabb)
            if 'static' in objects:
                objects['static'].append(static_obstacle)
            else:
                objects['static'] = [static_obstacle]

        i += 1

    return objects
