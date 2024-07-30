"""
Additional utility functions for sensor_perception.py and perception_methods.py
"""

import random
import cv2
import weakref
import carla
import cv2
import numpy as np
import open3d as o3d
from EIdrive.scenario_testing import demo
from EIdrive.core.basic.auxiliary import \
    distance_angle_to_target
import EIdrive.core.sensing.perception.sensor_transformation as st
from EIdrive.core.sensing.perception.static_obstacle import TrafficLight
from EIdrive.core.sensing.perception.dynamic_obstacle import \
    DynamicObstacle


class CameraSensor:
    """
    Manage the camera for the vehicle.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle.

    world : carla.World
        The carla world object.

    global_position : list
        Global position of the infrastructure in form of [x, y, z]

    relative_position : str
        The position of the camera. Option: front, left, right.

    Attributes
    ----------
    image : np.ndarray
        Current rgb image.

    sensor : carla.sensor
        The carla sensor.

    """

    def __init__(self, vehicle, world, relative_position, global_position):
        if vehicle:
            world = vehicle.get_world()

        if vehicle is None:
            is_rsu = True
        else:
            is_rsu = False

        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('fov', '100')

        camera_spawn_point = self.calculate_camera_spawn_point(is_rsu, relative_position, global_position)

        self.sensor = world.spawn_actor(blueprint, camera_spawn_point, attach_to=vehicle) if vehicle else world.spawn_actor(
            blueprint, camera_spawn_point)

        self.image = None
        self.timstamp = None
        self.frame = 0
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CameraSensor.process_camera_data(weak_self, event))

        # camera attributes
        self.image_width = int(self.sensor.attributes['image_size_x'])
        self.image_height = int(self.sensor.attributes['image_size_y'])

    @staticmethod
    def calculate_camera_spawn_point(is_rsu, relative_position, global_position):
        """
        Calculate the spawn point for the camera based on the given relative and global positions. Adjustments are
        made according to the relative position specified (front, right, left, or default).

        Parameters
        ----------
        is_rsu : bool
            If it is for RSU.

        relative_position : str
            Relative position of the camera with respect to the vehicle.
            Accepted values are 'front', 'left', 'right', or any other value
            will default to the rearview.

        global_position : list or None
            A list representing the global position [x, y, z] of the infrastructure.
            If provided, the camera will be placed at this position with some adjustments
            based on relative_position. If None, a default location is used.

        Returns
        -------
        spawn_point : carla.Transform
            The spawn point for the camera.
        """
        location_adjustments = {
            'front': (2.5, 0, 1.0, 0),
            'right': (0.0, 0.3, 1.8, 100),
            'left': (0.0, -0.3, 1.8, -100),
            'default': (-2.0, 0, 1.5, 180)
        }

        if not is_rsu:
            if global_position:
                carla_location = carla.Location(*global_position)
                pitch = -35
            else:
                carla_location = carla.Location(0, 0, 0)
                pitch = 0

            x_adj, y_adj, z_adj, yaw = location_adjustments.get(relative_position, location_adjustments['default'])

            carla_location.x += x_adj
            carla_location.y += y_adj
            carla_location.z += z_adj
            carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)
        else:
            carla_location = carla.Location(*global_position[:3])
            carla_rotation = carla.Rotation(*global_position[-3:])

        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point

    @staticmethod
    def process_camera_data(weak_self, event):
        """camera  method"""
        self = weak_self()
        if not self:
            return
        image = np.array(event.raw_data)
        image = image.reshape((self.image_height, self.image_width, 4))
        # we need to remove the alpha channel
        image = image[:, :, :3]

        self.image = image
        self.frame = event.frame
        self.timestamp = event.timestamp


class LidarSensor:
    """
    Manage the Lidar sensor.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle.

    world : carla.World
        The carla world object.

    config_yaml : dict
        Configuration file for lidar.

    global_position : list
        Global position of the infrastructure in the form of [x, y, z]

    Attributes
    ----------
    o3d_pointcloud : 03d object
        Point cloud saved in o3d.Pointcloud.

    sensor : carla.sensor
        Lidar sensor.

    """

    def __init__(self, vehicle, world, config_yaml, global_position):
        if vehicle:
            world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        self.update_blueprint_attributes(blueprint, config_yaml)

        spawn_point = self.get_spawn_point(global_position)

        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle) if vehicle else world.spawn_actor(
            blueprint, spawn_point)

        # lidar data
        self.data = None
        self.timestamp = None
        self.frame = 0
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LidarSensor.process_lidar_data(weak_self, event))

    def update_blueprint_attributes(self, blueprint, config):
        attributes = ['upper_fov', 'lower_fov', 'channels', 'range',
                    'points_per_second', 'rotation_frequency',
                    'dropoff_general_rate', 'dropoff_intensity_limit',
                    'dropoff_zero_intensity', 'noise_stddev']

        for attr in attributes:
            if attr in config:
                blueprint.set_attribute(attr, str(config[attr]))

    def get_spawn_point(self, global_position):
        if global_position is None:
            return carla.Transform(carla.Location(x=-0.5, z=1.9))
        return carla.Transform(carla.Location(x=global_position[0],
                                            y=global_position[1],
                                            z=global_position[2]))

    @staticmethod
    def process_lidar_data(weak_self, event):
        """Lidar method"""
        self = weak_self()
        if not self:
            return

        # retrieve the raw lidar data and reshape to (N, 4)
        data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
        # (x, y, z, intensity)
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        self.data = data
        self.frame = event.frame
        self.timestamp = event.timestamp


class SemanticLidarSensor:
    """
    Manage semantic lidar sensor. It is used for data dumping.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle.

    world : carla.World
        The carla world object.

    config_yaml : dict
        Configuration file for lidar.

    global_position : list
        Global position of the infrastructure, [x, y, z]

    Attributes
    ----------
    o3d_pointcloud : 03d object
        Received point cloud, saved in o3d.Pointcloud format.

    sensor : carla.sensor
        Lidar sensor that will be attached to the vehicle.

    """

    def __init__(self, vehicle, world, config_yaml, global_position):
        if vehicle:
            world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        self.update_blueprint_attributes(blueprint, config_yaml)

        spawn_point = self.get_spawn_point(global_position)
        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle) if vehicle else world.spawn_actor(
            blueprint, spawn_point)

        # lidar data
        self.points = None
        self.obj_idx = None
        self.obj_tag = None
        self.timestamp = None
        self.frame = 0
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: SemanticLidarSensor.process_lidar_data(weak_self, event))

    def update_blueprint_attributes(self, blueprint, config):
        attributes = ['upper_fov', 'lower_fov', 'channels', 'range', 'points_per_second', 'rotation_frequency']
        for attr in attributes:
            if attr in config:
                blueprint.set_attribute(attr, str(config[attr]))

    def get_spawn_point(self, global_position):
        if global_position is None:
            return carla.Transform(carla.Location(x=-0.5, z=1.9))
        return carla.Transform(carla.Location(x=global_position[0], y=global_position[1], z=global_position[2]))

    @staticmethod
    def process_lidar_data(weak_self, event):
        """Semantic Lidar method"""
        self = weak_self()
        if not self:
            return

        data = np.frombuffer(event.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32),
            ('ObjTag', np.uint32)]))

        self.points = np.array([data['x'], data['y'], data['z']]).T
        self.obj_tag = np.array(data['ObjTag'])
        self.obj_idx = np.array(data['ObjIdx'])
        self.frame = event.frame
        self.timestamp = event.timestamp


def filter_and_update_vehicles(objects, carla_world, lidar, ego_pos, ego_id):
    """
    Retrieve and filter vehicles, then update the object dictionary.
    
    Parameters
    ----------
    objects : dict
        The object dictionary.
    carla_world : carla.World
        The carla world object.
    lidar : LidarSensor
        The lidar sensor object.
    ego_pos : carla.Transform
        The ego vehicle position.
    ego_id : int   
        The ego vehicle id.

    Returns
    -------
    objects : dict
        Updated object dictionary.
    """
    world = carla_world
    vehicle_list = world.get_actors().filter("*vehicle*")
    pedestrian_list = world.get_actors().filter("*walker*")
    thresh = 50
    vehicle_list = [v for v in vehicle_list if dist(v, ego_pos) < thresh and v.id != ego_id]
    pedestrian_list = [p for p in pedestrian_list if dist(p, ego_pos) < thresh]
    if lidar:
        vehicle_list = [DynamicObstacle(None, None, v, lidar.sensor) for v in
                        vehicle_list]
        pedestrian_list = [DynamicObstacle(None, None, p, None) for p in
                        pedestrian_list]
    else:
        vehicle_list = [DynamicObstacle(None, None, v, None) for v in vehicle_list]
        pedestrian_list = [DynamicObstacle(None, None, p, None) for p in pedestrian_list]

    objects.update({'vehicles': (vehicle_list+pedestrian_list)})
    return objects

def visualize_bbx_front_camera(objects, rgb_image, camera_index, rgb_camera):
    """
    Visualize the 3d bounding box on front camera image.

    Parameters
    ----------
    objects : dict
        The object dictionary.
    rgb_image : np.ndarray
        Received rgb image at current timestamp.
    camera_index : int
        Indicate the index of the current camera.
    rgb_camera : list
        List of camera objects.
    """

    # Extract camera details
    camera = rgb_camera[camera_index].sensor
    camera_transform = camera.get_transform()
    camera_location = camera_transform.location
    camera_yaw_rotation = camera_transform.rotation.yaw

    # Iterate through each vehicle to visualize the bounding box
    for vehicle in objects.get('vehicles', []):
        _, angle_to_vehicle = distance_angle_to_target(vehicle.get_location(), camera_location, camera_yaw_rotation)

        # Draw bounding box if vehicle is within camera field of view (FOV)
        if angle_to_vehicle < 60:
            bounding_box = st.get_2d_bbx(vehicle, camera, camera_transform)
            top_left = (int(bounding_box[0, 0]), int(bounding_box[0, 1]))
            bottom_right = (int(bounding_box[1, 0]), int(bounding_box[1, 1]))

            # Draw the rectangle on the RGB image
            cv2.rectangle(rgb_image, top_left, bottom_right, (255, 0, 0), thickness=2)

    return rgb_image

def visualize_yolo_bbx(yolo_result, original_image, detection_index):
    """
    Overlay bounding boxes detected by YOLO on the given image.

    Parameters
    ----------
    yolo_result : yolo.Result
        Detection outcomes from YoloV5.
    original_image : np.ndarray
        The original image to which detections will be overlaid.
    detection_index : int
        Specific index for detection.

    Returns
    ----------
    modified_image : np.ndarray
        Image with the overlaid bounding boxes.
    """

    bounding_data = yolo_result.xyxy[detection_index]
    bounding_data = bounding_data.cpu().detach().numpy() if bounding_data.is_cuda else bounding_data.detach().numpy()

    for entry in bounding_data:
        label = int(entry[5])
        label_name = yolo_result.names[label]

        if is_vehicle_in_cococlass(label):
            label_name = 'vehicle'

        x1, y1, x2, y2 = map(int, entry[:4])
        cv2.rectangle(original_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(original_image, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

    return original_image

def visualize_ssd_bbx(best_results_per_input, classes_to_labels, image):
    """
    Overlay bounding boxes detected by SSD on the given image.

    Parameters
    ----------
    best_results_per_input : list of tuple
        Contains detection results for each image input, where each tuple comprises bounding boxes, classes, and confidences.

    classes_to_labels : dict
        A mapping from class IDs to their corresponding string labels.

    image : np.ndarray
        The original image on which the bounding boxes will be overlaid.

    index : int
        Specific index for detection (currently unused in the function).

    Returns
    ----------
    imagechange : np.ndarray
        The input image with overlaid bounding boxes and associated labels.

    Notes
    -----
    The function assumes that the image dimensions are fixed at 800x600 pixels for calculating bounding box coordinates.
    """

    imagechange = image.copy()  # Create a copy of the input image

    for image_idx in range(len(best_results_per_input)):
        bboxes, classes, confidences = best_results_per_input[image_idx]
        for idx in range(len(bboxes)):
            left, bot, right, top = bboxes[idx]
            x, w = [int(val * 800) for val in [left, right - left]]
            y, h = [int(val * 600) for val in [bot, top - bot]]
            cv2.rectangle(imagechange, (x, y), (x + w, y + h), (0, 255, 0), 2)

            label = classes_to_labels[classes[idx] - 1]
            confidence = int(confidences[idx] * 100)
            text = f"{label} {confidence}%"
            cv2.putText(imagechange, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

    return imagechange

def get_traffic_lights(detected_objects, carla_world, ego_pos):
    """
    Get the nearby traffic lights directly from the server.

    Parameters
    ----------
    detected_objects : dict
        Dictionary containing all the detected objects.
    carla_world : carla.World
        The carla world object.
    ego_pos : carla.Transform
        The ego vehicle position.

    Returns
    -------
    detected_objects : dict
        Updated dictionary with traffic light information.
    """

    # Get list of all traffic lights from the CARLA world
    traffic_lights = carla_world.get_actors().filter('traffic.traffic_light*')
    stop_signs = carla_world.get_actors().filter('traffic.stop*')
    nearby_lights = []

    for light in traffic_lights:
        if dist(light, ego_pos) < 50:
            light_info = TrafficLight(light.get_location(), light.get_state())
            nearby_lights.append(light_info)

    for stop_sign in stop_signs:
        if dist(stop_sign, ego_pos) < 50:
            light_info = TrafficLight(stop_sign.get_location(), 'Red')
            nearby_lights.append(light_info)
    
    detected_objects['traffic_lights'] = nearby_lights

    return detected_objects

def camera_window_pos(name):
    """Position the visualization windows."""
    if demo.player_ids is None:
        pass
    elif len(demo.player_ids) != 6:
        pass
    elif id in demo.player_ids:
        if id == demo.player_ids[0]:
            cv2.moveWindow('%s camera of actor %d' % (name, id), 0, 0)
        if id == demo.player_ids[1]:
            cv2.moveWindow('%s camera of actor %d' % (name, id), 0, 750)
        if id == demo.player_ids[2]:
            cv2.moveWindow('%s camera of actor %d' % (name, id), 0, 1500)

def get_error(error_rate):
    """
    Returns True based on the given percentage chance.

    Parameters
    ----------
    percentage (float): 
        The probability (from 0 to 100) of returning True.

    Returns
    -------
    bool: 
        True if the random number is less than or equal to the percentage, False otherwise.
    """

    if not 0 <= error_rate <= 1:
        raise ValueError("Percentage must be between 0 and 1.")
    
    error_present =  random.random() <= error_rate

    return error_present

def is_vehicle_in_cococlass(label):
    """
    Determine if the provided label corresponds to a vehicle class in the COCO dataset.

    Parameters
    ----------
    label : int
        Predicted class label from YOLO detection.

    Returns
    -------
    bool
        True if the label corresponds to a vehicle class, otherwise False.
    """
    vehicle_classes = {1, 2, 3, 5, 7}
    return label in vehicle_classes

def dist(obstacle, ego_pos):
    """
    Get the distance from the obstacle to ego vehicle form server.

    Parameters
    ----------
    obstacle : carla.actor
        The obstacle vehicle.
    ego_pos : carla.Transform
        The ego vehicle position

    Returns
    -------
    distance : float
        The distance between ego and the target obstacle.
    """
    return obstacle.get_location().distance(ego_pos.location)