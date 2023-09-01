"""
Perception module with all sensors.
"""

import weakref
import sys
import time

import torchvision
from torchvision import transforms

import carla
import cv2
import numpy as np
import open3d as o3d

import EIdrive.core.sensing.perception.sensor_transformation as st
from EIdrive.core.common.misc import \
    cal_distance_angle, get_speed
from EIdrive.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from EIdrive.core.sensing.perception.static_obstacle import TrafficLight
from EIdrive.core.sensing.perception.o3d_lidar_libs import \
    o3d_visualizer_init, convert_raw_to_o3d_pointcloud, visualize_point_cloud, \
    camera_lidar_fusion_yolo
from PIL import Image
import torch
# from yolov5.detectclass import YoloDetector

from EIdrive.scenario_testing import demo


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

        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('fov', '100')

        camera_spawn_point = self.calculate_camera_spawn_point(relative_position, global_position)

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
    def calculate_camera_spawn_point(relative_position, global_position):
        """
        Calculate the spawn point for the camera based on the given relative and global positions. Adjustments are
        made according to the relative position specified (front, right, left, or default).

        Parameters
        ----------
        relative_position : str
            Relative position of the camera with respect to the vehicle.
            Accepted values are 'front', 'left', 'right', or any other value
            will default to the rear view.

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


class Perception:
    """
    Perception module to detect objects, such as vehicles.

    Parameters
    ----------
    vehicle : carla.Vehicle
        carla Vehicle to spawn sensors.

    config_yaml : dict
        Configuration for perception.

    world : EIdrive object
        EIdrive World object that saves all vehicle information, ML model, and id mapping dictionary.

    data_dump : bool
        Whether dumping data, if true, semantic lidar will be spawned.

    carla_world : carla.world
        CARLA world, used for rsu.

    Attributes
    ----------
    lidar : EIdrive object
        Lidar sensor.

    rgb_camera : EIdrive object
        RGB camera.

    o3d_vis : o3d object
        Open3d point cloud visualizer.
    """

    def __init__(self, vehicle, config_yaml, world,
                 data_dump=False, carla_world=None, infra_id=None):
        self.vehicle = vehicle
        self.carla_world = carla_world if carla_world is not None \
            else self.vehicle.get_world()
        self.id = infra_id if infra_id is not None else vehicle.id

        self.activate = config_yaml['activate']
        self.activate_model = config_yaml['model']
        self.camera_visualize = config_yaml['camera']['visualize']
        self.camera_num = min(config_yaml['camera']['num'], 4)
        self.lidar_visualize = config_yaml['lidar']['visualize']
        self.global_position = config_yaml['global']['position'] \
            if 'global_position' in config_yaml else None

        self.cav_world = weakref.ref(world)()
        ml_manager = world.ml_manager

        if self.activate and data_dump:
            sys.exit("When you dump data, please deactivate the "
                     "detection function for precise label.")

        if self.activate and not ml_manager:
            sys.exit(
                'If you activate the perception module, '
                'then apply_ml must be set to true in'
                'the argument parser to load the detection DL model.')
        self.ml_manager = ml_manager

        # we only spawn the camera when perception module is activated or
        # camera visualization is needed
        if self.activate or self.camera_visualize:
            self.rgb_camera = []
            mount_position = ['front', 'right', 'left', 'back']
            for i in range(self.camera_num):
                self.rgb_camera.append(
                    CameraSensor(
                        vehicle, self.carla_world, mount_position[i],
                        self.global_position))

        else:
            self.rgb_camera = None

        # we only spawn the LiDAR when perception module is activated or lidar visualization is needed
        if self.activate or self.lidar_visualize:
            self.lidar = LidarSensor(vehicle,
                                     self.carla_world,
                                     config_yaml['lidar'],
                                     self.global_position)
            self.o3d_vis = o3d_visualizer_init(self.id)
        else:
            self.lidar = None
            self.o3d_vis = None

        # if data dump is true, semantic lidar is also spawned
        self.data_dump = data_dump
        if data_dump:
            self.semantic_lidar = SemanticLidarSensor(vehicle,
                                                      self.carla_world,
                                                      config_yaml['lidar'],
                                                      self.global_position)

        # count how many steps have been passed
        self.count = 0
        # ego position
        self.ego_pos = None

        # the dictionary contains all objects
        self.objects = {}

    def dist(self, obstacle):
        """
        Get the distance from the obstacle to ego vehicle form server.

        Parameters
        ----------
        obstacle : carla.actor
            The obstacle vehicle.

        Returns
        -------
        distance : float
            The distance between ego and the target obstacle.
        """
        return obstacle.get_location().distance(self.ego_pos.location)

    def object_detect(self, ego_pos):
        """
        Detect surrounding objects.

        Parameters
        ----------
        ego_pos : carla.Transform
            Ego vehicle position.

        Returns
        -------
        objects : list
            A list that contains all detected obstacle vehicles.

        """
        self.ego_pos = ego_pos

        objects = {'vehicles': [],
                   'traffic_lights': []}

        if not self.activate:
            objects = self.server_detection(objects)

        else:
            if self.activate_model == "yolo":
                objects = self.yolo_detection(objects)
            elif self.activate_model == "ssd":
                objects = self.ssd_detection(objects)

        self.count += 1

        return objects

    def yolo_detection(self, objects):
        """
        Detect objects by Yolov5 + Lidar fusion.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all detected objects.
            The key is the object category name and value is its 3d coordinates and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        # retrieve current cameras and lidar data
        rgb_images = []
        for rgb_camera in self.rgb_camera:
            while rgb_camera.image is None:
                continue
            rgb_images.append(
                cv2.cvtColor(
                    np.array(
                        rgb_camera.image),
                    cv2.COLOR_BGR2RGB))
        # yolo detection
        yolo_detection = self.ml_manager.object_detector_yolo(rgb_images)

        # rgb_images for drawing
        rgb_draw_images = []

        for (i, rgb_camera) in enumerate(self.rgb_camera):
            # lidar projection
            rgb_image, projected_lidar = st.convert_lidar_to_camera(
                self.lidar.sensor,
                rgb_camera.sensor, self.lidar.data, np.array(
                    rgb_camera.image))
            rgb_draw_images.append(rgb_image)

            # camera lidar fusion
            objects = camera_lidar_fusion_yolo(
                objects,
                yolo_detection.xyxy[i],
                self.lidar.data,
                projected_lidar,
                self.lidar.sensor)

        # calculate the speed. current we retrieve from the server directly.
        self.get_speed(objects)

        if self.camera_visualize:
            names = ['front', 'right', 'left', 'back']
            for (i, rgb_image) in enumerate(rgb_draw_images):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break
                rgb_image = self.ml_manager.draw_2d_box(
                    yolo_detection, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)
                cv2.imshow(
                    '%s camera of actor %d, perception activated' %
                    (names[i], self.id), rgb_image)
            cv2.waitKey(1)

        if self.lidar_visualize:
            while self.lidar.data is None:
                continue
            convert_raw_to_o3d_pointcloud(self.lidar.data, self.lidar.o3d_pointcloud)
            visualize_point_cloud(
                self.o3d_vis,
                self.count,
                self.lidar.o3d_pointcloud,
                objects)
        # add traffic light
        objects = self.get_traffic_lights(objects)
        self.objects = objects

        return objects

    def ssd_detection(self, objects):
        """
        Detect objects by SSD (Single Shot MultiBox Detector).

        Parameters
        ----------
        objects : dict
            The dictionary that contains all detected objects.
            The key is the object category name and value is its 3d coordinates and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        rgb_images = []
        for rgb_camera in self.rgb_camera:
            while rgb_camera.image is None:
                continue
            rgb_images.append(
                rgb_camera.image),

        original_width = 800
        original_height = 600
        # Preprocess images for SSD
        input_size = 300  # For SSD300, adjust if you're using a different version
        processed_images = []

        for img in rgb_images:
            # Resize
            resized_img = cv2.resize(img, (input_size, input_size))
            # Normalize to [0,1]
            normalized_img = resized_img / 255.0
            processed_images.append(normalized_img)

        # Convert to PyTorch tensor and adjust dimensions (batch_size, channels, height, width)
        input_tensor = torch.tensor(processed_images).permute(0, 3, 1, 2).float()

        with torch.no_grad():
            detections_batch = self.ml_manager.object_detector_SSD(input_tensor)

        results_per_input = self.ml_manager.utils.decode_results(detections_batch)
        best_results_per_input = [self.ml_manager.utils.pick_best(results, 0.4) for results in results_per_input]
        classes_to_labels = self.ml_manager.utils.get_coco_object_dictionary()

        # Assuming each detection in best_results_per_input is in the format
        # [class_id, score, x_min, y_min, x_max, y_max]
        rescaled_results = []

        for image_detections in best_results_per_input:
            boxx, class_id, score = image_detections
            if boxx.any():
                x_min = boxx[0, 0]
                x_max = boxx[0, 1]
                y_min = boxx[0, 2]
                y_max = boxx[0, 3]
                x_min = x_min * original_width
                x_max = x_max * original_width
                y_min = y_min * original_height
                y_max = y_max * original_height
                rescaled_detections = [[x_min, y_min, x_max, y_max, class_id, score]]
                rescaled_results.append(rescaled_detections)
        if not rescaled_results:
            rescaled_detections = [[200, 300, 400, 500, 0.7, 1]]
            rescaled_results.append(rescaled_detections)
            rescaled_results.append(rescaled_detections)

        # rgb_images for drawing
        rgb_draw_images = []

        for (i, rgb_camera) in enumerate(self.rgb_camera):
            # lidar projection
            rgb_image, projected_lidar = st.convert_lidar_to_camera(
                self.lidar.sensor,
                rgb_camera.sensor, self.lidar.data, np.array(
                    rgb_camera.image))
            rgb_draw_images.append(rgb_image)

        # Calculate the speed. Current we retrieve from the server directly.
        self.get_speed(objects)

        if self.camera_visualize:
            names = ['front', 'right', 'left', 'back']
            for (i, rgb_image) in enumerate(rgb_draw_images):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break
                # rgb_image = self.ml_manager.draw_2d_box_SSD(
                #     rescaled_detections, classes_to_labels, rgb_image, i)
                rgb_image = self.ml_manager.draw_2d_box_SSD(
                    best_results_per_input, classes_to_labels, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)
                cv2.imshow(
                    '%s camera of actor %d, perception activated' %
                    (names[i], self.id), rgb_image)
            cv2.waitKey(1)

        if self.lidar_visualize:
            while self.lidar.data is None:
                continue
            convert_raw_to_o3d_pointcloud(self.lidar.data, self.lidar.o3d_pointcloud)
            visualize_point_cloud(
                self.o3d_vis,
                self.count,
                self.lidar.o3d_pointcloud,
                objects)
        # add traffic light
        objects = self.get_traffic_lights(objects)
        self.objects = objects
        return objects

    def server_detection(self, objects):
        """
        Object detection with information from server information.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all categories of detected objects.
            The key is the object category name and value is its 3d coordinates
            and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        objects = self.filter_and_update_vehicles(objects)
        self.visualize_camera(objects)
        self.visualize_lidar()
        objects = self.get_traffic_lights(objects)
        self.objects = objects

        return objects

    def filter_and_update_vehicles(self, objects):
        """Retrieve and filter vehicles, then update the object dictionary."""
        world = self.carla_world
        vehicle_list = world.get_actors().filter("*vehicle*")
        thresh = 50 if not self.data_dump else 120
        vehicle_list = [v for v in vehicle_list if self.dist(v) < thresh and v.id != self.id]

        if self.data_dump:
            vehicle_list = self.filter_vehicle_out_of_range(vehicle_list)

        if self.lidar:
            vehicle_list = [ObstacleVehicle(None, None, v, self.lidar.sensor, self.cav_world.sumo2carla_ids) for v in
                            vehicle_list]
        else:
            vehicle_list = [ObstacleVehicle(None, None, v, None, self.cav_world.sumo2carla_ids) for v in vehicle_list]

        objects.update({'vehicles': vehicle_list})
        return objects

    def visualize_camera(self, objects):
        """Visualize cameras"""
        if self.camera_visualize:
            while self.rgb_camera[0].image is None:
                continue

            names = ['front', 'right', 'left', 'back']

            for (i, rgb_camera) in enumerate(self.rgb_camera):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break

                rgb_image = np.array(rgb_camera.image)
                rgb_image = self.visualize_bbx_front_camera(objects, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.9, fy=1.0)

                cv2.imshow('%s camera of actor %d' % (names[i], self.id), rgb_image)
                self.camera_window_pos(names[i])

                cv2.waitKey(1)

    def camera_window_pos(self, name):
        """Position the visualization windows."""
        if demo.player_ids is None:
            pass
        elif len(demo.player_ids) != 6:
            pass
        elif self.id in demo.player_ids:
            if self.id == demo.player_ids[0]:
                cv2.moveWindow('%s camera of actor %d' % (name, self.id), 0, 0)
            if self.id == demo.player_ids[1]:
                cv2.moveWindow('%s camera of actor %d' % (name, self.id), 0, 750)
            if self.id == demo.player_ids[2]:
                cv2.moveWindow('%s camera of actor %d' % (name, self.id), 0, 1500)

    def visualize_lidar(self):
        """Visualize using lidar."""
        if self.lidar_visualize:
            while self.lidar.data is None:
                continue

            convert_raw_to_o3d_pointcloud(self.lidar.data, self.lidar.o3d_pointcloud)
            visualize_point_cloud(self.o3d_vis, self.count, self.lidar.o3d_pointcloud, self.objects)

    def filter_vehicle_out_of_range(self, vehicle_list):
        """
        By utilizing semantic lidar, we can retrieve the objects in the lidar detection range from the server and
        filter out those are not in range.

        Parameters
        ----------
        vehicle_list : list
            The list contains all vehicles information retrieves from the server.

        Returns
        -------
        new_vehicle_list : list
            The list that only contains vehicle in range.

        """
        # Get the indices of vehicles from the semantic lidar data
        vehicle_indices = self.semantic_lidar.obj_idx[self.semantic_lidar.obj_tag == 10]

        # Determine the unique vehicle IDs detected by the lidar
        detected_vehicle_ids = set(np.unique(vehicle_indices))

        # Filter vehicles based on whether their ID exists in the detected vehicle IDs
        vehicles_in_range = [veh for veh in vehicle_list if veh.id in detected_vehicle_ids]

        return vehicles_in_range

    def visualize_bbx_front_camera(self, objects, rgb_image, camera_index):
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
        """

        # Extract camera details
        camera = self.rgb_camera[camera_index].sensor
        camera_transform = camera.get_transform()
        camera_location = camera_transform.location
        camera_yaw_rotation = camera_transform.rotation.yaw

        # Iterate through each vehicle to visualize the bounding box
        for vehicle in objects.get('vehicles', []):
            _, angle_to_vehicle = cal_distance_angle(vehicle.get_location(), camera_location, camera_yaw_rotation)

            # Draw bounding box if vehicle is within camera field of view (FOV)
            if angle_to_vehicle < 60:
                bounding_box = st.get_2d_bbx(vehicle, camera, camera_transform)
                top_left = (int(bounding_box[0, 0]), int(bounding_box[0, 1]))
                bottom_right = (int(bounding_box[1, 0]), int(bounding_box[1, 1]))

                # Draw the rectangle on the RGB image
                cv2.rectangle(rgb_image, top_left, bottom_right, (255, 0, 0), thickness=2)

        return rgb_image

    def get_speed(self, detected_objects):
        """
        Get the speed of detected vehicles from the server.

        Parameters
        ----------
        detected_objects : dict
            Dictionary containing detected objects.
        """

        # Ensure vehicles exist in the detected objects
        if 'vehicles' not in detected_objects:
            return

        # Filter vehicles within a close radius
        nearby_vehicles = [
            vehicle for vehicle in self.carla_world.get_actors().filter("*vehicle*")
            if self.dist(vehicle) < 50 and vehicle.id != self.id
        ]

        # Match detected vehicles with the list from the server and retrieve speed
        for server_vehicle in nearby_vehicles:
            server_loc = server_vehicle.get_location()

            for detected_vehicle in detected_objects['vehicles']:
                current_speed = get_speed(detected_vehicle)

                # If speed has been matched, continue to next detected vehicle
                if current_speed > 0:
                    continue

                detected_loc = detected_vehicle.get_location()

                # Check proximity between server's vehicle and detected vehicle
                if abs(server_loc.x - detected_loc.x) <= 3.0 and \
                        abs(server_loc.y - detected_loc.y) <= 3.0:
                    detected_vehicle.set_velocity(server_vehicle.get_velocity())

                    # Assign the CARLA ID to the detected vehicle
                    detected_vehicle.set_carla_id(server_vehicle.id)

    def get_traffic_lights(self, detected_objects):
        """
        Get the nearby traffic lights directly from the server.

        Parameters
        ----------
        detected_objects : dict
            Dictionary containing all the detected objects.

        Returns
        -------
        detected_objects : dict
            Updated dictionary with traffic light information.
        """

        # Get list of all traffic lights from the CARLA world
        traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light*')

        nearby_lights = []

        for light in traffic_lights:
            if self.dist(light) < 50:
                light_info = TrafficLight(light.get_location(), light.get_state())
                nearby_lights.append(light_info)

        detected_objects['traffic_lights'] = nearby_lights

        return detected_objects

    def destroy(self):
        """
        Destroy all sensors.
        """
        if self.rgb_camera:
            for rgb_camera in self.rgb_camera:
                rgb_camera.sensor.destroy()

        if self.lidar:
            self.lidar.sensor.destroy()

        if self.camera_visualize:
            cv2.destroyAllWindows()

        if self.lidar_visualize:
            self.o3d_vis.destroy_window()

        if self.data_dump:
            self.semantic_lidar.sensor.destroy()
