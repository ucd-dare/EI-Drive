"""
Hosts all the perception methods for the perception module.

Add additonal perception methods in the PerceptionMethods class.
"""


import cv2
import numpy as np
import math
import torch
from collections import deque
import EIdrive.core.sensing.perception.sensor_transformation as st
from EIdrive.core.sensing.perception.open3d_visualize import \
    convert_raw_to_o3d_pointcloud, visualize_point_cloud, \
    camera_lidar_fusion_yolo, camera_lidar_fusion_SSD
from EIdrive.core.sensing.perception.sensor_perception_utils import \
    filter_and_update_vehicles, get_traffic_lights, \
    visualize_bbx_front_camera, camera_window_pos, \
    visualize_yolo_bbx, visualize_ssd_bbx, \
    CameraSensor, LidarSensor
from EIdrive.core.sensing.perception.open3d_visualize import o3d_visualizer_init


class PerceptionMethods:
    """
    Hosts all the perception methods for the perception module.

    Parameters
    ----------
    config_yaml : dict
        The configuration yaml file that contains all the parameters for the perception module.
    ml_model : object
        The machine learning model object.
    vehicle : object
        The vehicle object.
    id : int
        The id of the ego vehicle.
    carla_world : object
        The carla world object.
    """

    def __init__(self, config_yaml, ml_model, vehicle, id, carla_world=None):
        
        self.carla_world = carla_world if carla_world is not None \
            else vehicle.get_world()
        self.vehicle = vehicle
        self.id = id

        self.activate = config_yaml['activate']
        self.camera_visualize = config_yaml['camera']['visualize']
        self.camera_num = min(config_yaml['camera']['num'], 4)
        self.lidar_visualize = config_yaml['lidar']['visualize']
        self.global_position = config_yaml['global_position'] \
            if 'global_position' in config_yaml else None
    
        object_detection_model = ml_model.object_detection_model

        # if self.activate and not object_detection_model:
        #     sys.exit(
        #         'If you activate the perception module, '
        #         'then apply_ml must be set to true in'
        #         'the argument parser to load the detection DL model.')
        self.object_detection_model = object_detection_model

        # Transmission model
        self.transmission_latency = config_yaml['transmission_latency'] \
            if 'transmission_latency' in config_yaml else None
        self.transmission_latency_in_sec = config_yaml['transmission_latency_in_sec'] \
            if 'transmission_latency_in_sec' in config_yaml else None
        
        self.yolo_detected_objects_queue = deque(maxlen=1000)
        self.server_detected_objects_queue = deque(maxlen=1000)

        # we only spawn the camera when perception module is activated or
        # camera visualization is needed
        if self.activate or self.camera_visualize:
            self.rgb_camera = []
            mount_position = ['front', 'right', 'left', 'back']
            for i in range(self.camera_num):
                self.rgb_camera.append(CameraSensor(vehicle, self.carla_world, mount_position[i], self.global_position))
        else:
            self.rgb_camera = None

        # we only spawn the LiDAR when perception module is activated or lidar visualization is needed
        if self.activate:
            self.lidar = LidarSensor(vehicle,
                                    self.carla_world,
                                    config_yaml['lidar'],
                                    self.global_position)
            if self.lidar_visualize:
                self.o3d_vis = o3d_visualizer_init(self.id)
            else:
                self.o3d_vis = None
        elif self.lidar_visualize:
            self.lidar = LidarSensor(vehicle,
                                    self.carla_world,
                                    config_yaml['lidar'],
                                    self.global_position)
            self.o3d_vis = o3d_visualizer_init(self.id)
        else:
            self.lidar = None
            self.o3d_vis = None

        self.count = 0


    def yolo_detection(self, objects, ego_pos):
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
        # If no camera is created, stop detection.
        if len(self.rgb_camera) == 0:
            return {'vehicles': [], 'traffic_lights': []}

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
        yolo_detection = self.object_detection_model.object_detector_yolo(rgb_images)

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

        if self.camera_visualize:
            names = ['front', 'right', 'left', 'back']
            for (i, rgb_image) in enumerate(rgb_draw_images):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break

                # Visualize object detection bbx
                rgb_image = visualize_yolo_bbx(yolo_detection, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=1.2, fy=1.2)

                # Creates the window for RGB camera
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
        objects = get_traffic_lights(objects, self.carla_world, ego_pos)
        current_objects = objects

        if self.transmission_latency and self.transmission_latency_in_sec > 0:
            latency = math.ceil(self.transmission_latency_in_sec / 0.05)  # The latency in ticks, which is the maximum length of the queue.
            if len(self.yolo_detected_objects_queue) == latency:
                final_objects = self.yolo_detected_objects_queue.popleft()
                self.yolo_detected_objects_queue.append(objects)
                objects = final_objects
            self.yolo_detected_objects_queue.append(current_objects)

        self.count += 1
        return objects


    def ssd_detection(self, objects, ego_pos):
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
        # If no camera is created, stop detection.
        if len(self.rgb_camera) == 0:
            return {'vehicles': [], 'traffic_lights': []}
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
        input_tensor = input_tensor.to('cuda')

        with torch.no_grad():
            detections_batch = self.object_detection_model.object_detector_SSD(input_tensor)
        results_per_input = self.object_detection_model.utils.decode_results(detections_batch)
        best_results_per_input = [self.object_detection_model.utils.pick_best(results, 0.4) for results in results_per_input]
        classes_to_labels = self.object_detection_model.utils.get_coco_object_dictionary()
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

            objects = camera_lidar_fusion_SSD(
                objects,
                rescaled_results[i],
                self.lidar.data,
                projected_lidar,
                self.lidar.sensor)

        if self.camera_visualize:
            names = ['front', 'right', 'left', 'back']
            for (i, rgb_image) in enumerate(rgb_draw_images):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break

                rgb_image = visualize_ssd_bbx(best_results_per_input, classes_to_labels, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=1.2, fy=1.2)
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
        objects = get_traffic_lights(objects, self.carla_world, ego_pos)

        self.count += 1
        return objects


    def server_detection(self, objects, ego_pos):
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

        current_objects = filter_and_update_vehicles(objects, self.carla_world, self.lidar, ego_pos, self.id)

        if self.camera_visualize:
            while self.rgb_camera[0].image is None:
                continue
            names = ['front', 'right', 'left', 'back']

            for (i, rgb_camera) in enumerate(self.rgb_camera):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break
                rgb_image = np.array(rgb_camera.image)
                rgb_image = visualize_bbx_front_camera(current_objects, rgb_image, i, self.rgb_camera)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=1.2, fy=1.2)

                cv2.imshow('%s camera of actor %d' % (names[i], self.id), rgb_image)
                camera_window_pos(names[i])
                cv2.waitKey(1)

        if self.lidar_visualize:
            while self.lidar.data is None:
                continue
            convert_raw_to_o3d_pointcloud(self.lidar.data, self.lidar.o3d_pointcloud)
            visualize_point_cloud(self.o3d_vis, self.count, self.lidar.o3d_pointcloud, objects)
        

        objects = get_traffic_lights(objects, self.carla_world, ego_pos)            
        current_objects = objects

        if self.transmission_latency and self.transmission_latency_in_sec > 0:
            latency = math.ceil(self.transmission_latency_in_sec / 0.05)  # The latency in ticks, which represents the maximum length of the queue.
            if len(self.server_detected_objects_queue) == latency:
                objects = self.server_detected_objects_queue.popleft()
            self.server_detected_objects_queue.append(current_objects)

        self.count += 1
        return objects
    

    # Add additional perception methods here