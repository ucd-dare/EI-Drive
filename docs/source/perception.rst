Perception
===================================

In this section, we introduce the key class and attributes functions in the perception modules.

PerceptionMethods Class
------------------------

The `PerceptionMethods` class hosts all the perception methods used in the perception module. It is designed to handle multiple perception techniques, including YOLO ,SSD, and groundtruth.

Class Overview
^^^^^^^^^^^^^^

.. code-block:: python

   class PerceptionMethods:
       def __init__(self, config_yaml, ml_model, vehicle, id, carla_world=None):
           self.carla_world = carla_world if carla_world is not None else vehicle.get_world()
           self.vehicle = vehicle
           self.id = id
           self.activate = config_yaml['activate']
           self.camera_visualize = config_yaml['camera']['visualize']
           self.camera_num = min(config_yaml['camera']['num'], 4)
           self.lidar_visualize = config_yaml['lidar']['visualize']
           self.global_position = config_yaml.get('global_position')
           self.object_detection_model = ml_model.object_detection_model
           self.transmission_latency = config_yaml.get('transmission_latency')
           self.transmission_latency_in_sec = config_yaml.get('transmission_latency_in_sec')
           self.yolo_detected_objects_queue = deque(maxlen=1000)
           self.server_detected_objects_queue = deque(maxlen=1000)
           ...

Key Attribute Function: `yolo_detection`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `yolo_detection` method integrates YOLOv5 object detection with lidar data for robust perception. It detects objects in the scene and updates the object dictionary with their coordinates and confidence levels.

.. code-block:: python

   def yolo_detection(self, objects, ego_pos):
       # If no camera is created, stop detection.
       if len(self.rgb_camera) == 0:
           return {'vehicles': [], 'traffic_lights': []}
       
       # Retrieve current cameras and lidar data
       rgb_images = []
       for rgb_camera in self.rgb_camera:
           while rgb_camera.image is None:
               continue
           rgb_images.append(cv2.cvtColor(np.array(rgb_camera.image), cv2.COLOR_BGR2RGB))
       
       # YOLO detection
       yolo_detection = self.object_detection_model.object_detector_yolo(rgb_images)
       
       # Process lidar data and perform camera-lidar fusion
       rgb_draw_images = []
       for i, rgb_camera in enumerate(self.rgb_camera):
           rgb_image, projected_lidar = st.convert_lidar_to_camera(
               self.lidar.sensor, rgb_camera.sensor, self.lidar.data, np.array(rgb_camera.image)
           )
           rgb_draw_images.append(rgb_image)
           objects = camera_lidar_fusion_yolo(
               objects, yolo_detection.xyxy[i], self.lidar.data, projected_lidar, self.lidar.sensor
           )
       
       # Visualization (optional)
       if self.camera_visualize:
           for i, rgb_image in enumerate(rgb_draw_images):
               rgb_image = visualize_yolo_bbx(yolo_detection, rgb_image, i)
               cv2.imshow(f'Camera {i}', rgb_image)
           cv2.waitKey(1)
       
       # Add traffic light information
       objects = get_traffic_lights(objects, self.carla_world, ego_pos)
       return objects

The `PerceptionMethods` class also includes functions for:
- **SSD-based object detection:** `ssd_detection`
- **Server-based object detection:** `server_detection`

These methods provide flexible options for integrating different detection models and techniques in the simulation.


Perception Class
------------------

The `Perception` class serves as the central module for detecting objects. It integrates multiple perception methods and configurations, allowing for flexible and customizable object detection. 

.. note::
   
   Customized perception methods can be added by modifying the `object_detect` function.

.. code-block:: python

   class Perception:
       def __init__(self, vehicle, config_yaml, ml_model, carla_world=None, infra_id=None):
           self.carla_world = carla_world if carla_world is not None else vehicle.get_world()
           self.id = infra_id if infra_id is not None else vehicle.id
           self.activate = config_yaml['activate']
           self.activate_model = config_yaml['model']
           self.camera_visualize = config_yaml['camera']['visualize']
           self.lidar_visualize = config_yaml['lidar']['visualize']
           self.coop_perception = config_yaml.get('coop_perception', False)
           self.transmission_latency = config_yaml.get('transmission_latency')
           self.transmission_latency_in_sec = config_yaml.get('transmission_latency_in_sec')
           self.errors = config_yaml.get('errors')
           self.error_rate = config_yaml.get('error_rate')
           self.error_present = False
           self.global_position = config_yaml.get('global_position')
           self.PerceptionMethods = perception_methods.PerceptionMethods(config_yaml, ml_model, vehicle, self.id, carla_world)
           self.objects = {}

Key Attribute Function: `object_detect`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `object_detect` method identifies surrounding objects using the selected perception method. It supports multiple detection models and handles errors in the detection process.

.. code-block:: python

   def object_detect(self, ego_pos):
       objects = {'vehicles': [], 'traffic_lights': []}
       if not self.activate:
           objects = self.PerceptionMethods.server_detection(objects, ego_pos)
       else:
           if self.activate_model == "yolo":
               objects = self.PerceptionMethods.yolo_detection(objects, ego_pos)
           elif self.activate_model == "ssd":
               objects = self.PerceptionMethods.ssd_detection(objects, ego_pos)
           # Add additional perception methods here
           # elif self.activate_model == "{method_name}":
           #     objects = self.PerceptionMethods.{method_name}_detection(objects, ego_pos)
       if self.errors:
           self.error_present = spu.get_error(self.error_rate)
           if self.error_present:
               objects = {'vehicles': [], 'traffic_lights': []}
       return objects

Customizing the Perception Module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To add a new perception method:
1. Implement the method in the `PerceptionMethods` class.
2. Add the method to the `object_detect` function using the format:

   .. code-block:: python

      elif self.activate_model == "{method_name}":
          objects = self.PerceptionMethods.{method_name}_detection(objects, ego_pos)


CameraSensor Class
-------------------

The `CameraSensor` class manages the RGB camera for vehicles or infrastructure. It provides methods to calculate spawn points of camera and process camera data.

.. code-block:: python

   class CameraSensor:
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
           self.image_width = int(self.sensor.attributes['image_size_x'])
           self.image_height = int(self.sensor.attributes['image_size_y'])

calculate_camera_spawn_point
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Calculates the spawn point for the camera based on global or relative position.

.. code-block:: python

   @staticmethod
   def calculate_camera_spawn_point(is_rsu, relative_position, global_position):
         ...
       return spawn_point

process_camera_data
^^^^^^^^^^^^^^^^^^^

Processes incoming camera data and extracts RGB frames.

.. code-block:: python

   @staticmethod
   def process_camera_data(weak_self, event):
       self = weak_self()
       if not self:
           return
       image = np.array(event.raw_data)
       image = image.reshape((self.image_height, self.image_width, 4))
       image = image[:, :, :3]  # Remove the alpha channel
       self.image = image
       self.frame = event.frame
       self.timestamp = event.timestamp

LidarSensor Class
------------------

The `LidarSensor` class manages lidar data collection and processes point cloud data for vehicles or infrastructure.

.. code-block:: python

   class LidarSensor:
       def __init__(self, vehicle, world, config_yaml, global_position):
           if vehicle:
               world = vehicle.get_world()
           blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
           self.update_blueprint_attributes(blueprint, config_yaml)
           spawn_point = self.get_spawn_point(global_position)
           self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle) if vehicle else world.spawn_actor(
               blueprint, spawn_point)
           self.data = None
           self.timestamp = None
           self.frame = 0
           self.o3d_pointcloud = o3d.geometry.PointCloud()
           weak_self = weakref.ref(self)
           self.sensor.listen(lambda event: LidarSensor.process_lidar_data(weak_self, event))

get_spawn_point
^^^^^^^^^^^^^^^

Calculates the spawn point for the lidar sensor.

.. code-block:: python

   def get_spawn_point(self, global_position):
       if global_position is None:
           return carla.Transform(carla.Location(x=-0.5, z=1.9))
       return carla.Transform(carla.Location(x=global_position[0],
                                             y=global_position[1],
                                             z=global_position[2]))

process_lidar_data
^^^^^^^^^^^^^^^^^^

Processes lidar data to generate point clouds.

.. code-block:: python

   @staticmethod
   def process_lidar_data(weak_self, event):
       self = weak_self()
       if not self:
           return
       data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
       data = np.reshape(data, (int(data.shape[0] / 4), 4))  # (x, y, z, intensity)
       self.data = data
       self.frame = event.frame
       self.timestamp = event.timestamp

SemanticLidarSensor Class
--------------------------

The `SemanticLidarSensor` class manages semantic lidar data collection, used primarily for data dumping.

.. code-block:: python

   class SemanticLidarSensor:
       def __init__(self, vehicle, world, config_yaml, global_position):
           if vehicle:
               world = vehicle.get_world()
           blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
           self.update_blueprint_attributes(blueprint, config_yaml)
           spawn_point = self.get_spawn_point(global_position)
           self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle) if vehicle else world.spawn_actor(
               blueprint, spawn_point)
           self.points = None
           self.obj_idx = None
           self.obj_tag = None
           self.timestamp = None
           self.frame = 0
           self.o3d_pointcloud = o3d.geometry.PointCloud()
           weak_self = weakref.ref(self)
           self.sensor.listen(lambda event: SemanticLidarSensor.process_lidar_data(weak_self, event))


get_spawn_point
^^^^^^^^^^^^^^^

Calculates the spawn point for the semantic lidar sensor.

.. code-block:: python

   def get_spawn_point(self, global_position):
       if global_position is None:
           return carla.Transform(carla.Location(x=-0.5, z=1.9))
       return carla.Transform(carla.Location(x=global_position[0], y=global_position[1], z=global_position[2]))

process_lidar_data
^^^^^^^^^^^^^^^^^^

Processes semantic lidar data, extracting points, object indices, and tags.

.. code-block:: python

   @staticmethod
   def process_lidar_data(weak_self, event):
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

BoundingBox Class
-----------------

The `BoundingBox` class represents a bounding box for obstacles or vehicles. It calculates the central location and dimensions of an object based on its corners.

.. code-block:: python

   class BoundingBox:
       def __init__(self, corners):
           self.loc = self.compute_location(corners)
           self.size = self.compute_extent(corners)

compute_location
^^^^^^^^^^^^^^^^

Calculates the central location of the bounding box.

.. code-block:: python

   @staticmethod
   def compute_location(corners):
       avg_x, avg_y, avg_z = np.mean(corners, axis=0)
       return carla.Location(x=avg_x, y=avg_y, z=avg_z)

compute_extent
^^^^^^^^^^^^^^

Calculates the dimensions of the bounding box (width, height, and depth).

.. code-block:: python

   @staticmethod
   def compute_extent(corners):
       half_x = (np.max(corners[:, 0]) - np.min(corners[:, 0])) / 2
       half_y = (np.max(corners[:, 1]) - np.min(corners[:, 1])) / 2
       half_z = (np.max(corners[:, 2]) - np.min(corners[:, 2])) / 2
       return carla.Vector3D(x=half_x, y=half_y, z=half_z)

StaticObstacle Class
--------------------

The `StaticObstacle` class represents static obstacles such as stop signs or traffic lights. It encapsulates bounding box details for the obstacle.

.. code-block:: python

   class StaticObstacle:
       def __init__(self, bounding_corners, o3d_box):
           self.bbx = BoundingBox(bounding_corners)
           self.o3d_box = o3d_box

TrafficLight Class
------------------

The `TrafficLight` class maps traffic light information retrieved from the server and encapsulates its location and state.

.. code-block:: python

   class TrafficLight:
       def __init__(self, position, state):
           self.position = position
           self.state = state

get_state
^^^^^^^^^

Returns the current state of the traffic light.

.. code-block:: python

   def get_state(self):
       return self.state
