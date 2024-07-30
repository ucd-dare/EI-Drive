"""
Perception module that detects objects, such as vehicles.

Modify object_detect method to add additional perception methods.
"""

import cv2
import EIdrive.core.sensing.perception.perception_methods as perception_methods
import EIdrive.core.sensing.perception.sensor_perception_utils as spu

class Perception:
    """
    Perception module to detect objects, such as vehicles.

    Parameters
    ----------
    vehicle : carla.Vehicle
        carla Vehicle to spawn sensors.

    config_yaml : dict
        Configuration for perception.

    ml_model : EIdrive object
        ML model object.

    carla_world : carla.world
        CARLA world, used for rsu.

    Attributes
    ----------
    coop_perception : bool
        Indicator of whether using cooperative perception.
    transmission_latency : bool
        Indicator of whether using transmission latency.
    transmission_latency_in_sec : float
        Transmission latency in seconds.
    errors : bool
        Indicator of whether using errors.
    error_rate : float
        Error rate.
    error_present : bool
        Indicator of whether errors are present at this tick.
    """

    def __init__(self, vehicle, config_yaml, ml_model,
                    carla_world=None, infra_id=None):
        
        self.carla_world = carla_world if carla_world is not None \
            else vehicle.get_world()
        self.id = infra_id if infra_id is not None else vehicle.id

        self.activate = config_yaml['activate']
        self.activate_model = config_yaml['model']
        self.camera_visualize = config_yaml['camera']['visualize']
        self.lidar_visualize = config_yaml['lidar']['visualize']

        # If coop_perception is activated in the config file, then set it
        self.coop_perception = config_yaml['coop_perception'] \
            if 'coop_perception' in config_yaml else False
        # Transmission model
        self.transmission_latency = config_yaml['transmission_latency'] \
            if 'transmission_latency' in config_yaml else None
        self.transmission_latency_in_sec = config_yaml['transmission_latency_in_sec'] \
            if 'transmission_latency_in_sec' in config_yaml else None
        
        # Error in transmission
        self.errors = config_yaml['errors'] \
            if 'errors' in config_yaml else None
        self.error_rate = config_yaml['error_rate'] \
            if 'error_rate' in config_yaml else None
        self.error_present = False
        

        self.global_position = config_yaml['global_position'] \
            if 'global_position' in config_yaml else None
        
        # Perception module configuration
        self.PerceptionMethods = perception_methods.PerceptionMethods(config_yaml, ml_model, vehicle, self.id, carla_world)

        # the dictionary contains all objects
        self.objects = {}

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

        objects = {'vehicles': [],
                    'traffic_lights': []}

        # Run server detection if perception not activated
        if not self.activate:
            objects = self.PerceptionMethods.server_detection(objects, ego_pos)

        else:
            # Run the selected model based on perception yaml file
            if self.activate_model == "yolo":
                objects = self.PerceptionMethods.yolo_detection(objects, ego_pos)
            elif self.activate_model == "ssd":
                objects = self.PerceptionMethods.ssd_detection(objects, ego_pos)


            # Add addtional perception methods here:

            #elif self.activate_model == "{addtional method name}:
            #    objects = self.PerceptionMethods.{your method here}_detection(objects, ego_pos)


        # If errors are enabled, remove all detected objects and return empty list if error is present
        if self.errors:
            self.error_present = spu.get_error(self.error_rate)
            if self.error_present:
                objects = {'vehicles': [],
                    'traffic_lights': []}

        return objects

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
