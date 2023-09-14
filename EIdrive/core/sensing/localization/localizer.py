"""
Localization module
"""

import weakref
from collections import deque

import carla
import numpy as np

from EIdrive.core.basic.auxiliary import get_speed
from EIdrive.core.sensing.localization.kalman_filter import KalmanFilter
from EIdrive.core.sensing.localization.convert_coordinate \
    import convert_geo_to_esu


class GnssSensor(object):
    """
    The GNSS sensor module.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. Class to spawn GNSS and IMU sensor.

    config : dict
        The configuration of the localizer.

    Attributes
    ----------
    sensor : CARLA actor
        The expected sensor.
    """

    def __init__(self, vehicle, config):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')

        # Noise for GPS
        blueprint.set_attribute('noise_alt_stddev', str(config['noise_alt_stddev']))
        blueprint.set_attribute('noise_lat_stddev', str(config['noise_lat_stddev']))
        blueprint.set_attribute('noise_lon_stddev', str(config['noise_lon_stddev']))

        # Spawn the sensor
        self.sensor = world.spawn_actor(
            blueprint,
            carla.Transform(
                carla.Location(
                    x=0.0,
                    y=0.0,
                    z=0.0)),
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid)

        self.lat, self.lon, self.alt, self.timestamp = 0.0, 0.0, 0.0, 0.0

        # Create weak reference to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor.gnss_update(weak_self, event))

    @staticmethod
    def gnss_update(weak_self, event):
        """
        Callback function to update and store latest GNSS data.

        Parameters
        ----------
        weak_self : weakref
            A weak reference to the instance of the GnssSensor class.

        event : CARLA GNSS sensor data
            New GNSS data event captured by the sensor.
        """
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.timestamp = event.timestamp


class ImuSensor(object):
    """
    The IMU sensor module.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. Class to spawn GNSS and IMU sensor.

    Attributes
    ----------
    sensor : CARLA actor
        The expected sensor.
    """

    def __init__(self, vehicle):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=vehicle)

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: ImuSensor.imu_update(
                weak_self, sensor_data))
        self.gyroscope = None

    @staticmethod
    def imu_update(weak_self, sensor_data):
        """
        Callback method for the IMU sensor. Processes and stores accelerometer,
        gyroscope, and compass data from the sensor.

        Parameters
        ----------
        weak_self : weakref
            A weak reference to the instance of the ImuSensor class.

        sensor_data : CARLA IMU sensor data
            Data packet received from the IMU sensor.
        """
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)

        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))

        self.gyroscope = (
            max(limits[0], min(limits[1], sensor_data.gyroscope.x)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.y)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.z)))
        self.compass = sensor_data.compass


class Localizer(object):
    """
    The localizer module.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle.
    config_yaml : dict
        The configuration of the localizer.
    carla_map : carla.Map
        The carla HDMap.

    Attributes
    ----------
    gnss : EIdrive object
        GNSS sensor manager.

    imu : EIdrive object
        Imu sensor.

    kf : EIdrive object
        The Kalman Filter used to fuse different sensors.
    """

    def __init__(self, vehicle, config_yaml, carla_map):

        self.vehicle = vehicle
        self.activate = config_yaml['activate']
        self.map = carla_map
        self.geo_ref = self.map.transform_to_geolocation(carla.Location(x=0, y=0, z=0))

        self.ego_pos = None
        self.speed = 0

        # History info
        self.ego_pos_history = deque(maxlen=100)

        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        self.imu = ImuSensor(vehicle)

        # Add heading and speed noise
        self.heading_noise_std = \
            config_yaml['gnss']['heading_direction_stddev']
        self.speed_noise_std = config_yaml['gnss']['speed_stddev']

        self.dt = config_yaml['dt']

        # Kalman Filter
        self.kf = KalmanFilter(self.dt)


    def localize(self):
        """
        Localizes the vehicle's position by integrating GNSS and IMU sensor data.

        If localization is not activated, it retrieves the current vehicle position
        directly from the server. When activated, it uses sensors, combined
        with Kalman Filter, to get an estimation of the vehicle's position.
        """

        if not self.activate:
            self.ego_pos = self.vehicle.get_transform()
            self.speed = get_speed(self.vehicle)
        else:
            actual_speed = get_speed(self.vehicle)
            noisy_speed = self.set_speed_noise(actual_speed)

            # GNSS coordinates under ESU(Unreal coordinate system)
            x, y, z = convert_geo_to_esu(self.gnss.lat,
                                         self.gnss.lon,
                                         self.gnss.alt,
                                         self.geo_ref.latitude,
                                         self.geo_ref.longitude, 0.0)

            # Add noise to heading direction
            rotation = self.vehicle.get_transform().rotation
            noisy_heading_angle = self.set_heading_direction_noise(rotation.yaw)

            # Assume the initial position is accurate
            if len(self.ego_pos_history) == 0:
                x_kf, y_kf, heading_angle_kf = x, y, noisy_heading_angle
                self.speed = actual_speed
                self.kf.initiate_kf(x, y, np.deg2rad(noisy_heading_angle), self.speed / 3.6)
            else:
                x_kf, y_kf, heading_angle_kf, speed_kf = self.kf.run_kf_filter(
                    x, y, np.deg2rad(noisy_heading_angle),
                    noisy_speed / 3.6,
                    self.imu.gyroscope[2])
                self.speed = speed_kf * 3.6
                heading_angle_kf = np.rad2deg(heading_angle_kf)

            # The final position of the vehicle
            self.ego_pos = carla.Transform(
                carla.Location(
                    x=x_kf, y=y_kf, z=z), carla.Rotation(
                    pitch=0, yaw=heading_angle_kf, roll=0))

            # Save the data in the history
            self.ego_pos_history.append(self.ego_pos)

    def set_heading_direction_noise(self, heading_direction):
        """
        Set gaussian white noise to heading direction.

        Parameters
        ----------
        heading_direction : float
            Heading_direction obtained from the server.

        Returns
        ----------
        heading_direction : float
            Noisy heading direction.
        """
        return heading_direction + np.random.normal(0, self.heading_noise_std)

    def set_speed_noise(self, speed):
        """
        Set gaussian white noise to the current speed.

        Parameters
        __________
        speed : float
           Current speed from server.

        Returns
        -------
        speed : float
            The speed with noise.
        """
        return speed + np.random.normal(0, self.speed_noise_std)

    def get_ego_pos(self):
        """
        Get ego vehicle position.
        """
        return self.ego_pos

    def get_ego_speed(self):
        """
        Get ego vehicle speed.
        """
        return self.speed

    def destroy(self):
        """
        Destroy all the sensors.
        """
        self.gnss.sensor.destroy()
        self.imu.sensor.destroy()
