"""
Localization module for RSU.
"""

import weakref
from collections import deque

import carla

from EIdrive.core.sensing.localization.convert_coordinate \
    import convert_geo_to_esu


class RsuGnssSensor(object):
    """
    The RSU GNSS sensor module.

    Parameters
    ----------
    world : carla.world
        The carla.World.

    config : dict
        The configuration of the localizer.

    rsu_position : list
        The global position of the RSU.

    Attributes
    ----------
    sensor : CARLA actor
        The expected sensor.
    """

    def __init__(self, world, config, rsu_position):
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
                    x=rsu_position[0],
                    y=rsu_position[1],
                    z=rsu_position[2])))

        self.lat, self.lon, self.alt, self.timestamp = 0.0, 0.0, 0.0, 0.0

        # Create weak reference to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: RsuGnssSensor.rsu_gnss_update(weak_self, event))

    @staticmethod
    def rsu_gnss_update(weak_self, event):
        """
        Callback function to update and store latest GNSS data.

        Parameters
        ----------
        weak_self : weakref
            A weak reference to the instance of the RsuGnssSensor class.

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


class RsuLocalizer(object):
    """
    Localization module for RSU (Roadside Units).

    Parameters
    ----------
    world : carla.world
        The CARLA world.

    config_yaml : dict
        Configuration for localization.

    carla_map : carla.Map
        The High-Definition Map of CARLA.
    """

    def __init__(self, world, config_yaml, carla_map):

        self.activate = config_yaml['activate']
        self.map = carla_map
        self.geo_ref = self.map.transform_to_geolocation(carla.Location(x=0, y=0, z=0))

        self.ego_position = None
        self.speed = 0

        # History info
        self.ego_position_history = deque(maxlen=100)

        self.rsu_gnss = RsuGnssSensor(world,
                                      config_yaml['gnss'],
                                      config_yaml['global_position'])
        self.true_ego_pos = carla.Transform(
            carla.Location(x=config_yaml['global_position'][0],
                           y=config_yaml['global_position'][1],
                           z=config_yaml['global_position'][2]))
        self.speed = 0

    def localize(self):
        """
        Determines the RSU's position.

        If localization is inactive, it fetches the pre-defined position
        directly from the configuration. When activated, it converts GNSS
        coordinates to the Unreal coordinate system (ESU) for accurate position.
        """

        if not self.activate:
            self.ego_position = self.true_ego_pos
        else:
            x, y, z = convert_geo_to_esu(self.rsu_gnss.lat,
                                         self.rsu_gnss.lon,
                                         self.rsu_gnss.alt,
                                         self.geo_ref.latitude,
                                         self.geo_ref.longitude, 0.0)
            self.ego_position = carla.Transform(carla.Location(x=x, y=y, z=z))

    def get_ego_pos(self):
        """
        Get ego vehicle position
        """
        return self.ego_position

    def get_ego_spd(self):
        """
        Get ego vehicle speed
        """
        return self.speed

    def destroy(self):
        """
        Destroy the sensors
        """
        self.rsu_gnss.sensor.destroy()
