"""
Class for RSU with perception, localization and V2X module.
"""

from EIdrive.core.sensing.perception.sensor_perception import Perception
from EIdrive.core.sensing.localization.rsu_localizer import RsuLocalizer


class RSU(object):
    """
    Road Side Unit for edge computing. It has its own perception, localization, and communication module.

    Parameters
    ----------
    carla_world : carla.World
        CARLA world.

    config_yaml : dict
        The configuration for the RSU.

    carla_map : carla.Map
        The CARLA map.

    ml_model : EIdrive object
        ML model object.

    Attributes
    ----------
    localizer : EIdrive object
        The localization module.

    perception : EIdrive object
        The perception module.

    """
    def __init__(
            self,
            carla_world,
            config_yaml,
            carla_map,
            ml_model
    ):

        self.rsu_id = config_yaml['id']
        self.detected_objects = None

        # The rsu id is negative
        if self.rsu_id > 0:
            self.rsu_id = -self.rsu_id
            print('Make sure RSU id is negative.')

        # Read map here to avoid repeatedly reading map
        self.carla_map = carla_map

        # Load config
        localization_config = config_yaml['localization']
        perception_config = config_yaml['perception']
        localization_config['global_position'] = config_yaml['spawn_position']
        perception_config['global_position'] = config_yaml['spawn_position']

        # Localizer
        self.localizer = RsuLocalizer(carla_world,
                                      localization_config,
                                      self.carla_map)
        # Perception
        self.perception = Perception(vehicle=None,
                                     config_yaml=perception_config,
                                     ml_model=ml_model,
                                     carla_world=carla_world,
                                     infra_id=self.rsu_id)

    def update_info(self, dynamic_latency=None):
        """
        Retrieve relative info for localization and perception.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()

        # object detection
        if dynamic_latency is not None:
            self.perception.update_trans_latency(dynamic_latency)
        self.detected_objects = self.perception.object_detect(ego_pos)

    def destroy(self):
        """
        Destroy all actors.
        """
        self.perception.destroy()
        self.localizer.destroy()
