"""
Class for RSU with perception, localization and V2X module.
"""

from EIdrive.core.sensing.perception.sensor_perception import Perception
from EIdrive.core.sensing.localization.rsu_localizer import RsuLocalizer


class RSU(object):
    """
    Road Side Unit for edge computing. It has its own perception, localization, and communication module.
    TODO: add V2X module to it to enable sharing sensing information online.

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

        # The rsu id is negative
        if self.rsu_id > 0:
            self.rsu_id = -self.rsu_id

        # Read map here to avoid repeatedly reading map
        self.carla_map = carla_map

        # Load config
        sensing_config = config_yaml['sensing']
        sensing_config['localization']['global_position'] = config_yaml['spawn_position']
        sensing_config['perception']['global_position'] = config_yaml['spawn_position']

        # Localizer
        self.localizer = RsuLocalizer(carla_world,
                                      sensing_config['localization'],
                                      self.carla_map)
        # Perception
        self.perception = Perception(vehicle=None,
                                     config_yaml=sensing_config['perception'],
                                     ml_model=ml_model,
                                     carla_world=carla_world,
                                     infra_id=self.rsu_id)

    def update_info(self):
        """
        Retrieve relative info for localization and perception.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception.object_detect(ego_pos)

    def destroy(self):
        """
        Destroy all actors.
        """
        self.perception.destroy()
        self.localizer.destroy()