# -*- coding: utf-8 -*-
"""
Customized Localization Module.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from EIdrive.core.sensing.localization.localizer \
    import Localizer
from EIdrive.customize.core.sensing.localization.extented_kalman_filter \
    import ExtentedKalmanFilter


class CustomizedLocalizer(Localizer):
    """Customized Localization module to replace the default module.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    -config_yaml: dict
        The configuration dictionary of the localization module.
    -carla_map: carla.Map
        The carla HDMap. We need this to find the map origin
        to convert wg84 to enu coordinate system.

    Attributes
    -kf : EIdrive object
        The filter used to fuse different sensors.
    """

    def __init__(self, vehicle, config_yaml, carla_map):
        super(
            CustomizedLocalizer,
            self).__init__(
            vehicle,
            config_yaml,
            carla_map)
        self.kf = ExtentedKalmanFilter(self.dt)
