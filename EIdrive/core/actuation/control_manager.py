# -*- coding: utf-8 -*-
"""
Controller interface
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import importlib


class ControlManager(object):
    """
    Controller manager that is used to select
    and call different controller's functions.

    Parameters
    ----------
    control_config : dict
        The configuration dictionary of the control manager module.

    Attributes
    ----------
    controller : EIdrive object.
        The controller object of the OpenCDA framework.
    """

    def __init__(self, control_config):
        controller_type = control_config['type']
        controller = getattr(
            importlib.import_module(
                "EIdrive.core.actuation.%s" %
                controller_type), 'Controller')
        self.controller = controller(control_config['args'])

    def update_info(self, ego_pos, ego_speed):
        """
        Update ego vehicle information for controller.
        """
        self.controller.update_info(ego_pos, ego_speed)

    def run_step(self, target_speed, waypoints):
        """
        Execute current controller step.
        """
        waypoint = waypoints
        control_command = self.controller.run_step(target_speed, waypoints)
        return control_command