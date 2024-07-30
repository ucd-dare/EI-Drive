"""
Controller class for vehicle control in the CARLA simulator.
"""

import numpy as np
from EIdrive.core.actuation.control_methods import \
    ControlMethods as cm
import carla


class Controller:
    """
    Implements a controller class for vehicle control in the CARLA simulator.

    Attributes
    ----------
    args : dict
        Dictionary containing the controller arguments.
    controller_type : str
        The type of controller to be used for vehicle control.
    """

    def __init__(self, args, controller_type='PID'):

        self.args = args
        self.control_method = controller_type

        self.max_brake = args['max_brake']
        self.max_throttle = args['max_throttle']

        self.max_steering = args['max_steering']
        self.max_steering_rate = args['max_steering_rate']

        # Vehicle state information
        self.current_transform = None
        self.current_speed = 0.
        self.previous_steering = 0.

        self.cm = cm(args, controller_type)

    def update_info(self, current_pos, current_speed):
        """
        Update ego position and speed to controller.

        Parameters
        ----------
        current_pos : carla.location
            Position of the ego vehicle.

        current_speed : float
            Speed of the ego vehicle

        Returns
        -------

        """
        self.current_transform = current_pos
        self.current_speed = current_speed

    def get_control(self, desired_speed, desired_location):
        """
        Get control from a specific controller based on the desired speed and location.
        
        Parameters
        ----------
        desired_speed : float
            The target speed for the ego vehicle.
        desired_location : carla.location
            The target location for the ego vehicle.
        """

        if self.control_method == 'PID':
            acceleration, current_steering = self.cm.PID_controller(
                                                desired_speed, desired_location,
                                                self.current_speed, self.current_transform)       
        return acceleration, current_steering
    

    def run_controller(self, desired_speed, waypoints):
        """
        Executes a control step using to approach a target waypoint with a specified desired_speed.

        Parameters
        ----------
        desired_speed : deque
            The queue of upcoming target speeds for the ego vehicle.

        waypoints : list of carla.location
            The list of upcoming waypoints the vehicle should follow.

        Returns
        -------
        control : carla.VehicleControl
            The determined vehicle control command for the current timestep.
        """

        # Initialize vehicle control class for CARLA
        control = carla.VehicleControl()

        # Check if waypoints are provided and valid
        if not waypoints:
            # print("Waypoints are empty.")
            return control
        else:
            desired_location = waypoints[0].location
            desired_speed = desired_speed[0]

        # Apply emergency stop if required conditions are met
        if desired_speed == 0 or not waypoints:
            control.steer, control.throttle, control.brake = 0.0, 0.0, 1.0
            control.hand_brake = False
            return control

        # Get acceleration and steering values from the controller
        acceleration, current_steering = self.get_control(desired_speed, desired_location)

        # Define throttle and brake commands based on acceleration value
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Ensure steering change is smooth and within allowable limits
        current_steering = np.clip(current_steering,
                                    self.previous_steering - self.max_steering_rate,
                                    self.previous_steering + self.max_steering_rate)

        # Limit the steering value within the maximum allowable range
        steering = np.clip(current_steering, -self.max_steering, self.max_steering)

        # Assign control variables
        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.previous_steering = steering

        return control
