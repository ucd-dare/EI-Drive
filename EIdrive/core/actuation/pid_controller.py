"""
PID Control Class
"""
# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from collections import deque
import math
import numpy as np

import carla


class Controller:
    """
    Implements a PID Controller for vehicle control in the CARLA simulator.

    Parameters
    ----------
    args : dict
        Configuration settings parsed from a YAML file.

    Attributes
    ----------
    lon_error_buffer : deque
        Buffer for storing the history of longitudinal control errors. Limited to the 10 most recent errors.

    lat_error_buffer : deque
        Buffer for storing the history of latitudinal control errors. Limited to the 10 most recent errors.

    current_transform : carla.transform
        Represents the current transformation (location and orientation) of the ego vehicle in the CARLA world.

    current_speed : float
        Speed of the ego vehicle.

    previous_steering : float
        The steering angle from the previous control action.
    """

    def __init__(self, args):

        # Longitudinal (speed) control settings
        self.max_brake = args['max_brake']
        self.max_throttle = args['max_throttle']

        self.lon_kp = args['lon']['k_p']
        self.lon_kd = args['lon']['k_d']
        self.lon_ki = args['lon']['k_i']

        self.lon_error_buffer = deque(maxlen=10)

        # Lateral (steering) control settings
        self.max_steering = args['max_steering']
        self.max_steering_rate = args['max_steering_rate']

        self.lat_kp = args['lat']['k_p']
        self.lat_kd = args['lat']['k_d']
        self.lat_ki = args['lat']['k_i']

        self.lat_error_buffer = deque(maxlen=10)

        self.dt = args['dt']

        # Vehicle state information
        self.current_transform = None
        self.current_speed = 0.
        self.previous_steering = 0.

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

    def longitudinal_pid(self, desired_speed):
        """
        Computes the throttle command based on the difference between current speed and desired speed by PID.

        Parameters
        ----------
        desired_speed : float
            The target speed for the ego vehicle.

        Returns
        -------
        throttle_command : float
            The calculated throttle/acceleration command, constrained between -1.0 and 1.0, to move the vehicle towards
            the desired speed.
        """

        # Calculate the speed error
        speed_error = desired_speed - self.current_speed
        self.lon_error_buffer.append(speed_error)

        # Calculate derivative and integral components if there are enough error samples
        if len(self.lon_error_buffer) >= 2:
            derivative_error = (self.lon_error_buffer[-1] - self.lon_error_buffer[-2]) / self.dt
            integral_error = sum(self.lon_error_buffer) * self.dt
        else:
            derivative_error = 0.0
            integral_error = 0.0

        # Calculate the throttle command using the PID formula and constrain its value between -1 and 1
        throttle = self.lon_kp * speed_error + self.lon_kd * derivative_error + self.lon_ki * integral_error
        return np.clip(throttle, -1.0, 1.0)

    def lateral_pid(self, desired_location):
        """
        Computes the steering command based on the angle difference between the current orientation and the desired location.

        Parameters
        ----------
        desired_location : carla.location
            The location in the CARLA world towards which the ego vehicle should steer.

        Returns
        -------
        steering_command : float
            The calculated steering angle, constrained between -1.0 and 1.0, to guide the vehicle towards the desired location.
        """

        # Calculate the orientation vector of the vehicle
        starting_point = self.current_transform.location
        ending_point = starting_point + carla.Location(
            x=math.cos(math.radians(self.current_transform.rotation.yaw)),
            y=math.sin(math.radians(self.current_transform.rotation.yaw)))
        vehicle_vector = np.array([ending_point.x - starting_point.x, ending_point.y - starting_point.y, 0.0])

        # Calculate the vector from the vehicle to the desired location
        desired_vector = np.array([desired_location.x - starting_point.x, desired_location.y - starting_point.y, 0.0])

        # Calculate the angle difference between the two vectors
        angle_difference = math.acos(np.clip(
            np.dot(desired_vector, vehicle_vector) / (np.linalg.norm(desired_vector) * np.linalg.norm(vehicle_vector)),
            -1.0, 1.0))

        # Check the direction of the desired vector relative to the vehicle's orientation
        cross_product = np.cross(vehicle_vector, desired_vector)
        if cross_product[2] < 0:
            angle_difference *= -1.0

        self.lat_error_buffer.append(angle_difference)

        # Calculate derivative and integral terms for PID control
        if len(self.lat_error_buffer) >= 2:
            derivative_error = (self.lat_error_buffer[-1] - self.lat_error_buffer[-2]) / self.dt
            integral_error = sum(self.lat_error_buffer) * self.dt
        else:
            derivative_error = 0.0
            integral_error = 0.0

        # Compute the steering command using the PID formula and constrain between -1 and 1
        steering = (self.lat_kp * angle_difference) + (self.lat_kd * derivative_error) + (self.lat_ki * integral_error)
        return np.clip(steering, -1.0, 1.0)

    def run_controller(self, desired_speed, waypoints):
        """
        Executes a control step using both lateral and longitudinal PID controllers
        to approach a target waypoint with a specified desired_speed.

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
            location = waypoints[0].location
            desired_speed = desired_speed[0]

        # Apply emergency stop if required conditions are met
        if desired_speed == 0 or not waypoints:
            control.steer, control.throttle, control.brake = 0.0, 0.0, 1.0
            control.hand_brake = False
            return control

        # Compute the acceleration and steering values using PID controllers
        acceleration = self.longitudinal_pid(desired_speed)
        current_steering = self.lateral_pid(location)

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
