"""
Contains the ControlMethods class which implements different control methods for the ego vehicle.
"""

from collections import deque
import numpy as np
import math
import carla


class ControlMethods:
    """
    Helper class that implements different control methods for the ego vehicle.
    """

    def __init__(self, args, controller_type='PID'):
        
        # PID controller args
        if controller_type == 'PID':
            # Longitudinal (speed) control settings
            self.lon_kp = args['lon']['k_p']
            self.lon_kd = args['lon']['k_d']
            self.lon_ki = args['lon']['k_i']

            self.lon_error_buffer = deque(maxlen=10)

            # Lateral (steering) control settings
            self.lat_kp = args['lat']['k_p']
            self.lat_kd = args['lat']['k_d']
            self.lat_ki = args['lat']['k_i']

            self.lat_error_buffer = deque(maxlen=10)

            self.dt = args['dt']
        
        # Other controller types can be added here

    def PID_controller(self, desired_speed, desired_location, current_speed, current_transform):
        """
        Implements a PID controller for the ego vehicle.
        
        Parameters
        ----------
        desired_speed : float
            The target speed for the ego vehicle.
        desired_location : carla.location
            The location in the CARLA world towards which the ego vehicle should steer.
        current_speed : float
            The current speed of the ego vehicle.
        current_transform : carla.transform
            The current transform of the ego vehicle.
        
        Returns
        -------
        longitudinal_control : float
            The calculated throttle/acceleration command, constrained between -1.0 and 1.0, to move the vehicle towards
            the desired speed.
        lateral_control : float
            The calculated steering angle, constrained between -1.0 and 1.0, to guide the vehicle towards the desired location
        """

        def longitudinal_pid(desired_speed, current_speed):
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
            speed_error = desired_speed - current_speed
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

        def lateral_pid(desired_location, current_transform):
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
            starting_point = current_transform.location
            ending_point = starting_point + carla.Location(
                x=math.cos(math.radians(current_transform.rotation.yaw)),
                y=math.sin(math.radians(current_transform.rotation.yaw)))
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

        # Get the control commands from the PID controllers
        longitudinal_control = longitudinal_pid(desired_speed, current_speed)
        lateral_control = lateral_pid(desired_location, current_transform)

        # Return the control commands as acceleration and steering values
        return longitudinal_control, lateral_control

    # Other controller methods can be added here