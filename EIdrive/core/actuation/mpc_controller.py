import math
import numpy as np
import scipy.optimize as opt
import matplotlib.pyplot as plt
from collections import deque

import carla
import logging


class Controller:
    """
    MPC Controller implementation.

    Parameters
    ----------
    args : dict
        The configuration dictionary parsed from yaml file.

    Attributes
    ----------
    lon_error_buffer : deque
        A deque buffer that stores longitudinal control errors.
    """

    def __init__(self, args):
        self.wheelbase = args['wheelbase']
        self.Q = args['Q']
        self.Q_W = args['Q_W']
        self.R = args['R']
        self.R_W = args['R_W']
        self.M = args['M']
        self.M_W = args['M_W']
        self.max_a = args['max_a']
        self.min_a = args['min_a']
        self.min_steering = args['min_steering']
        self.max_steering = args['max_steering']
        self.max_brake = args['max_brake']
        self.max_throttle = args['max_throttle']
        self.prediction_horizon = args['prediction_horizon']
        self.dt = 0.05
        self.u_cur = np.array([[0, 0.1]])

        # current speed and localization retrieved from sensing layer
        self.current_transform = None
        self.current_speed = 0.
        self.tick = 0
        logging.basicConfig(filename='example.log', level=logging.DEBUG)

    def vehicle_dynamics(self, x, u):
        # Example vehicle dynamics
        x_next = np.ndarray(shape=(4,))
        x_next[0] = x[0] + x[2] * math.cos(x[3]) * self.dt
        x_next[1] = x[1] + x[2] * math.sin(x[3]) * self.dt
        x_next[2] = x[2] + u[0] * self.dt
        x_next[3] = x[3] + x[2] * math.tan(u[1]) / self.wheelbase * self.dt
        return x_next

    def cost(self, u, x, x_ref, dt, prediction_horizon):
        """
        u (prediction_horizon*2 np.array) control input
        """
        cost = 0.0
        x_cur = x
        u = u.reshape(prediction_horizon, 2)
        for i in range(prediction_horizon - 1):
            if i >= len(u):
                print('cost terminal stop')
                return cost  # Prevent possible stop at terminal
            x_next = self.vehicle_dynamics(x_cur, u[i])
            cost_deviation = np.dot(np.dot((x_next - x_ref[i]), self.Q), (x_next - x_ref[i]).T) * self.Q_W
            cost += cost_deviation  # Cost for deviation
            cost_input = np.dot(np.dot(u[i], self.R), u[i].T) * self.R_W
            cost += cost_input  # Cost for input
            x_cur = x_next
        x_next = self.vehicle_dynamics(x_cur, u[prediction_horizon - 1])
        cost += np.dot(np.dot((x_next - x_ref[prediction_horizon - 1]), self.M),
                       (x_next - x_ref[prediction_horizon - 1]).T) * self.M_W  # Cost for terminal deviation
        return cost

    # TODO: the function is not completed yet.
    def constraints_model(self, u):
        x_pred = np.copy(x)
        constraints_values = []
        for i in range(prediction_horizon):
            x_next = vehicle_dynamics(x_pred, u[i])
            for constraint in constraints:
                constraints_values.append(constraint(x_pred, u[i]))
            x_pred = x_next
        return constraints_values

    def update_info(self, ego_pos, ego_spd):
        """
        Update ego position and speed to controller.

        Parameters
        ----------
        ego_pos : carla.location
            Position of the ego vehicle.

        ego_spd : float
            Speed of the ego vehicle

        Returns
        -------

        """
        self.current_transform = ego_pos
        self.current_speed = ego_spd

    def vehicle_mpc(self, x, u, x_ref, dt, prediction_horizon):
        """
        Implement a Model Predictive Controller (MPC) for a vehicle.

        Parameters: x (np.array): Current state of the vehicle.
        u (np.array): Current control inputs.
        dt (float): Time step for simulation.
        vehicle_dynamics (callable): Function that returns the next state of the vehicle given the current state and control inputs.
        cost_function (callable): Function that returns the cost given the current state and control inputs.
        prediction_horizon (int): Number of steps to predict into the future.
        constraints (tuple): Tuple of constraint functions.

        Returns:
            u_opt (np.array): Optimal control inputs.
        """

        u_0 = np.repeat([u], self.prediction_horizon, axis=0)
        u_bounds = np.repeat((((self.min_a, self.max_a), (self.min_steering, self.max_steering))),
                             self.prediction_horizon, axis=0)  # Bounds for input u
        # constraints_bounds = [(0.0, None) for i in range(len(constraints) * prediction_horizon)]
        # print('vehicle_MPC:\n'+str(u_0))
        u_opt = opt.minimize(self.cost, u_0, args=(x, x_ref, dt, self.prediction_horizon),
                             bounds=None)

        return u_opt.x

    def run_controller(self, target_speed, waypoints):

        ref_waypoints = deque()
        for i in range(0, self.prediction_horizon):
            ref_waypoints.append(waypoints[i])
            # ref_waypoints.append(waypoints[i][0])  original version
        x_ref = deque()
        y_ref = deque()
        v_ref = deque()
        yaw_ref = deque()
        for loc in ref_waypoints:
            x_ref.append(loc.location.x)
            y_ref.append(loc.location.y)
            v_ref.append(0)
            yaw_ref.append(0.0)        # loc.rotation.yaw
        ref_trajectory = np.array((x_ref, y_ref, v_ref, yaw_ref))
        # print(x_ref)
        # print(ref_trajectory)

        ref_trajectory = np.transpose(ref_trajectory)
        loc_cur = np.array([self.current_transform.location.x, self.current_transform.location.y, 0,
                            self.current_transform.rotation.yaw])

        control = carla.VehicleControl()
        u_opt = self.vehicle_mpc(loc_cur, self.u_cur, ref_trajectory, self.dt, self.prediction_horizon)
        self.u_cur = u_opt.reshape((5, 2))[0]
        print(u_opt[1])
        if u_opt[1] > self.max_steering:
            control.steer = self.max_steering
        elif u_opt[1] < self.min_steering:
            control.steer = self.min_steering
        else:
            control.steer = u_opt[1]
        logging.info(control.steer)
        if u_opt[0] >= 0:
            control.throttle = min(u_opt[0], self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(u_opt[0]), self.max_brake)
        control.hand_brake = False
        control.manual_gear_shift = False
        #print(f'tick={self.tick}, throttle={control.throttle}, break={control.brake}, steering={control.steer}')
        self.tick = self.tick + 1
        return control
