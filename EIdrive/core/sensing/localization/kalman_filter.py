"""
Use Kalman Filter to fuse GPS + IMU for localization.
"""

import math
import numpy as np


class KalmanFilter:
    """
    Implements a Kalman Filter for GPS and IMU data fusion.

    Parameters
    ----------
    dt : float
        Time interval between filter steps.

    Attributes
    ----------
    Q : numpy.array
        Covariance matrix for state prediction.

    R : numpy.array
        Covariance matrix for observation in x, y position.

    time_step : float
        Interval between filter updates (same as dt).

    xEst : numpy.array
        Array of estimated states.

    PEst : numpy.array
        Covariance matrix of estimated states.
    """

    def __init__(self, dt):
        # Covariance for the prediction step (Q matrix).
        self.Q = np.diag([
            0.2,  # Variance in x position.
            0.2,  # Variance in y position.
            np.deg2rad(0.1),  # Variance in yaw.
            0.001  # Variance in velocity.
        ]) ** 2

        # Covariance for observation (R matrix).
        self.R = np.diag([
            0.5,  # Variance in x observation.
            0.5,  # Variance in y observation.
            0.2  # Variance in yaw observation.
        ]) ** 2

        self.time_step = dt

        # Initializing estimated states (x) and covariance (P) matrix.
        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)

    def dynamic_model(self, x, u):
        """
        Predicts the next state using the dynamic model: X = F * X_prev + B * u.

        Parameters
        ----------
        x : np.array
            Previous state with [x_position, y_position, yaw_angle, velocity]. Shape: (4, 1).

        u : np.array
            Control inputs with [current_velocity, yaw_rate_from_imu]. Shape: (2, 1).

        Returns
        -------
        x : np.array
            Predicted state for the next time step.
        """
        F = np.array([[1.0, 0, 0, 0],
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0]])

        B = np.array([[self.time_step * math.cos(x[2, 0]), 0],
                      [self.time_step * math.sin(x[2, 0]), 0],
                      [0.0, self.time_step],
                      [1.0, 0.0]])

        x = F @ x + B @ u

        return x

    def observation_model(self, x):
        """
        Converts the state into a format expected by the sensor, i.e., the measurement model.

        Parameters
        ----------
        x : np.array
            State vector with elements [x_position, y_position, yaw_angle, velocity]. Shape: (4, 1).

        Returns
        -------
        z : np.array
            Transformed state representing the predicted sensor measurement.
        """
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])

        z = H @ x

        return z

    def initiate_kf(self, x, y, heading, velocity):
        """
        Initial state filling of Kalman filter.

        Parameters
        ----------
        x : float
            The x coordinate.

        y : float
            The y coordinate.

        heading : float
            The heading direction.

        velocity : float
            Velocity.

        """
        self.xEst[0] = x
        self.xEst[1] = y
        self.xEst[2] = heading
        self.xEst[3] = velocity

    def run_kf_filter(self, x, y, heading, v, imu_yaw_rate):
        """
        Updates the state using Kalman Filter based on current measurements and previous predictions.

        Parameters
        ----------
        x : float
            x-coordinate from GNSS sensor for the current time step.

        y : float
            y-coordinate from GNSS sensor for the current time step.

        heading : float
            Heading direction for the current time step.

        v : float
            Speed for the current time step.

        imu_yaw_rate : float
            Yaw rate in rad/s sourced from the IMU sensor.

        Returns
        -------
        corrected_info : tuple
            Tuple containing corrected x, y, heading, and velocity values.
        """
        # Measurement vector from GNSS
        z = np.array([x, y, heading]).reshape(3, 1)

        # Control input vector (velocity and IMU yaw rate)
        u = np.array([v, imu_yaw_rate]).reshape(2, 1)

        # Predict the next state based on the dynamic model
        xPred = self.dynamic_model(self.xEst, u)

        # Predict the next measurement based on the observation model
        zPred = self.observation_model(xPred)
        y = z - zPred

        # Observation matrix
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])

        # State transition matrix
        F = np.array([
            [1.0, 0, 0, 0],
            [0, 1.0, 0, 0],
            [0, 0, 1.0, 0],
            [0, 0, 0, 0]
        ])

        # Covariance prediction for the next step
        PPred = F @ self.PEst @ F.T + self.Q
        S = np.linalg.inv(H @ PPred @ H.T + self.R)
        K = PPred @ H.T @ S

        # Update estimated state and covariance matrix
        self.xEst = xPred + K @ y
        self.PEst = K @ H @ PPred

        return self.xEst[0][0], \
            self.xEst[1][0], \
            self.xEst[2][0], \
            self.xEst[3][0]
