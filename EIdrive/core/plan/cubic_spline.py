"""
Cubic 1D and 2D spline for planning.
"""
import math
import numpy as np
import bisect


class CubicSpline:
    """
    A class to represent cubic splines used for interpolation and curvature calculation.

    Parameters:
    x : list[float]
        List of x-coordinates.
    y : list[float]
        List of corresponding y-coordinates.

    Attributes:
    a : list[float]
        Coefficients representing y-values at x.
    b, c, d : list[float]
        Cubic spline coefficients.
    x_vals : list[float]
        Input x-coordinates.
    num_points : int
        Number of data points.
    """

    def __init__(self, x, y):
        self.y = y
        self.a = y.copy()
        self.b, self.c, self.d = [], [], []
        self.x_vals = x
        self.num_points = len(x)
        h = np.diff(x)

        A = self.compute_A_matrix(h)
        B = self.compute_B_matrix(h)
        self.c = np.linalg.solve(A, B)

        for i in range(self.num_points - 1):
            d_coeff = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b_coeff = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(b_coeff)
            self.d.append(d_coeff)

    def interpolate(self, t):
        """Compute the interpolated y-value for a given x-value t."""
        if t < self.x_vals[0] or t > self.x_vals[-1]:
            return None

        idx = self.find_segment_index(t)
        dx = t - self.x_vals[idx]
        return self.a[idx] + self.b[idx] * dx + self.c[idx] * dx**2 + self.d[idx] * dx**3

    def first_derivative(self, t):
        """Compute the first derivative of the spline at t."""
        if t < self.x_vals[0] or t > self.x_vals[-1]:
            return None

        idx = self.find_segment_index(t)
        dx = t - self.x_vals[idx]
        return self.b[idx] + 2.0 * self.c[idx] * dx + 3.0 * self.d[idx] * dx**2

    def second_derivative(self, t):
        """Compute the second derivative of the spline at t."""
        if t < self.x_vals[0] or t > self.x_vals[-1]:
            return None

        idx = self.find_segment_index(t)
        dx = t - self.x_vals[idx]
        return 2.0 * self.c[idx] + 6.0 * self.d[idx] * dx

    def find_segment_index(self, x):
        """Determine the index of the segment that x belongs to."""
        return bisect.bisect(self.x_vals, x) - 1

    def compute_A_matrix(self, h):
        """Compute the A matrix for the cubic spline."""
        A = np.zeros((self.num_points, self.num_points))
        A[0, 0] = 1.0

        for i in range(self.num_points - 1):
            if i != self.num_points - 2:
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[self.num_points - 1] = 1.0
        return A

    def compute_B_matrix(self, h):
        """Compute the B matrix for the cubic spline."""
        B = np.zeros(self.num_points)
        for i in range(self.num_points - 2):
            B[i + 1] = (3.0 * (self.a[i + 2] - self.a[i + 1]) / h[i + 1]) - (3.0 * (self.a[i + 1] - self.a[i]) / h[i])
        return B


class Spline2D:
    """
    Represents a 2D Cubic Spline for path representation and calculations.

    Parameters:
    - x : list[float]
        List of x-coordinates.
    - y : list[float]
        List of y-coordinates.

    Attributes:
    - arc_lengths : list[float]
        Cumulative arc lengths along the path.
    - x_spline : object
        Cubic spline representation for x-coordinates.
    - y_spline : object
        Cubic spline representation for y-coordinates.
    """

    def __init__(self, x, y):
        self.arc_lengths = self.compute_arc_lengths(x, y)
        self.x_spline = CubicSpline(self.arc_lengths, x)
        self.y_spline = CubicSpline(self.arc_lengths, y)

    def compute_arc_lengths(self, x, y):
        """
        Compute cumulative arc lengths along the path using the given x and y coordinates.
        """
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(distances))
        return s

    def get_position(self, s):
        """
        Return interpolated x and y positions for a given arc length s.
        """
        x_pos = self.x_spline.interpolate(s)
        y_pos = self.y_spline.interpolate(s)
        return x_pos, y_pos

    def get_curvature(self, s):
        """
        Return curvature of the path at a given arc length s.
        """
        dx = self.x_spline.first_derivative(s)
        ddx = self.x_spline.second_derivative(s)
        dy = self.y_spline.first_derivative(s)
        ddy = self.y_spline.second_derivative(s)
        curvature = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(1.5))
        return curvature

    def get_yaw(self, s):
        """
        Return yaw (heading angle) of the path at a given arc length s.
        """
        dx = self.x_spline.first_derivative(s)
        dy = self.y_spline.first_derivative(s)
        return math.atan2(dy, dx)
