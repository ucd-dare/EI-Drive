"""
Transfer coordinates to a different coordinate system
"""

import numpy as np


def convert_geo_to_esu(latitude, longitude, altitude, ref_latitude, ref_longitude, ref_altitude):
    """
    Transforms WGS84 coordinates to ESU (Unreal coordinate system) using a reference geo-point.

    Parameters
    ----------
    latitude : float
        Current point latitude.

    longitude : float
        Current point longitude.

    altitude : float
        Current point altitude.

    ref_latitude : float
        Reference latitude for the Unreal origin.

    ref_longitude : float
        Reference longitude for the Unreal origin.

    ref_altitude : float
        Reference altitude for the Unreal origin.

    Returns
    -------
    x : float
        x coordinates.

    y : float
        y coordinates.

    z : float
        z coordinates.
    """
    EARTH_RADIUS_EQUA = 6378137.0
    scaling_factor = np.cos(np.deg2rad(ref_latitude))

    y_component = longitude * np.pi * EARTH_RADIUS_EQUA * scaling_factor / 180
    ref_y = scaling_factor * np.deg2rad(ref_longitude) * EARTH_RADIUS_EQUA
    y = y_component - ref_y

    x_component = np.log(np.tan((latitude + 90) * np.pi / 360)) * EARTH_RADIUS_EQUA * scaling_factor
    ref_x = scaling_factor * EARTH_RADIUS_EQUA * np.log(np.tan((90 + ref_latitude) * np.pi / 360))
    x = x_component - ref_x

    z = altitude - ref_altitude

    return x, y, z
