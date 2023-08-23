"""
Functions for drawing map
"""

import numpy as np
import cv2

# Sub-pixel drawing precision constants
CV2_SUB_VALUES = {"shift": 9, "lineType": cv2.LINE_AA}
CV2_SHIFT_VALUE = 2 ** CV2_SUB_VALUES["shift"]
INTERPOLATION_POINTS = 20
AGENT_COLOR = (255, 255, 255)
ROAD_COLOR = (17, 17, 31)
Lane_COLOR = {'normal': (255, 217, 82),
              'red': (255, 0, 0),
              'yellow': (255, 255, 0),
              'green': (0, 255, 0)}


def cv2_subpixel(coords: np.ndarray) -> np.ndarray:
    """
    Convert floating-point coordinates to higher precision integer coordinates for cv2.

    This function scales the coordinates by a predefined constant (CV2_SHIFT_VALUE)
    and then converts them to integers. When using OpenCV functions with a shift
    parameter, this allows for subpixel precision.

    Parameters:
    -----------
        coords (np.ndarray): Floating-point XY coordinates.

    Returns:
    --------
        np.ndarray: Scaled integer coordinates suitable for cv2 with shift operations.
    """
    coords = coords * CV2_SHIFT_VALUE
    coords = coords.astype(np.int)
    return coords


def render_agent(agent_list, image):
    """
    Draw agents by their corners.

    Parameters
    ----------
    agent_list : list
        List containing the corner coordinates of each agent.

    image : np.ndarray
        The target image onto which the agents will be drawn.

    Returns
    -------
    np.ndarray
        The image with the agents.
    """
    for agent_corner in agent_list:
        agent_corner = agent_corner.reshape(-1, 2)
        cv2.fillPoly(image, [agent_corner], AGENT_COLOR, **CV2_SUB_VALUES)
    return image


def render_road(lane_area_list, image):
    """
    Render the road areas onto an image using lane boundaries.

    Parameters
    ----------
    lane_area_list : list
        Collection of coordinate points defining each lane boundary.

    image : np.ndarray
        The target image on which road areas will be rendered.

    Returns
    -------
    np.ndarray
        Image with road areas rendered.
    """

    for lane_area in lane_area_list:
        lane_area = lane_area.reshape(-1, 2)
        cv2.fillPoly(image, [lane_area], ROAD_COLOR, **CV2_SUB_VALUES)

    return image


def render_lane(lane_area_list, lane_type_list, image):
    """
    Draw lanes on image based on polylines.

    Parameters
    ----------
    lane_area_list : list
        Collection of coordinate points defining each lane.

    lane_type_list : list
        Descriptions of lane types (e.g., 'normal', 'red', 'green', 'yellow').

    image : np.ndarray
        The target image on which lanes will be overlaid.

    Returns
    -------
    np.ndarray
        Image with lanes overlaid.
    """
    for (lane_area, lane_type) in zip(lane_area_list, lane_type_list):
        cv2.polylines(image, lane_area, False, Lane_COLOR[lane_type], **CV2_SUB_VALUES)

    return image
