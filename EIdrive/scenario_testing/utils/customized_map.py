"""
Loading the customized map and create the world.
"""

import os
import sys
import carla


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def create_customized_world(xodr_path, client):
    """
    Load the customized carla world according to the xodr file. The value here is in meters.

    Parameters
    ----------
    xodr_path : str
        Path to the xodr file.

    client : carla.client
        The CARLA world client.
    """
    if os.path.exists(xodr_path):
        with open(xodr_path) as od_file:
            try:
                data = od_file.read()
            except OSError:
                print('xodr file cannot be read.')
                sys.exit()
        print('load map %r.' % os.path.basename(xodr_path))
        vertex_distance = 2.0
        max_road_length = 500.0
        wall_height = 1.0
        extra_width = 0.6
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=False,
                enable_mesh_visibility=True))
        return world
    else:
        print('xodr file not found.')
        return None