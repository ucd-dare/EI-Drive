"""
Build and render minimap
"""

import math
import uuid

import cv2
import carla
import numpy as np
from matplotlib.path import Path
from shapely.geometry import Polygon

from EIdrive.core.sensing.perception.sensor_transformation import \
    world_to_sensor
from EIdrive.core.map.map_utils import \
    lateral_shift, convert_locations_to_array, waypoints_to_array, traffic_light_state_to_string
from EIdrive.core.map.map_drawing import \
    cv2_subpixel, render_agent, render_road, render_lane


class GameMap(object):
    """
    Manages a High Definition (HD) Map. The class process and visualize information about lanes, crosswalks,
    traffic lights, and other critical mapping elements.

    Parameters
    ----------
    vehicle : Carla.vehicle
        The ego vehicle in the simulation.

    world_map : Carla.Map
        The map being used in the Carla simulator.

    config : dict
        Configuration parameters dictating various properties of the map.

    Attributes
    ----------
    world : carla.world
        Instance of the Carla world containing the simulation elements.

    agent_id : int
        Identifier for the ego vehicle.

    center : carla.Transform or None
        Represents the center pose of the rasterized map. None if not set.

    activate : bool
        Determines if certain features are activated.

    visualize : bool
        Flag for visualizations.

    pixels_per_meter : float
        Pixel density relative to actual measurements.

    meter_per_pixel : float
        Physical size each pixel represents (inverse of `pixels_per_meter`).

    raster_size : np.array
        Dimensions of the rasterized map in pixels.

    raster_radius : float
        Radius of the usable part of the rasterized map in meters.

    topology : list
        List of starting waypoints in the HD Map, sorted by altitude.

    lane_info : dict
        Contains detailed information about the lanes.

    crosswalk_info : dict
        Contains detailed information about the crosswalks.

    traffic_light_info : dict
        Contains detailed information about the traffic lights.

    bound_info : dict
        Boundary information for lanes and crosswalks for efficient data access.

    lane_sample_resolution : int
        Resolution used for sampling when drawing the lanes.

    dynamic_bev : np.array
        Bird's Eye View (BEV) map that incorporates dynamic objects like vehicles.

    static_bev : np.array
        BEV map focusing on static elements like roads and lanes.

    vis_bev : np.array
        Comprehensive BEV map used for visualizations.
    """

    def __init__(self, vehicle, world_map, config):
        # Initializing attributes
        self.world = vehicle.get_world()
        self.agent_id = vehicle.id
        self.world_map = world_map
        self.center = None

        self.activate = config['activate']
        self.visualize = config['visualize']
        self.pixels_per_meter = config['pixels_per_meter']
        self.meter_per_pixel = 1 / self.pixels_per_meter
        self.raster_size = np.array([config['raster_size'][0],
                                     config['raster_size'][1]])
        self.lane_sample_resolution = config['lane_sample_resolution']

        self.raster_radius = float(np.linalg.norm(self.raster_size * np.array(
                                                      [self.meter_per_pixel,
                                                       self.meter_per_pixel]))) / 2

        # Extracting and sorting all starting waypoints from the HD Map
        topology = [x[0] for x in world_map.get_topology()]
        self.topology = sorted(topology, key=lambda w: w.transform.location.z)

        # Initializing dictionaries for map elements
        self.lane_info = {}
        self.crosswalk_info = {}
        self.traffic_light_info = {}
        self.bound_info = {'lanes': {},
                           'crosswalks': {}}

        # Generate information for different map elements
        self.generate_traffic_light_data(self.world)
        self.generate_lane_cross_info()

        # Initializing BEV maps
        self.dynamic_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.static_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

    def set_center(self, ego_pose):
        """
        Set the ego pose as the map center.

        Parameters
        ----------
        ego_pose : carla.Transform
        """
        self.center = ego_pose

    def rasterize_bev_map(self):
        """
        Rasterize and Visualize the bev map.
        """
        if not self.activate:
            return
        self.render_static_agents()
        self.render_dynamic_agents()
        if self.visualize:
            cv2.imshow('the bev map of agent %s' % self.agent_id, self.vis_bev)
            cv2.waitKey(1)

    @staticmethod
    def compute_lane_bounds(left_boundary, right_boundary):
        """
        Computes the bounding box for a given lane defined by its left and right boundaries.

        Parameters
        ----------
        left_boundary : np.array
            Array representing the left boundary of the lane. Shape: (n, 3)

        right_boundary : np.array
            Array representing the right boundary of the lane. Shape: (n, 3)

        Returns
        -------
        bounding_box : np.array
            Array containing the minimum and maximum coordinates that enclose the lane.
        """

        # Calculate the bounds for the given lane
        x_min = min(np.min(left_boundary[:, 0]), np.min(right_boundary[:, 0]))
        y_min = min(np.min(left_boundary[:, 1]), np.min(right_boundary[:, 1]))
        x_max = max(np.max(left_boundary[:, 0]), np.max(right_boundary[:, 0]))
        y_max = max(np.max(left_boundary[:, 1]), np.max(right_boundary[:, 1]))

        bounding_box = np.asarray([[[x_min, y_min], [x_max, y_max]]])

        return bounding_box

    def filter_agents_within_radius(self, radius, all_agents):
        """
        Filters and returns agents within a specified radius from the center.

        Parameters
        ----------
        range_radius : float
            The range in meters within which to filter agents.

        all_agents : dict
            Dictionary containing details of all agents.

        Returns
        -------
        agents_within_range : dict
            A dictionary containing agents that are within the specified radius from the center.
        """

        agents_within_range = {}

        # Retrieve the center's coordinates
        center_coords = [self.center.location.x, self.center.location.y]

        for agent_key, agent_details in all_agents.items():
            agent_location = agent_details['location']
            euclidean_distance = math.sqrt((agent_location[0] - center_coords[0]) ** 2 +
                                           (agent_location[1] - center_coords[1]) ** 2)

            if euclidean_distance < radius:
                agents_within_range[agent_key] = agent_details

        return agents_within_range

    def indices_in_bounds(self,
                          bounding_boxes: np.ndarray,
                          box_half_extent: float) -> np.ndarray:
        """
        Obtain indices of elements where given bounding boxes intersect with a square bounding box
        centered around the `self.center` and having a side length of 2 * box_half_extent.

        Parameters
        ----------
        bounding_boxes : np.ndarray
            Array of shape Nx2x2 representing bounding boxes as [[x_min, y_min], [x_max, y_max]].

        box_half_extent : float
            Half the side length of the square bounding box centered around `self.center`.

        Returns
        -------
        np.ndarray
            Indices of elements within the specified bounding box around `self.center`.
        """

        center_x, center_y = self.center.location.x, self.center.location.y

        x_min_within = center_x > bounding_boxes[:, 0, 0] - box_half_extent
        y_min_within = center_y > bounding_boxes[:, 0, 1] - box_half_extent
        x_max_within = center_x < bounding_boxes[:, 1, 0] + box_half_extent
        y_max_within = center_y < bounding_boxes[:, 1, 1] + box_half_extent

        return np.nonzero(x_min_within & y_min_within & x_max_within & y_max_within)[0]

    def get_associated_traffic_light(self, lane_midline):
        """
        Identifies the traffic light that influences a given lane based on waypoints.

        Parameters
        ----------
        lane_midline : np.ndarray
            Represents the centerline waypoints of the lane.

        Returns
        -------
        associated_traffic_light_id : str
            The ID of the traffic light that is associated with the lane.
        """

        associated_traffic_light_id = ''

        # Iterate over all traffic lights to check influence on the lane
        for tl_id, tl_details in self.traffic_light_info.items():
            trigger_area = tl_details['corners']

            # Create a path object for efficient geometric computations
            trigger_path = Path(trigger_area.boundary)

            # Verify if any waypoint of the lane's centerline lies within the trigger area
            check_result = trigger_path.contains_points(lane_midline[:, :2])

            if check_result.any():
                associated_traffic_light_id = tl_id
                break

        return associated_traffic_light_id

    def generate_lane_cross_info(self):
        """
        Extracts lane and crosswalk information from the topology and transfer them
        into respective dictionaries using world coordinates.
        """

        # Lists to store lane and crosswalk IDs
        lane_ids = []
        crosswalk_ids = []

        # Arrays to store the boundaries of lanes and crosswalks
        lane_boundaries = np.empty((0, 2, 2), dtype=np.float)
        crosswalk_boundaries = np.empty((0, 2, 2), dtype=np.float)

        # Iterate through all waypoints to extract lane information
        for waypoint in self.topology:

            # Generate a unique ID for each lane
            current_lane_id = uuid.uuid4().hex[:6].upper()
            lane_ids.append(current_lane_id)

            collected_waypoints = [waypoint]
            next_waypoint = waypoint.next(self.lane_sample_resolution)[0]

            # Traverse waypoints in the same lane
            while next_waypoint.road_id == waypoint.road_id and \
                    next_waypoint.lane_id == waypoint.lane_id:
                collected_waypoints.append(next_waypoint)

                # Check to ensure the next waypoint exists
                if len(next_waypoint.next(self.lane_sample_resolution)) > 0:
                    next_waypoint = next_waypoint.next(self.lane_sample_resolution)[0]
                else:
                    break

            # Calculate left and right lane markings based on the waypoint's centerline
            left_lane_boundary = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in collected_waypoints]
            right_lane_boundary = [lateral_shift(w.transform, w.lane_width * 0.5) for w in collected_waypoints]

            # Convert the list of Carla locations to numpy arrays
            left_lane_boundary = convert_locations_to_array(left_lane_boundary)
            right_lane_boundary = convert_locations_to_array(right_lane_boundary)
            center_line = waypoints_to_array(collected_waypoints)

            # Determine boundary and append to lane boundaries
            current_boundary = self.compute_lane_bounds(left_lane_boundary, right_lane_boundary)
            lane_boundaries = np.append(lane_boundaries, current_boundary, axis=0)

            # Link lanes to their respective traffic lights
            traffic_light_id = self.get_associated_traffic_light(center_line)

            self.lane_info.update({
                current_lane_id: {
                    'xyz_left': left_lane_boundary,
                    'xyz_right': right_lane_boundary,
                    'xyz_mid': center_line,
                    'tl_id': traffic_light_id
                }
            })

            # Update boundary information
            self.bound_info['lanes']['ids'] = lane_ids
            self.bound_info['lanes']['bounds'] = lane_boundaries
            self.bound_info['crosswalks']['ids'] = crosswalk_ids
            self.bound_info['crosswalks']['bounds'] = crosswalk_boundaries

    def generate_traffic_light_data(self, world):
        """
        Generate information about traffic lights in the given world's coordinate system.

        Parameters
        ----------
        environment : carla.world
            Instance of the Carla simulator environment.

        Returns
        -------
        dict:
            A dictionary containing detailed information about each traffic light.
        """

        # Fetch all traffic light actors from the environment
        traffic_light_actors = world.get_actors().filter('traffic.traffic_light*')
        traffic_light_data = {}

        # Iterate over each traffic light actor to extract relevant information
        for tl in traffic_light_actors:
            unique_tl_id = uuid.uuid4().hex[:4].upper()

            primary_transform = tl.get_transform()
            primary_rotation_yaw = primary_transform.rotation.yaw

            # Compute the area location where vehicles are expected to stop
            stop_area_location = primary_transform.transform(tl.trigger_volume.location)
            stop_area_transform = carla.Transform(stop_area_location, carla.Rotation(yaw=primary_rotation_yaw))

            # Define extents of the bounding box
            bounding_box_extent = tl.trigger_volume.extent
            bounding_box_extent.y += 0.5
            bounding_box_corners = np.array([
                [-bounding_box_extent.x, -bounding_box_extent.y],
                [bounding_box_extent.x, -bounding_box_extent.y],
                [bounding_box_extent.x, bounding_box_extent.y],
                [-bounding_box_extent.x, bounding_box_extent.y]
            ])

            # Adjust corners based on transformed location
            for idx in range(bounding_box_corners.shape[0]):
                transformed_location = stop_area_transform.transform(
                    carla.Location(bounding_box_corners[idx][0], bounding_box_corners[idx][1]))
                bounding_box_corners[idx, 0] = transformed_location.x
                bounding_box_corners[idx, 1] = transformed_location.y

            # Convert corners to polygon using shapely library
            polygon_representation = Polygon(bounding_box_corners)

            # Update the dictionary with the extracted information
            traffic_light_data[unique_tl_id] = {
                'actor': tl,
                'corners': polygon_representation,
                'base_rot': primary_rotation_yaw,
                'base_transform': primary_transform
            }

        self.traffic_light_info.update(traffic_light_data)

    def compute_lane_polygon(self, left_markings, right_markings):
        """
        Constructs a lane polygon based on the map's central rasterization frame.

        Parameters
        ----------
        left_markings : np.ndarray
            Coordinates for the left edge of the lane, with a shape of (n, 3).
        right_markings : np.ndarray
            Coordinates for the right edge of the lane, with a shape of (n, 3).

        Returns
        -------
        lane_polygon : np.ndarray
            A polygon formed by merging the left and right edges of the lane.
        """
        lane_polygon = np.zeros((2, left_markings.shape[0], 2))

        # Homogenize coordinates for transformation
        left_markings = left_markings.T
        left_markings = np.r_[
            left_markings, [np.ones(left_markings.shape[1])]]
        right_markings = right_markings.T
        right_markings = np.r_[
            right_markings, [np.ones(right_markings.shape[1])]]

        # Convert world coordinates to the ego vehicle's frame
        left_markings = world_to_sensor(left_markings, self.center).T
        right_markings = world_to_sensor(right_markings, self.center).T

        # Extract x, y from the transformed coordinates and invert y-axis for right markings
        lane_polygon[0] = left_markings[:, :2]
        lane_polygon[1] = right_markings[::-1, :2]

        # Swap x and y coordinates
        lane_polygon = lane_polygon[..., ::-1]

        # Invert the y coordinate
        lane_polygon[:, :, 1] = -lane_polygon[:, :, 1]

        # Adjust coordinates based on pixel density and raster size
        lane_polygon[:, :, 0] = lane_polygon[:, :, 0] * self.pixels_per_meter + self.raster_size[0] // 2
        lane_polygon[:, :, 1] = lane_polygon[:, :, 1] * self.pixels_per_meter + self.raster_size[1] // 2

        # Refine the polygon for higher accuracy
        lane_polygon = cv2_subpixel(lane_polygon)

        return lane_polygon

    def generate_agent_area(self, bounding_box_corners):
        """
        Transforms the agent's bounding box corners from world coordinates
        to rasterization frame coordinates.

        Parameters
        ----------
        bounding_box_corners : list
            List of four corners of the agent's bounding box in the world coordinate frame.

        Returns
        -------
        Rasterized coordinates of the agent's bounding box corners.
        """
        # Convert the list of corners to a (4, 3) numpy array
        bounding_box_corners = np.array(bounding_box_corners)

        # Convert from world frame to the ego vehicle's frame
        bounding_box_corners = bounding_box_corners.T
        bounding_box_corners = np.r_[bounding_box_corners, [np.ones(bounding_box_corners.shape[1])]]
        bounding_box_corners = world_to_sensor(bounding_box_corners, self.center).T
        bounding_box_corners = bounding_box_corners[:, :2]

        # Swap x and y coordinates
        bounding_box_corners = bounding_box_corners[..., ::-1]
        # y revert
        bounding_box_corners[:, 1] = -bounding_box_corners[:, 1]

        bounding_box_corners[:, 0] = bounding_box_corners[:, 0] * self.pixels_per_meter + self.raster_size[0] // 2
        bounding_box_corners[:, 1] = bounding_box_corners[:, 1] * self.pixels_per_meter + self.raster_size[1] // 2

        # Refine the bounding box corners for higher precision
        corner_area = cv2_subpixel(bounding_box_corners[:, :2])

        return corner_area

    def fetch_dynamic_agents_data(self):
        """
        Retrieve information of all dynamic agents directly from the server and
        organize it into a dictionary.

        Returns
        -------
        A dictionary containing details of all dynamic agents present in the carla world.
        """

        vehicle_actors = self.world.get_actors().filter('vehicle.*')
        agents_data = {}

        for agent in vehicle_actors:
            agent_key = agent.id

            agent_current_transform = agent.get_transform()
            agent_position = [
                agent_current_transform.location.x,
                agent_current_transform.location.y,
                agent_current_transform.location.z,
            ]

            agent_orientation = agent_current_transform.rotation.yaw

            # Determine the four bounding box corners
            bounding_extent = agent.bounding_box.extent
            bounding_corners = [
                carla.Location(x=-bounding_extent.x, y=-bounding_extent.y),
                carla.Location(x=-bounding_extent.x, y=bounding_extent.y),
                carla.Location(x=bounding_extent.x, y=bounding_extent.y),
                carla.Location(x=bounding_extent.x, y=-bounding_extent.y)
            ]
            # Convert bounding corners from the agent's coordinate frame to the world frame
            agent_current_transform.transform(bounding_corners)
            formatted_corners = [[corner.x, corner.y, corner.z] for corner in bounding_corners]

            agents_data[agent_key] = {
                'location': agent_position,
                'yaw': agent_orientation,
                'corners': formatted_corners
            }

        return agents_data

    def render_dynamic_agents(self):
        """
        Visualize the dynamic elements on a bird's-eye view (BEV) map.
        """

        # Initialize an empty image canvas for agent visualization
        self.dynamic_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

        # Calculate the raster's effective radius based on its size
        effective_radius = \
            0.5 * np.linalg.norm(self.raster_size *
                                 np.array([self.meter_per_pixel, self.meter_per_pixel]))

        # Fetch data of all dynamic agents in the world
        all_agents_data = self.fetch_dynamic_agents_data()

        # Retain only those agents within the effective radius
        relevant_agents = self.filter_agents_within_radius(effective_radius,
                                                           all_agents_data)

        formatted_corners = []
        for agent_key, agent_details in relevant_agents.items():
            rasterized_corners = self.generate_agent_area(agent_details['corners'])
            formatted_corners.append(rasterized_corners)

        # Draw agents on the canvas
        self.dynamic_bev = render_agent(formatted_corners, self.dynamic_bev)
        self.vis_bev = render_agent(formatted_corners, self.vis_bev)

    def render_static_agents(self):
        """
        Visualize the static elements on a bird's-eye view (BEV) map.
        """

        # Create empty canvases for static BEV and visualization
        self.static_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

        # Compute the effective radius for filtering elements
        effective_radius = \
            float(np.linalg.norm(self.raster_size *
                                 np.array([self.meter_per_pixel,
                                           self.meter_per_pixel]))) / 2

        # Fetch lanes that are within the determined bounds
        relevant_lane_indices = self.indices_in_bounds(self.bound_info['lanes']['bounds'],
                                                       effective_radius)

        lane_areas = []
        lane_status_list = []

        for idx, lane_id in enumerate(relevant_lane_indices):
            specific_lane_id = self.bound_info['lanes']['ids'][lane_id]
            lane_details = self.lane_info[specific_lane_id]
            left_points, right_points = lane_details['xyz_left'], lane_details['xyz_right']

            # Calculate the polygon for the lane
            lane_polygon = self.compute_lane_polygon(left_points, right_points)
            lane_areas.append(lane_polygon)

            # Get the status of the associated traffic light
            traffic_light_id = lane_details['tl_id']
            if traffic_light_id:
                tl_actor = self.traffic_light_info[traffic_light_id]['actor']
                tl_status = traffic_light_state_to_string(tl_actor.get_state())
                lane_status_list.append(tl_status)
            else:
                lane_status_list.append('normal')

        # Draw roads and lanes on the static BEV and visualization
        self.static_bev = render_road(lane_areas, self.static_bev)
        self.static_bev = render_lane(lane_areas, lane_status_list, self.static_bev)

        self.vis_bev = render_road(lane_areas, self.vis_bev)
        self.vis_bev = render_lane(lane_areas, lane_status_list, self.vis_bev)

        # Adjust color for display
        self.vis_bev = cv2.cvtColor(self.vis_bev, cv2.COLOR_RGB2BGR)

    def destroy(self):
        cv2.destroyAllWindows()
