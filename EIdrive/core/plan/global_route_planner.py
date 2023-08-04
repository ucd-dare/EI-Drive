"""
The global planner is implemented by A* algorithm in GlobalRoutePlanner.
"""

import math
import numpy as np
import networkx as nx
import carla
from EIdrive.core.plan.local_planner_behavior import RoadOption
from EIdrive.core.common.misc import vector


class GlobalRoutePlanner(object):
    """
    The class offers a high-level route plan.
    The instance of this class is created by passing a reference to a GlobalRoutePlannerDAO object.

    Attributes
    ----------
    dao : carla.dao
        A global plan containing routes from the starting point to the end.
    topology : carla.topology
        The topology graph representing current routes.
    graph : nx.DiGraph
        The node-edge graph depicting the current routes.
    id_map : dict
        A dictionary built with road segment IDs.
    road_id_to_edge : list
        A list mapping road id to corresponding edge in the graph.
    intersection_end_node : int
        The ID of the node situated at the end of the intersection.
    prior_decision : carla.RoadOption
        The last behavioral choice of the ego vehicle.
    """

    def __init__(self, dao):

        self.dao = dao
        self.topology = None
        self.graph = None
        self.id_map = None
        self.road_id_to_edge = None
        self.intersection_end_node = -1
        self.prior_decision = RoadOption.VOID

    def setup(self):
        """
        Performs initial server data lookup for detailed topology
        and builds graph representation of the world map.
        """
        self.topology = self.dao.get_topology()
        self.graph, self.id_map, self.road_id_to_edge = self.construct_graph()
        self.identify_unconnected_segments()
        self.add_lane_change_links()

    def construct_graph(self):
        """
        Constructs a networkx graph representation of topology, which is read from self.topology.

        Args:
            vertex (graph node): Position (x,y,z) in world map.
            entry_vector (graph edge): Unit vector along tangent at entry point.
            exit_vector (graph edge): Unit vector along tangent at exit point.
            net_vector (graph edge): Unit vector of the chord from entry to exit.
            intersection (graph edge): Boolean indicating if the edge belongs to an intersection.

        Returns:
            graph (nx.DiGraph): Networkx graph representing the world map.
            node_map (dict): Mapping from (x,y,z) to node id.
            road_id_to_edge (dict): Mapping from road id and lane id to edge in the graph.
        """
        graph = nx.DiGraph()
        node_map = {}  # Dict with structure {(x,y,z): id, ... }
        road_id_to_edge = {}  # Dict with structure {road_id: {lane_id: edge, ... }, ... }

        # Iterate over each segment in the topology
        for segment in self.topology:
            entry_position, exit_position = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            is_intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            # Add unique nodes to the graph and id map
            for position in [entry_position, exit_position]:
                if position not in node_map:
                    new_id = len(node_map)
                    node_map[position] = new_id
                    graph.add_node(new_id, vertex=position)

            node1, node2 = node_map[entry_position], node_map[exit_position]

            # Map road_id and lane_id to the edge in the graph
            road_id_to_edge.setdefault(road_id, {}).setdefault(section_id, {})[lane_id] = (node1, node2)

            # Convert entry and exit vectors to numpy arrays
            entry_vector = np.array([entry_wp.transform.rotation.get_forward_vector().x,
                                     entry_wp.transform.rotation.get_forward_vector().y,
                                     entry_wp.transform.rotation.get_forward_vector().z])

            exit_vector = np.array([exit_wp.transform.rotation.get_forward_vector().x,
                                    exit_wp.transform.rotation.get_forward_vector().y,
                                    exit_wp.transform.rotation.get_forward_vector().z])

            # Adding edge to the graph
            graph.add_edge(
                node1, node2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=entry_vector,
                exit_vector=exit_vector,
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=is_intersection, type=RoadOption.LANEFOLLOW)

        return graph, node_map, road_id_to_edge

    def identify_unconnected_segments(self):
        """
        This method identifies road segments that have an unconnected end, and
        adds them to the internal graph representation.
        """
        loose_end_count = 0
        resolution = self.dao.get_resolution()

        # Iterate over the topology to find loose ends
        for segment in self.topology:
            exit_waypoint = segment['exit']
            exit_position = segment['exitxyz']
            road_id, section_id, lane_id = exit_waypoint.road_id, exit_waypoint.section_id, exit_waypoint.lane_id

            # Check if the road_id, section_id, and lane_id are already in road_id_to_edge
            if road_id in self.road_id_to_edge and section_id in self.road_id_to_edge[road_id] and lane_id in self.road_id_to_edge[road_id][section_id]:
                continue

            # If not, increment the loose_end_count and add them to road_id_to_edge
            loose_end_count += 1
            self.road_id_to_edge.setdefault(road_id, {}).setdefault(section_id, {})[lane_id] = (self.id_map[exit_position], -1 * loose_end_count)

            # Get the next waypoint
            next_waypoint = exit_waypoint.next(resolution)
            path = []

            # Continue along the path until there are no more waypoints or until the road, section, or lane changes
            while next_waypoint and next_waypoint[0].road_id == road_id and next_waypoint[0].section_id == section_id and next_waypoint[0].lane_id == lane_id:
                path.append(next_waypoint[0])
                next_waypoint = next_waypoint[0].next(resolution)

            if path:
                loose_end_position = (path[-1].transform.location.x, path[-1].transform.location.y, path[-1].transform.location.z)
                self.graph.add_node(-1 * loose_end_count, vertex=loose_end_position)

                # Add edge to the graph
                self.graph.add_edge(
                    self.id_map[exit_position], -1 * loose_end_count,
                    length=len(path) + 1, path=path,
                    entry_waypoint=exit_waypoint, exit_waypoint=path[-1],
                    entry_vector=None, exit_vector=None, net_vector=None,
                    intersection=exit_waypoint.is_junction,
                    type=RoadOption.LANEFOLLOW)

    def localize_closest_road_segment(self, location):
        """
        This function identifies the road segment nearest to the provided position.

        Args:
            location (carla.location): Position to be localized in the graph.

        Returns:
            edge (tuple): A pair of node IDs representing an edge in the graph.
        """
        waypoint = self.dao.get_waypoint(location)
        edge = None
        try:
            edge = \
                self.road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            print(
                "Failed to localize! : ",
                "Road id : ", waypoint.road_id,
                "Section id : ", waypoint.section_id,
                "Lane id : ", waypoint.lane_id,
                "Location : ", waypoint.transform.location.x, waypoint.transform.location.y
                )
        return edge

    def add_lane_change_links(self):
        """
        This method inserts zero-cost links into the topology graph,
        representing the availability of lane changes.
        """

        for segment in self.topology:
            left_lane_available, right_lane_available = False, False

            for waypoint in segment['path']:

                # Ignore junction waypoints
                if not segment['entry'].is_junction:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    # Check if lane change to the right is possible and has not been processed yet
                    if waypoint.right_lane_marking.lane_change & carla.LaneChange.Right and not right_lane_available:
                        next_waypoint = waypoint.get_right_lane()

                        # Verify that the right lane is a driving lane and belongs to the same road
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT
                            next_segment = self.localize_closest_road_segment(next_waypoint.transform.location)

                            # Add an edge to the graph representing the lane change
                            if next_segment is not None:
                                self.graph.add_edge(
                                    self.id_map[segment['entryxyz']], next_segment[0],
                                    entry_waypoint=waypoint, exit_waypoint=next_waypoint,
                                    intersection=False, exit_vector=None, path=[],
                                    length=100, type=next_road_option, change_waypoint=next_waypoint
                                )
                                right_lane_available = True

                    # Check if lane change to the left is possible and has not been processed yet
                    if waypoint.left_lane_marking.lane_change & carla.LaneChange.Left and not left_lane_available:
                        next_waypoint = waypoint.get_left_lane()

                        # Verify that the left lane is a driving lane and belongs to the same road
                        if next_waypoint is not None and next_waypoint.lane_type == carla.LaneType.Driving and waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            next_segment = self.localize_closest_road_segment(next_waypoint.transform.location)

                            # If the next segment exists, add an edge to the graph representing the lane change
                            if next_segment is not None:
                                self.graph.add_edge(
                                    self.id_map[segment['entryxyz']], next_segment[0],
                                    entry_waypoint=waypoint, exit_waypoint=next_waypoint,
                                    intersection=False, exit_vector=None, path=[],
                                    length=100, type=next_road_option, change_waypoint=next_waypoint
                                )
                                left_lane_available = True

                        # If both lane changes are processed for the segment, stop further processing for the segment
                    if left_lane_available and right_lane_available:
                        break

    def heuristic_distance(self, n1, n2):
        """
        Define the heuristic distance for path searching
        """
        l1 = np.array(self.graph.nodes[n1]['vertex'])
        l2 = np.array(self.graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1 - l2)

    def global_route_search(self, origin, destination):
        """
        Executes a global path search from the specified origin to the destination.

        The method uses the A* search algorithm to find the shortest path between the origin
        and destination. The search algorithm utilizes a distance heuristic to optimize the
        search and prefer paths that are closer to the destination.

        Args:
            origin (carla.Location): The starting location for the search. This is the location
            from which the path search will begin.
            destination (carla.Location): The ending location for the search. This is the location
            at which the path search will terminate.

        Returns:
            route (list): A list of node identifiers from the self.graph attribute, representing
            the optimal path from the origin to the destination. The nodes are represented as
            integers.
        """

        start, end = self.localize_closest_road_segment(origin), self.localize_closest_road_segment(destination)

        route = nx.astar_path(
            self.graph, source=start[0], target=end[0],
            heuristic=self.heuristic_distance, weight='length')
        route.append(end[1])
        return route

    def identify_last_intersection(self, starting_index, route):
        """
        This method navigates through the route from a given starting index, locating the last consecutive
        intersection edge if present. This is useful to navigate past small intersection edges when making accurate
        turn decisions during route navigation.

        Args:
            starting_index (int): The starting index in the route from where the search for the intersection edge begins.
            route (list): The list of nodes, represented as integers, which constitute the route.

        Returns:
            last_route_node (int): The node identifier of the last intersection edge in the identified sequence.
            final_intersection_edge (dict): The details of the last intersection edge in the sequence. The details
            include information like edge type, whether it's an intersection, etc.

        """

        final_intersection_edge = None
        last_route_node = None
        for current_node, next_node in [(route[i], route[i + 1])
                                        for i in range(starting_index, len(route) - 1)]:
            potential_edge = self.graph.edges[current_node, next_node]

            # If the current node is the starting node of the route
            if current_node == route[starting_index]:
                final_intersection_edge = potential_edge

            if potential_edge['type'] == RoadOption.LANEFOLLOW and \
                    potential_edge['intersection']:
                final_intersection_edge = potential_edge
                last_route_node = next_node
            else:
                break

        return last_route_node, final_intersection_edge

    def determine_turn(self, route_index, planned_route, angle_threshold=math.radians(35)):
        """
        This method analyses the current position in the route and the succeeding segments to determine the optimal
        turn decision. The decisions could be to keep straight, turn left, or turn right. This function uses the
        characteristics of each road segment, such as the type of the road and intersection, to make this decision.

        Args:
            route_index (int): The current position in the route list.
            planned_route (list): The list of nodes representing the route.
            angle_threshold (float, optional): The threshold value to determine the angle of turn.

        Returns:
            turn_decision (RoadOption): The decision to be made at the current route index. It could be to
            continue straight, turn left, or turn right.
        """

        turn_decision = None
        previous_route_node = planned_route[route_index - 1]
        current_route_node = planned_route[route_index]
        upcoming_route_node = planned_route[route_index + 1]
        upcoming_edge = self.graph.edges[current_route_node, upcoming_route_node]

        if route_index > 0:
            # if the vehicle was previously in an intersection and the next edge is still part of the intersection,
            # the vehicle will continue with its previous decision
            if self.prior_decision != RoadOption.VOID and \
                    self.intersection_end_node > 0 and \
                    self.intersection_end_node != previous_route_node and \
                    upcoming_edge['type'] == RoadOption.LANEFOLLOW and \
                    upcoming_edge['intersection']:
                turn_decision = self.prior_decision
            else:
                self.intersection_end_node = -1
                current_edge = \
                    self.graph.edges[previous_route_node, current_route_node]
                calculate_turn = \
                    current_edge['type'] == RoadOption.LANEFOLLOW and not \
                        current_edge['intersection'] and \
                    upcoming_edge['type'] == RoadOption.LANEFOLLOW and \
                    upcoming_edge['intersection']

                if calculate_turn:
                    last_node, last_intersection_edge = \
                        self.identify_last_intersection(route_index, planned_route)
                    self.intersection_end_node = last_node

                    if last_intersection_edge is not None:
                        upcoming_edge = last_intersection_edge

                    current_vector, upcoming_vector = current_edge['exit_vector'], \
                        upcoming_edge['exit_vector']

                    if current_vector is None or upcoming_vector is None:
                        return upcoming_edge['type']

                    turn_decision = self.determine_direction(route_index, planned_route, current_vector,
                                                             upcoming_vector, angle_threshold)
        else:
            turn_decision = upcoming_edge['type']

        self.prior_decision = turn_decision

        return turn_decision

    def determine_direction(self, index, path, current_vector, next_vector, angle_threshold=math.radians(35)):
        """
        Calculates and determines the direction of movement (left, straight, or right) at a specific point in the path.

        Args:
            index (int): The index in the path where the direction decision is to be made.
            path (list): The list of nodes, represented as integers, which make up the path.
            current_vector (np.array): The vector pointing towards the current node in the path.
            next_vector (np.array): The vector pointing towards the next node in the path.
            angle_threshold (float): The angle (in radians) below which the path is deemed to proceed straight.

        Returns:
            direction_decision (RoadOption): The determined direction of the path at the given index: left,
            straight, or right.
        """

        cross_product_values = []
        current_route_node = path[index]

        # Cross product calculation for all followers of current node except the next node on the path
        for follower in self.graph.successors(current_route_node):
            selected_edge = self.graph.edges[current_route_node, follower]
            if selected_edge['type'] == RoadOption.LANEFOLLOW and follower != path[index + 1]:
                follower_vector = selected_edge['net_vector']
                cross_product_values.append(np.cross(current_vector, follower_vector)[2])

        # Cross product calculation with next node on the path
        cross_product_with_next = np.cross(current_vector, next_vector)[2]

        # Calculating deviation angle
        deviation_angle = math.acos(np.clip(
            np.dot(current_vector, next_vector) / (np.linalg.norm(current_vector) * np.linalg.norm(next_vector)), -1.0,
            1.0))

        # Default to 0 if no cross product values were calculated
        if not cross_product_values:
            cross_product_values.append(0)

        # Determine direction based on deviation and cross product calculations
        if deviation_angle < angle_threshold:
            direction_decision = RoadOption.STRAIGHT
        elif cross_product_values and cross_product_with_next < min(cross_product_values):
            direction_decision = RoadOption.LEFT
        elif cross_product_values and cross_product_with_next > max(cross_product_values):
            direction_decision = RoadOption.RIGHT
        elif cross_product_with_next < 0:
            direction_decision = RoadOption.LEFT
        elif cross_product_with_next > 0:
            direction_decision = RoadOption.RIGHT

        return direction_decision

    def get_nearest_waypoint_index(self, reference_waypoint, waypoints_pool):
        """
        This function determines the closest waypoint in a list of waypoints to the provided reference waypoint.

        Args:
            reference_waypoint (carla.Waypoint): The waypoint to which we are finding the closest point in the waypoints list.
            waypoints_pool (list): The list of waypoints from which to find the closest waypoint to the reference_waypoint.

        Returns:
            nearest_index (int): The index of the closest waypoint in the waypoint_list to the reference waypoint.
        """

        min_distance = float('inf')
        nearest_index = -1
        for i, waypoint in enumerate(waypoints_pool):
            distance = waypoint.transform.location.distance(
                reference_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index

    def route_generate(self, start, end):
        """
        This method generates a list of tuples containing a carla.Waypoint and the corresponding
        RoadOption from the starting location to the destination.

        Args:
            start (carla.Location): The starting location for the route.
            end (carla.Location): The ending location for the route.

        Returns:
            route_waypoint (list): A list of tuples where each tuple contains a carla.Waypoint object and
            the corresponding RoadOption indicating the type of maneuver required at that waypoint.
        """

        route_waypoint = []
        path = self.global_route_search(start, end)
        current_waypoint = self.dao.get_waypoint(start)
        destination_waypoint = self.dao.get_waypoint(end)
        resolution = self.dao.get_resolution()

        for i in range(len(path) - 1):
            maneuver = self.determine_turn(i, path)
            edge = self.graph.edges[path[i], path[i + 1]]
            waypoint_sequence = []

            if edge['type'] != RoadOption.LANEFOLLOW and edge['type'] != RoadOption.VOID:
                route_waypoint.append((current_waypoint, maneuver))
                exit_wp = edge['exit_waypoint']

                # Get the next edge in the graph that contains the exit waypoint.
                n1, n2 = self.road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge = self.graph.edges[n1, n2]

                # If the next edge has intermediate waypoints, find the closest one to the current waypoint.
                if next_edge['path']:
                    closest_index = self.get_nearest_waypoint_index(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path']) - 1, closest_index + 5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_waypoint.append((current_waypoint, maneuver))

            else:
                waypoint_sequence = waypoint_sequence + [edge['entry_waypoint']] + edge['path'] + [
                    edge['exit_waypoint']]
                closest_index = self.get_nearest_waypoint_index(current_waypoint, waypoint_sequence)
                for waypoint in waypoint_sequence[closest_index:]:
                    current_waypoint = waypoint
                    route_waypoint.append((current_waypoint, maneuver))

                    # Stop adding waypoints if the current one is close enough to the destination.
                    if len(path) - i <= 2 and waypoint.transform.location.distance(end) < 2 * resolution:
                        break
                    elif len(path) - i <= 2 and current_waypoint.road_id == destination_waypoint.road_id and \
                            current_waypoint.section_id == destination_waypoint.section_id and \
                            current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self.get_nearest_waypoint_index(destination_waypoint, waypoint_sequence)
                        if closest_index > destination_index:
                            break

        return route_waypoint
