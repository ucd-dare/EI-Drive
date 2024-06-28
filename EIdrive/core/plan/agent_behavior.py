"""
Behavior planning module by rule-base method.
"""

import sys
import numpy as np
import carla
import importlib

from collections import deque
from EIdrive.core.basic.auxiliary import get_speed, positive, distance_angle_to_target
from EIdrive.core.plan.collision_detect import CollisionDetector
from EIdrive.core.plan.local_trajectory_planner import LocalPlanner
from EIdrive.core.plan.global_route_planner import GlobalRoutePlanner


class AgentBehavior(object):
    """
    A modulized version of carla AgentBehavior to deal with vehicle behavior, especially the trajectory planning with
    rule-based method.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    carla_map : carla.map
        The carla HD map for simulation world.

    behavior_config : dict
        The configuration dictionary of the localization module.

    Attributes
    ----------
    vehicle_pos : carla.position
        Position of the ego vehicle.

    vehicle_speed : float
        Speed of the ego vehicle.

    map : carla.map
        The HD map of the current simulation world.

    max_speed : float
        The current speed limit of the ego vehicles.

    break_distance : float
        The current distance needed for ego vehicle to reach a steady stop.

    _collision_detector : CollisionDetetor
        A collision check class to estimate the collision with front obstacle.

    ignore_traffic_light : boolean
        Boolean indicator of whether to ignore traffic light.

    overtake_allowed : boolean
        Boolean indicator of whether to allow overtake.

    local_planner : LocalPlanner
        A carla local planner class for behavior planning.

    lane_change_allowed : boolean
        Boolean indicator of whether the lane change is allowed.

    safe_vehicles : list
        The white list contains all position of target
        platoon member for joining.

    obstacle_vehicles : list
        The list contains all obstacle vehicles nearby.

    objects : dict
        The dictionary that contains all kinds of objects nearby.
    """

    def __init__(self, vehicle, carla_map, behavior_config, control_config):

        self.vehicle = vehicle

        # Position and speed from localization module
        self.vehicle_pos = None
        self.vehicle_speed = 0.0
        self.map = carla_map

        # Speed related configurations
        self.max_speed = behavior_config['max_speed']
        self.tailgate_speed = behavior_config['tailgate_speed']
        self.max_speed_margin = behavior_config['max_speed_margin']
        self.speed_decrease = behavior_config['speed_decrease']

        # Safety related configurations
        self.safety_time = behavior_config['safety_time']
        self.emergency_param = behavior_config['emergency_param']
        self.break_distance = 0
        self.ttc = 1000

        # Initialize collision detector
        self._collision_detector = CollisionDetector(
            time_ahead=behavior_config['collision_time_ahead'])
        self.ignore_traffic_light = behavior_config['ignore_traffic_light']
        self.overtake_allowed = behavior_config['overtake_allowed']
        self.overtake_allowed_origin = behavior_config['overtake_allowed']
        self.overtake_counter = 0

        # Route planning related
        self.global_planner = None
        self.start_waypoint = None
        self.end_waypoint = None
        self.sampling_resolution = behavior_config['sample_resolution']

        # Intersection handling
        self.light_state = "Red"
        self.light_id_to_ignore = -1
        self.stop_sign_wait_count = 0

        # Initialize trajectory planning
        self.local_planner = LocalPlanner(self, carla_map, behavior_config['local_planner'])

        # Initialize controller
        controller_type = control_config['type']
        controller = getattr(importlib.import_module("EIdrive.core.actuation.%s" % controller_type), 'Controller')
        self.controller = controller(control_config['args'])

        # Behavior indicator related
        self.car_following_flag = False
        # lane change allowed flag
        self.lane_change_allowed = True
        # destination temp push flag
        self.destination_push_flag = 0

        self.white_list = []
        self.obstacle_vehicles = []
        self.objects = {}

    def update_information(self, current_position, current_speed, detected_objects):
        """
        Update agent state from the perception and localization module.

        Parameters:
        ----------
        current_position : carla.Transform
            Current location of the vehicle from the localization module.

        current_speed : float
            The vehicle's speed in km/h.

        detected_objects : dict
            Detected entities from the perception module, encompassing vehicles and other objects.
        """
        # update vehicle's movement information
        self.vehicle_speed = current_speed
        self.vehicle_pos = current_position
        self.break_distance = self.vehicle_speed / 3.6 * self.emergency_param

        # update the localization info to local planner
        self.get_local_planner().update_vehicle_info(current_position, current_speed)

        # update the localization info to the controller
        self.controller.update_info(current_position, current_speed)

        self.objects = detected_objects

        # For current version, consider vehicles only.
        obstacle_vehicles = detected_objects['vehicles']
        self.obstacle_vehicles = self.white_list_filter(obstacle_vehicles)
        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

    def white_list_filter(self, detected_obstacles):
        """
        Filters out obstacles that are present in the whitelist.
        The whitelist consists of the positions of target platoon members for joining.

        Parameters
        ----------
        detected_obstacles : list
            A list of detected carla.Vehicle or DynamicObstacle instances.

        Returns
        -------
        filtered_obstacles : list
            List of obstacles after filtering out whitelisted ones.
        """

        filtered_obstacles = []

        for obstacle in detected_obstacles:
            is_whitelisted = False

            obstacle_location = obstacle.get_location()
            obstacle_waypoint = self.map.get_waypoint(obstacle_location)
            obstacle_lane_id = obstacle_waypoint.lane_id

            for vehicle_member in self.white_list:
                member_position = vehicle_member.v2x_manager.get_ego_pos()
                member_waypoint = self.map.get_waypoint(member_position.location)
                member_lane_id = member_waypoint.lane_id

                # Compare lane IDs and positions
                if (obstacle_lane_id == member_lane_id) and \
                   (abs(member_position.location.x - obstacle_location.x) <= 3.0) and \
                   (abs(member_position.location.y - obstacle_location.y) <= 3.0):
                    is_whitelisted = True
                    break

            if not is_whitelisted:
                filtered_obstacles.append(obstacle)

        return filtered_obstacles

    def set_local_planner(
            self,
            origin,
            destination,
            clear_waypoints=False,
            reset_end=True,
            clear_history=False):
        """
        Establish a route of waypoints from the vehicle's current position to the specified destination.
        It utilizes the global router to derive this route.

        Parameters
        ----------
        origin : carla.location
            Starting location for the route.

        destination : carla.location
            Ending location for the route.

        clear_waypoints : boolean
            If True, clears the existing waypoint queue.

        reset_end : boolean
            If True, resets the ending waypoint of the route.

        clear_history : boolean
            If True, clears the waypoint history.
        """

        # Clear the existing waypoints and trajectories if needed
        if clear_waypoints:
            self.local_planner.get_waypoints_queue().clear()
            self.local_planner.get_trajectory().clear()
            self.local_planner.get_global_route().clear()
        if clear_history:
            self.local_planner.get_history_buffer().clear()

        # Retrieve the starting waypoint
        self.start_waypoint = self.map.get_waypoint(origin)

        # Ensure the start waypoint is situated behind the vehicle
        if self.vehicle_pos:
            current_location = self.vehicle_pos.location
            current_yaw = self.vehicle_pos.rotation.yaw
            _, yaw_angle = distance_angle_to_target(self.start_waypoint.transform.location, current_location, current_yaw)

            while yaw_angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, yaw_angle = distance_angle_to_target(self.start_waypoint.transform.location, current_location, current_yaw)

        end_waypoint = self.map.get_waypoint(destination)
        if reset_end:
            self.end_waypoint = end_waypoint

        # Compute the global route from start to end
        global_route = self.generate_global_route(self.start_waypoint, end_waypoint)

        # Set this global route for the local planner
        self.local_planner.update_global_route(global_route, clear_waypoints)

    def get_local_planner(self):
        """
        return the local planner
        """
        return self.local_planner

    def generate_global_route(self, start_waypoint, end_waypoint):
        """
        Generate an optimal route between the provided start and end waypoints using a global planner.

        Parameters
        ----------
        start_waypoint : carla.waypoint
            The starting waypoint for the route.

        end_waypoint : carla.waypoint
            The destination waypoint for the route.
        """

        # Initialize global router if it's not set
        if self.global_planner is None:
            world = self.vehicle.get_world()
            route_planner = GlobalRoutePlanner(world.get_map(), sampling_resolution=self.sampling_resolution)
            route_planner.setup()
            self.global_planner = route_planner

        # Fetch route using global planner
        global_route = self.global_planner.route_generate(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return global_route

    def check_collision(self, path_x, path_y, path_yaw, current_waypoint, check_adjacent_lane=False):
        """
        Monitors the potential collisions along the planned path with obstacles.

        Parameters
        ----------
        path_x : float
            X coordinates of the planned path.

        path_y : float
            Y coordinates of the planned path.

        path_yaw : float
            Yaw angle of the planned path.

        current_waypoint : carla.waypoint
            Current waypoint of the agent.

        check_adjacent_lane : boolean, optional
            Flag to determine if adjacent lanes should be checked for collisions.

        Returns
        -------
        is_collision_detected: bool
            True if there's a potential collision, False otherwise.

        colliding_vehicle: carla.Vehicle
            The vehicle causing the potential collision, or None if there's no collision.

        min_distance: float
            Minimum distance to the colliding vehicle.
        """
        is_collision_detected = False
        min_distance = float('inf')
        colliding_vehicle = None

        for vehicle in self.obstacle_vehicles:
            collision_free = self._collision_detector.circle_collision_check(
                path_x, path_y, path_yaw, vehicle, self.vehicle_speed / 3.6, self.map,
                adjacent_check=check_adjacent_lane)

            if not collision_free:
                is_collision_detected = True
                # Adjusting for typical vehicle length
                distance_to_vehicle = max(vehicle.get_location().distance(current_waypoint.transform.location) - 3, 0)

                if distance_to_vehicle < min_distance:
                    min_distance = distance_to_vehicle
                    colliding_vehicle = vehicle
        return is_collision_detected, colliding_vehicle, min_distance

    def overtake_management(self, obstacle_vehicle):
        """
        Determines if the vehicle can overtake on the left or right, and sets the destination accordingly.

        Parameters
        ----------
        obstacle_vehicle : carla.vehicle
            The vehicle obstructing the path.

        Returns
        -------
        hazardous_situation: bool
            True if the vehicle is in a hazardous situation, otherwise False.
        """
        # Determine the location and waypoints related to the obstacle vehicle
        obstacle_location = obstacle_vehicle.get_location()
        obstacle_waypoint = self.map.get_waypoint(obstacle_location)

        # Determine lane change possibilities
        can_turn_left = obstacle_waypoint.left_lane_marking.lane_change
        can_turn_right = obstacle_waypoint.right_lane_marking.lane_change

        left_adjacent_waypoint = obstacle_waypoint.get_left_lane()
        right_adjacent_waypoint = obstacle_waypoint.get_right_lane()

        # Check for possible right overtaking
        if (can_turn_right in [carla.LaneChange.Right, carla.LaneChange.Both]) and \
           right_adjacent_waypoint and \
           obstacle_waypoint.lane_id * right_adjacent_waypoint.lane_id > 0 and \
           right_adjacent_waypoint.lane_type == carla.LaneType.Driving:

            path_x, path_y, path_yaw = self._collision_detector.check_adjacent_lane_collision(
                ego_loc=self.vehicle_pos.location,
                target_wpt=right_adjacent_waypoint,
                overtake=True)

            hazardous_situation, _, _ = self.check_collision(
                path_x, path_y, path_yaw,
                self.map.get_waypoint(self.vehicle_pos.location),
                check_adjacent_lane=True)

            if not hazardous_situation:
                print("Performing right overtaking")
                self.overtake_counter = 100
                next_waypoints = right_adjacent_waypoint.next(self.vehicle_speed / 3.6 * 6)

                if not next_waypoints:
                    return True

                target_waypoint = next_waypoints[0]
                right_adjacent_waypoint = right_adjacent_waypoint.next(5)[0]

                self.set_local_planner(
                    right_adjacent_waypoint.transform.location,
                    target_waypoint.transform.location,
                    clear_waypoints=True,
                    reset_end=False)
                return hazardous_situation

        # Check for possible left overtaking
        if (can_turn_left in [carla.LaneChange.Left, carla.LaneChange.Both]) and \
           left_adjacent_waypoint and \
           obstacle_waypoint.lane_id * left_adjacent_waypoint.lane_id > 0 and \
           left_adjacent_waypoint.lane_type == carla.LaneType.Driving:

            path_x, path_y, path_yaw = self._collision_detector.check_adjacent_lane_collision(
                ego_loc=self.vehicle_pos.location,
                target_wpt=left_adjacent_waypoint,
                overtake=True)

            hazardous_situation, _, _ = self.check_collision(
                path_x, path_y, path_yaw,
                self.map.get_waypoint(self.vehicle_pos.location),
                check_adjacent_lane=True)

            if not hazardous_situation:
                print("Performing left overtaking")
                self.overtake_counter = 100
                next_waypoints = left_adjacent_waypoint.next(self.vehicle_speed / 3.6 * 6)

                if not next_waypoints:
                    return True

                target_waypoint = next_waypoints[0]
                left_adjacent_waypoint = left_adjacent_waypoint.next(5)[0]

                self.set_local_planner(
                    left_adjacent_waypoint.transform.location,
                    target_waypoint.transform.location,
                    clear_waypoints=True,
                    reset_end=False)
                return hazardous_situation


        return True

    def check_lane_change(self):
        """
        Evaluate if a lane change presents any potential hazards.

        Returns
        -------
        hazardous_situation: bool
            True if the lane change is risky, otherwise False.
        """
        # Determine the current lane of the vehicle
        current_waypoint = self.map.get_waypoint(self.vehicle_pos.location)
        current_lane_id = current_waypoint.lane_id
        adjacent_lane_waypoint = None

        # Identify the nearest waypoint in the neighboring lane
        for waypoint_data in self.get_local_planner().get_global_route():
            if waypoint_data[0].lane_id != current_lane_id:
                adjacent_lane_waypoint = waypoint_data[0]
                break

        # If no adjacent lane waypoint found, consider lane change as risky
        if not adjacent_lane_waypoint:
            return True

        # Check for potential collisions during a lane change
        path_x, path_y, path_yaw = self._collision_detector.check_adjacent_lane_collision(
            ego_loc=self.vehicle_pos.location,
            target_wpt=adjacent_lane_waypoint,
            overtake=False)

        hazardous_situation, _, _ = self.check_collision(
            path_x, path_y, path_yaw,
            self.map.get_waypoint(self.vehicle_pos.location),
            check_adjacent_lane=True)

        return hazardous_situation

    def manage_car_following(self, lead_vehicle, lead_distance, desired_speed=None):
        """
        Handles the car-following behaviors when there is a vehicle in front.

        Parameters
        ----------
        lead_vehicle : carla.vehicle
            The vehicle in front which our vehicle follows.

        lead_distance : float
            Distance to the leading vehicle.

        desired_speed : float, optional
            Desired speed for following. Defaults to max speed minus margin.

        Returns
        -------
        adjusted_speed : float
            The recommended speed to maintain safe following distance.
        """
        # Default to a reduced maximum speed if none is provided
        if not desired_speed:
            desired_speed = self.max_speed - self.max_speed_margin

        lead_vehicle_speed = get_speed(lead_vehicle)

        # Calculate speed difference and time to collision
        speed_difference = max(1, (self.vehicle_speed - lead_vehicle_speed) / 3.6)
        time_to_collision = lead_distance / speed_difference if speed_difference != 0 else lead_distance / np.nextafter(0., 1.)

        self.ttc = time_to_collision

        # If within a critical time frame, decelerate.
        if self.safety_time > time_to_collision > 0.0:
            adjusted_speed = min(positive(lead_vehicle_speed - self.speed_decrease), desired_speed)
        # Otherwise, attempt to match the leading vehicle's speed.
        else:
            adjusted_speed = 0 if lead_vehicle_speed == 0 else min(lead_vehicle_speed + 1, desired_speed)
        return adjusted_speed

    def is_near_intersection(self, objects, waypoint_buffer):
        """
        Determines if upcoming waypoints are near an intersection by comparing
        their distances to traffic lights.

        Parameters
        ----------
        objects : dict
            Dictionary containing information about various environmental objects.

        waypoint_buffer : deque
            Queue of upcoming waypoints.

        Returns
        -------
        is_near_intersection : boolean
            Indicates if any upcoming waypoint is near an intersection.
        """

        traffic_lights = objects['traffic_lights']
        for traffic_light in traffic_lights:
            for waypoint, _ in waypoint_buffer:
                distance_to_light = traffic_light.get_location().distance(waypoint.transform.location)
                if distance_to_light < 20:
                    return True

        return False

    def is_near_destination(self):
        """
        Determines if the ego vehicle's current position is close to destination.

        Returns
        -------
        is_near : boolean
            True if the ego vehicle's current position is close to its destination; otherwise, False.
        """
        proximity_x = abs(self.vehicle_pos.location.x - self.end_waypoint.transform.location.x) <= 10
        proximity_y = abs(self.vehicle_pos.location.y - self.end_waypoint.transform.location.y) <= 10

        is_near = proximity_x and proximity_y
        return is_near

    def is_lane_change_allowed(self, previous_permission, collision_detection_enabled, curvature_list):
        """
        Determines whether a lane change is allowed.
        A multitude of conditions impact this decision, including road curvature,
        the state of the collision detector, and current overtake and push statuses.
        For an exhaustive list of conditions, please consult the function's logic.

        Parameters
        ----------
        previous_permission : boolean
            Permission for lane change from the previous check.

        collision_detection_enabled : boolean
            Indicates if the collision detection mechanism is active.

        curvature_list : list
            List containing curvature values for the planned path points.

        Returns
        -------
        is_lane_change_allowed : boolean
            True if the lane change is feasible, otherwise False.
        """

        # Forbid lane change if the path has significant curvature
        if len(curvature_list) > 2 and np.mean(np.abs(np.array(curvature_list))) > 0.04:
            previous_permission = False

        # Allow for a potential lane change only when all requirements are met:
        # - Collision detection is active.
        # - There's an actual lane change in the planned route, which presents as lane id and lane lateral changes.
        # - overtake hasn't happened : lane change should not be allowed if previously we have been doing an overtake.
        # - The destination has not been pushed.
        conditions_met = collision_detection_enabled and \
                         self.get_local_planner().lane_id_change and \
                         self.get_local_planner().lane_lateral_change and \
                         self.overtake_counter <= 0 and \
                         not self.destination_push_flag

        if conditions_met:
            previous_permission = previous_permission and self.check_lane_change()
            if not previous_permission:
                print("Lane change is not allowed")

        return previous_permission

    def push_destination(self, current_waypoint, is_intersection):
        """
        Determine a temporary destination (waypoint) for a push operation. This is useful in situations
        where a potential collision might occur, so the vehicle's destination should be "pushed" forward.

        Parameters
        ----------
        current_waypoint : carla.waypoint
            The current waypoint of the ego vehicle.

        is_intersection : boolean
            Indicates if the vehicle is currently at an intersection.

        Returns
        -------
        pushed_destination : carla.waypoint
            The calculated waypoint for the push operation.
        """

        waypoint_buffer = self.get_local_planner().get_global_route()
        mid_buffer_index = len(waypoint_buffer) // 2
        # If at an intersection, select a future waypoint that ensures continued travel on the same lane.
        if is_intersection:
            pushed_destination = waypoint_buffer[mid_buffer_index][0].next(
                max(self.vehicle_speed / 3.6, 10.0))[0]
        else:
            pushed_destination = current_waypoint.next(
                max(self.vehicle_speed / 3.6 * 3, 10.0))[0]

        print(
            'Vehicle ID: %d: Pushed the destination forward to prevent potential collision. New destination: %f, %f, %f' %
            (self.vehicle.id,
             pushed_destination.transform.location.x,
             pushed_destination.transform.location.y,
             pushed_destination.transform.location.z))

        return pushed_destination

    def handle_traffic_signals(self, current_waypoint):
        """
        Manages the agent's behavior at traffic lights and stop signs.
        NOTE: A temporary measure is used to prevent the vehicle from braking
        after passing a yellow light. This occurs when the vehicle remains affected
        by the traffic signal even after crossing it. To address this, the signal's
        ID is temporarily saved to bypass this behavior until the vehicle approaches
        a new signal.

        Parameters
        ----------
        current_waypoint : carla.waypoint
            The vehicle's present waypoint.

        Returns
        -------
        int
            1 if an emergency stop is required, 0 otherwise.
        """

        light_id = self.vehicle.get_traffic_light().id if self.vehicle.get_traffic_light() else -1
        # This condition represents a scenario where the vehicle recently passed a stop sign
        # and won't halt at any subsequent stop sign for the next 4 seconds.
        if 60 <= self.stop_sign_wait_count < 240:
            self.stop_sign_wait_count += 1
        elif self.stop_sign_wait_count >= 240:
            self.stop_sign_wait_count = 0

        if self.light_state == "Red":
            # If the light state is red and signal_id is -1, it means the vehicle is near a stop sign.
            if light_id == -1:
                # Make the vehicle wait for 2 seconds at the stop sign.
                if self.stop_sign_wait_count < 60:
                    self.stop_sign_wait_count += 1
                    # Indicate an emergency stop is needed.
                    return 1
                # Ensure the vehicle doesn't stop at the opposing stop sign after crossing one.
                else:
                    return 0

            # Case when the vehicle is at a red light, not at a junction, and the signal isn't the one to be ignored.
            if not current_waypoint.is_junction and (self.light_id_to_ignore != light_id or light_id == -1):
                return 1
            elif current_waypoint.is_junction and light_id != -1:
                self.light_id_to_ignore = light_id

        if self.light_id_to_ignore != light_id:
            self.light_id_to_ignore = -1
        return 0

    def rule_based_trajectory_planning(self, desired_speed=None, collision_detection=True, lane_change_allowed=True):
        """
        Implement the trajectory planning for vehicle with rule-based method. This method deals with overtaking,
        following, traffic light, and normal behavior.

        Parameters
        ----------
        desired_speed : float, optional
            Target speed the vehicle aims to achieve.

        collision_detection : bool
            Determines if collision checks should be made.

        lane_change_allowed : bool
            Permission to change lanes, typically passed from platoon behavior.

        Returns
        -------
        speed_buffer : narray
            The narray with planned speed repeated.

        location_buffer : deque
            The deque with all the locations in trajectory buffer.
        """

        # Fetch vehicle's current position
        ego_location = self.vehicle_pos.location
        ego_waypoint = self.map.get_waypoint(ego_location)
        waypoint_buffer = self.get_local_planner().get_global_route()

        # Set time-to-collision to a high value
        self.ttc = 1000

        # When overtake_counter > 0, another overtake/lane change is forbidden
        if self.overtake_counter > 0:
            self.overtake_counter -= 1

        # Reset destination push flag periodically
        if self.destination_push_flag > 0:
            self.destination_push_flag -= 1

        # Detect if the car is near an intersection
        near_intersection = self.is_near_intersection(self.objects, waypoint_buffer)

        # 0. Check if the vehicle is close to its destination
        if self.is_near_destination():
            print('Simulation is Over')
            sys.exit(0)

        # 1. Manage behavior at traffic lights
        if self.handle_traffic_signals(ego_waypoint) != 0:
            return 0, None

        # 2. Reset to global route if temporary route is finished
        if len(self.get_local_planner().get_waypoints_queue()) == 0 and len(waypoint_buffer) <= 2:
            print('Destination Reset!')
            self.overtake_allowed = self.overtake_allowed_origin
            self.lane_change_allowed = True
            self.destination_push_flag = 0
            self.set_local_planner(ego_location, self.end_waypoint.transform.location, clear_waypoints=True,
                                   clear_history=True)

        # No overtaking is allowed around intersections
        if near_intersection:
            self.overtake_allowed = False
        else:
            self.overtake_allowed = self.overtake_allowed_origin

        # Generate a local path based on global route
        path_x, path_y, path_k, path_yaw = self.local_planner.generate_local_path()

        # Check if lane change is allowed
        self.lane_change_allowed = self.is_lane_change_allowed(lane_change_allowed, collision_detection, path_k)

        # 3. Collision checking
        is_hazardous = False
        if collision_detection:
            is_hazardous, blocking_vehicle, gap = self.check_collision(path_x, path_y, path_yaw, ego_waypoint)
        following_other_vehicle = False

        # 4. Push the destination to a temporary one when lane change can not be made.
        # This happens when other vehicle block in the other lane when doing lane change.
        if not self.lane_change_allowed and self.get_local_planner().potential_curved_road and not self.destination_push_flag and self.overtake_counter <= 0:
            self.overtake_allowed = False
            new_target = self.push_destination(ego_waypoint, near_intersection)
            self.destination_push_flag = 90
            self.set_local_planner(ego_location, new_target.transform.location, clear_waypoints=True, reset_end=False)
            path_x, path_y, path_k, path_yaw = self.local_planner.generate_local_path()

        # 5. Handle front blocking vehicles and overtaking restrictions
        elif is_hazardous and (
                not self.overtake_allowed or self.overtake_counter > 0 or self.get_local_planner().potential_curved_road):
            following_other_vehicle = True

        # 6. Manage overtaking behavior
        elif is_hazardous and self.overtake_allowed and self.overtake_counter <= 0:
            blocking_speed = get_speed(blocking_vehicle)
            blocking_lane_id = self.map.get_waypoint(blocking_vehicle.get_location()).lane_id
            ego_lane_id = ego_waypoint.lane_id

            # Overtake only when having higher speed and at same lane
            if ego_lane_id == blocking_lane_id:
                if self.vehicle_speed >= blocking_speed - 5:
                    following_other_vehicle = self.overtake_management(blocking_vehicle)
                else:
                    following_other_vehicle = True

        # 7. Implement car-following behavior
        if following_other_vehicle:
            if gap < max(self.break_distance, 3):
                print("Stop car following")
                return 0, None

            desired_speed = self.manage_car_following(blocking_vehicle, gap, desired_speed)
            planned_speed, planned_location = self.local_planner.run_trajectory_planning(path_x, path_y, path_k,
                                                                                         desired_speed=desired_speed)
            trajectory_buffer = self.local_planner.get_trajectory()

            # Split trajectory into speed buffer and location buffer
            location_buffer = deque()
            for traj in trajectory_buffer:
                location_buffer.append(traj[0])
            speed_buffer = np.repeat(planned_speed, len(location_buffer))
            return speed_buffer, location_buffer

        # 8. Standard navigation behavior
        planned_speed, planned_location = self.local_planner.run_trajectory_planning(path_x, path_y, path_k,
                                                                                     desired_speed=self.max_speed - self.max_speed_margin if not desired_speed else desired_speed)
        trajectory_buffer = self.local_planner.get_trajectory()
        location_buffer = deque()
        for traj in trajectory_buffer:
            location_buffer.append(traj[0])
        speed_buffer = np.repeat(planned_speed, len(location_buffer))
        return speed_buffer, location_buffer

    def vehicle_control(self, target_speed, trajectory_buffer):
        """
        Pass the trajectory to controller and implement the vehicle control.

        Parameters
        ----------
        target_speed: float
            target_speed for vehicle control.

        trajectory_buffer: deque
            future trajectory deque.

        Returns
        -------
        control: carla.VehicleControl
            Carla control command.
        """
        control = self.controller.run_controller(target_speed, trajectory_buffer)
        return control
