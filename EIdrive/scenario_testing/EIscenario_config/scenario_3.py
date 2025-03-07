#!/usr/bin/env python

"""
Ego vehicle is at an intersection in Town 3. There is a vehicle hidden by a truck also entering the intersection.
Objective is to cross the intersection and avoid the hidden vehicle.
"""

import py_trees
import carla

from scenario_runner.srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle)
from scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from scenario_runner.srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from scenario_runner.srunner.scenarios.basic_scenario import BasicScenario


class Scenario_3(BasicScenario):

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Scenario 3")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        self.num_vehicle = 6
        self.vehicle_01_velocity = 7  # Violated vehicle
        self.vehicle_02_velocity = 0  # Large vehicles from 02 to 06
        self.vehicle_03_velocity = 0
        self.vehicle_04_velocity = 0
        self.vehicle_05_velocity = 0
        self.vehicle_06_velocity = 0
        self._trigger_distance = 150

        super(Scenario_3, self).__init__("Scenario_3",
                                                ego_vehicles,
                                                config,
                                                world,
                                                debug_mode,
                                                criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Spawn vehicles
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        # Transformation that renders the vehicle visible
        for i in range(self.num_vehicle):
            car_transform = self.other_actors[i].get_transform()
            setattr(self, f"car_0{i + 1}_visible", carla.Transform(
                carla.Location(car_transform.location.x,
                               car_transform.location.y,
                               car_transform.location.z + 501),
                car_transform.rotation))

            # Trigger location for the actors
            setattr(self, f"vehicle_0{i + 1}_trigger_location", carla.Location(
                car_transform.location.x,
                car_transform.location.y,
                car_transform.location.z + 501, ))

    def _create_behavior(self):

        sequence_vehicle = []

        # Vehicle behaviors
        for i in range(self.num_vehicle):
            sequence_vehicle.append(py_trees.composites.Sequence(f"Vehicle_0{i + 1}"))
            trigger_location = getattr(self, f"vehicle_0{i + 1}_trigger_location")
            actor = self.other_actors[i]
            transform = getattr(self, f"car_0{i + 1}_visible")
            velocity = getattr(self, f"vehicle_0{i + 1}_velocity")

            trigger_behavior = InTriggerDistanceToLocation(self.ego_vehicles[0], trigger_location,
                                                           self._trigger_distance)
            set_transform_behavior = ActorTransformSetter(actor, transform)
            if i == 0:
                waypoint = [carla.Location(x=-108.6, y=129.5, z=0.5), carla.Location(x=-120.6, y=129.5, z=0.5), carla.Location(x=-140.6, y=115.2, z=0.5), carla.Location(x=-142.0, y=87.6, z=0.5)]
                drive_behavior = WaypointFollower(actor, velocity, plan=waypoint)
            else:
                drive_behavior = WaypointFollower(actor, velocity)

            sequence_vehicle[i].add_child(set_transform_behavior)
            sequence_vehicle[i].add_child(trigger_behavior)
            sequence_vehicle[i].add_child(drive_behavior)
            sequence_vehicle[i].add_child(Idle())

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 100)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(self.num_vehicle):
            root.add_child(sequence_vehicle[i])
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
