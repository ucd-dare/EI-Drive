#!/usr/bin/env python

"""
Overtake Scenario:

The scripts simulate a scenario where an ego vehicle has to overtake a background vehicle
that is ahead of the ego vehicle and at a lower speed. There are two fearless pedestrians
that suddenly appear in front of the ego vehicle and the ego vehicle has to avoid a collision
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


class Overtake(BasicScenario):

    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Overtake Scenario")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        self.vehicle_01_velocity = 0
        self.vehicle_02_velocity = 8
        self.vehicle_03_velocity = 8
        self.vehicle_04_velocity = 8
        self._trigger_distance = 150

        super(Overtake, self).__init__("Overtake",
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
        car_01_transform = self.other_actors[0].get_transform()
        self.car_01_visible = carla.Transform(
            carla.Location(car_01_transform.location.x,
                           car_01_transform.location.y,
                           car_01_transform.location.z + 501),
            car_01_transform.rotation)

        car_02_transform = self.other_actors[1].get_transform()
        self.car_02_visible = carla.Transform(
            carla.Location(car_02_transform.location.x,
                           car_02_transform.location.y,
                           car_02_transform.location.z + 501),
            car_02_transform.rotation)

        car_03_transform = self.other_actors[2].get_transform()
        self.car_03_visible = carla.Transform(
            carla.Location(car_03_transform.location.x,
                           car_03_transform.location.y,
                           car_03_transform.location.z + 501),
            car_03_transform.rotation)

        car_04_transform = self.other_actors[3].get_transform()
        self.car_04_visible = carla.Transform(
            carla.Location(car_04_transform.location.x,
                           car_04_transform.location.y,
                           car_04_transform.location.z + 501),
            car_04_transform.rotation)

        # Trigger location for the actors
        self.vehicle_01_trigger_location = carla.Location(
            car_01_transform.location.x,
            car_01_transform.location.y,
            car_01_transform.location.z + 501,
        )
        self.vehicle_02_trigger_location = carla.Location(
            car_02_transform.location.x,
            car_02_transform.location.y,
            car_02_transform.location.z + 501,
        )
        self.vehicle_03_trigger_location = carla.Location(
            car_03_transform.location.x,
            car_03_transform.location.y,
            car_03_transform.location.z + 501,
        )
        self.vehicle_04_trigger_location = carla.Location(
            car_04_transform.location.x,
            car_04_transform.location.y,
            car_04_transform.location.z + 501,
        )

    def _create_behavior(self):
        # Vehicle behaviors
        sequence_vehicle_01 = py_trees.composites.Sequence("Vehicle_01")
        trigger_vehicle_01 = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.vehicle_01_trigger_location, self._trigger_distance)
        set_01_visible = ActorTransformSetter(
            self.other_actors[0], self.car_01_visible)
        vehicle_01_drive = WaypointFollower(
            self.other_actors[0], self.vehicle_01_velocity)
        # Drive fast vehicle
        sequence_vehicle_01.add_child(set_01_visible)
        sequence_vehicle_01.add_child(trigger_vehicle_01)
        sequence_vehicle_01.add_child(vehicle_01_drive)
        sequence_vehicle_01.add_child(Idle())

        sequence_vehicle_02 = py_trees.composites.Sequence("Vehicle_02")
        trigger_vehicle_02 = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.vehicle_02_trigger_location, self._trigger_distance)
        set_02_visible = ActorTransformSetter(
            self.other_actors[1], self.car_02_visible)
        vehicle_02_drive = WaypointFollower(
            self.other_actors[1], self.vehicle_02_velocity)
        sequence_vehicle_02.add_child(set_02_visible)
        sequence_vehicle_02.add_child(trigger_vehicle_02)
        sequence_vehicle_02.add_child(vehicle_02_drive)
        sequence_vehicle_02.add_child(Idle())

        sequence_vehicle_03 = py_trees.composites.Sequence("Vehicle_03")
        trigger_vehicle_03 = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.vehicle_03_trigger_location, self._trigger_distance)
        set_03_visible = ActorTransformSetter(
            self.other_actors[2], self.car_03_visible)
        vehicle_03_drive = WaypointFollower(
            self.other_actors[2], self.vehicle_03_velocity)
        sequence_vehicle_03.add_child(set_03_visible)
        sequence_vehicle_03.add_child(trigger_vehicle_03)
        sequence_vehicle_03.add_child(vehicle_03_drive)
        sequence_vehicle_03.add_child(Idle())

        sequence_vehicle_04 = py_trees.composites.Sequence("Vehicle_04")
        trigger_vehicle_04 = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.vehicle_04_trigger_location, self._trigger_distance)
        set_04_visible = ActorTransformSetter(
            self.other_actors[3], self.car_04_visible)
        vehicle_04_drive = WaypointFollower(
            self.other_actors[3], self.vehicle_04_velocity)
        sequence_vehicle_04.add_child(set_04_visible)
        sequence_vehicle_04.add_child(trigger_vehicle_04)
        sequence_vehicle_04.add_child(vehicle_04_drive)
        sequence_vehicle_04.add_child(Idle())

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 100)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(sequence_vehicle_01)
        root.add_child(sequence_vehicle_02)
        root.add_child(sequence_vehicle_03)
        root.add_child(sequence_vehicle_04)
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
