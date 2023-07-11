#!/usr/bin/env python

"""
A-to-B Scenario:

The scripts simulate the simplest scenario where an ego vehicle goes from A to B
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


class AtoB(BasicScenario):
    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running A-to-B Scenario")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        super(AtoB, self).__init__("AtoB",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        pass

    def _create_behavior(self):
        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 200)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Scenario Behaviors", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        if self.ego_vehicles[0] is not None:
            print("HAVE EGO VEHICLE")
        else:
            print("NO EGO VEHICLE")

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
