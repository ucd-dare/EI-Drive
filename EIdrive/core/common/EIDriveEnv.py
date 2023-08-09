# -*- coding: utf-8 -*-
# Author: Wei Shao <phdweishao@gmail.com>
# License: TDG-Attribution-NonCommercial-NoDistribute

import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from EIdrive.scenario_testing.utils.yaml_utils import load_yaml
from EIdrive.core.common.misc import get_speed
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import time
import gym
from gym import spaces
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
# from EIdrive.core.plan import RL_utils
from collections import deque
import matplotlib.pyplot as plt
import torch
import scenario_runner as sr
from multiprocessing import Process

from pynput import keyboard


class EIDriveEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render.modes": ["human"]}

    def exec_scenario_runner(self, scenario_params):
        """
        Execute the SenarioRunner process

        Parameters
        ----------
        scenario_params: Parameters of ScenarioRunner

        Returns
        -------
        """
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()

    def __init__(self, scenario_params, use_scenario_runner=False):
        super().__init__()
        self.use_scenario_runner=True
        self.discrete = False
        self.discrete_size = 21
        self.local_steering = False
        self.goal_reached = 250
        self.terminated = -250
        self.timeout = 0
        self.max_episode_length = 1000
        self.terminate_off_road = True
        self.terminate_off_lane = True
        self.scenario_params = scenario_params
        self.discrete = self.scenario_params['RL_environment']['actions']['discrete']
        if self.discrete:
            self.discrete_size = self.scenario_params['RL_environment']['actions']['n_steering_angles']
        self.local_steering = self.scenario_params['RL_environment']['actions']['local_steering']
        self.goal_reached = self.scenario_params['RL_environment']['rewards']['goal_reached']
        self.terminated = self.scenario_params['RL_environment']['rewards']['terminated']
        self.timeout = self.scenario_params['RL_environment']['rewards']['timeout']
        self.max_episode_length = self.scenario_params['RL_environment']['termination']['max_episode_length']
        self.terminate_off_road = self.scenario_params['RL_environment']['termination']['terminate_off_road']
        self.terminate_off_lane = self.scenario_params['RL_environment']['termination']['terminate_off_lane']
        self.dt = self.scenario_params['world']['fixed_delta_seconds']
        self.curr_control = carla.VehicleControl()
        self.parameters = scenario_params['scenario']

        self.cav_world = CavWorld()
        self.gameworld = sim_api.GameWorld(scenario_params, scenario_params.scenario.edge,
                                           town='Town06', cav_world=self.cav_world)
        self.world = self.gameworld.world
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.single_cav_list = None
        self.sr_process = None
        self.num_actors = 0
        self.prev_waypoint = None

        self.waypoint_horizon = self.scenario_params['RL_environment']['states']['waypoint_horizon']
        self.lane_resolution = self.scenario_params['RL_environment']['states']['lane_resolution']
        self.reward_progress = self.scenario_params['RL_environment']['rewards']['reward_progress']
        self.crosstrack_error = self.scenario_params['RL_environment']['rewards']['crosstrack_error']
        self.brake_penalty = self.scenario_params['RL_environment']['rewards']['brake_penalty']
        self.target_speed = 60


        self.scenario_runner = None
        if self.use_scenario_runner:
            self.scenario_runner = sr.ScenarioRunner(scenario_params.scenario.scenario_runner)
            self.scenario_runner.parse_scenarios()


        if self.discrete:
            self.action_space = spaces.Discrete(self.discrete_size*(2+3))
        else:
            if self.local_steering:
                self.action_space = spaces.Box(low=np.array([0.0, -0.1, 0.0]), high=np.array([1.0, 0.1, 1.0]),
                                               dtype=np.float32)
            else:
                self.action_space = spaces.Box(low=np.array([0.0, -1.0, 0.0]), high=np.array([1.0, 1.0, 0.0]),
                                           dtype=np.float32)
        if self.local_steering:
            self.observation_space = spaces.Box(low=np.concatenate(([-180, 0.0, 0.0, 0.0, 0.0, -1.0], np.full(self.waypoint_horizon, -180))),
                                                high=np.concatenate((np.array([180, 80, 1, 1, 1, 1]), np.full(self.waypoint_horizon, 180))), dtype=np.float32)
        else:
            self.observation_space = spaces.Box(low=np.concatenate(([-180, 0.0, 0.0, 0.0, 0.0], np.full(self.waypoint_horizon, -180))),
                                            high=np.concatenate((np.array([180, 80, 1, 1, 1]), np.full(self.waypoint_horizon, 180))), dtype=np.float32)


        self.render_waypoints = True
        self.t = 0
        self.lane_waypoints = []
        self.waypoint_buffer = deque()


        if not self.use_scenario_runner:
            single_cav_list = self.gameworld.create_vehicle_agent(application=['single'])
            self.single_cav_list = single_cav_list
            self.ego_vehicle = single_cav_list[0].vehicle
            self.spawn_points = self.parameters['single_cav_list'][0]['spawn_position']
            dest_idx = np.random.randint(len(self.spawn_points))
            self.spawn_point = carla.Location(x=self.spawn_points[dest_idx][0], y=self.spawn_points[dest_idx][1], z=self.spawn_points[dest_idx][2])
        self.spectator=None

        self.episode_start = time.perf_counter()
        self.episode_reward = 0


        self.reward_prog = 0
        self.penalty_lane = 0
        self.total_return = 0
        self.average_speed = 0
        self.episodes = 0
        self.reward_lane_prog = 0



    def step(self, action):
        if self.discrete:
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0,
                                           hand_brake=False, reverse=False, manual_gear_shift=False, gear=0)
            mod5 = action % 5

            if mod5 == 1:
                control.throttle = 0.5
            if mod5 == 2:
                control.throttle = 1.0
            if mod5 == 3:
                control.brake = 0.75
            if mod5 == 4:
                control.brake = 1.0
            div5 = action // 5
            if self.local_steering:
                angle_diff=(div5 - (self.discrete_size - 1) / 2) * (1 / ((self.discrete_size - 1) / 2)) / 10
                new_steering = max(min(self.curr_control.steer + angle_diff, 1.0), -1.0)
                control.steer=new_steering
            else:
                control.steer = (div5 - (self.discrete_size-1)/2) * (1 / ((self.discrete_size-1)/2))/3
        else:
            if self.local_steering:
                new_steering = max(min(self.curr_control.steer + (action[1]), 1.0), -1.0)
                control = carla.VehicleControl(throttle=float(action[0]), steer=new_steering,
                                               brake=float(action[2]), hand_brake=False,
                                               reverse=False, manual_gear_shift=False, gear=0)
            else:
                control = carla.VehicleControl(throttle=float(action[0]), steer=float(action[1]), brake=float(action[2]), hand_brake=False,
                                           reverse=False, manual_gear_shift=False, gear=0)

        if self.t <= 10000:
            control.brake = 0.0

        if control.brake < 0.05:
            control.brake = 0.0
        self.prev_control = self.ego_vehicle.get_control()
        self.ego_vehicle.apply_control(control)
        self.curr_control = control
        self.scenario_runner.tick()
        self.gameworld.tick(self.single_cav_list)   # TODO: After revise the function of tick(), bug remains to be fixed.
        '''
        if self.use_scenario_runner:
            time.sleep(self.dt)
        '''
        self.t += 1

        observation = self._get_obs()
        reward = self._get_reward(observation)
        info = {}
        done = False
        waypoint = self.map.get_waypoint(self.ego_location, False)
        if self.t >= self.max_episode_length or self.reward_lane_prog >= 500:
            done = True
            reward += self.timeout
            info["TimeLimit.truncated"] = True
            print('Timeout!')
        elif self.terminate_off_road and waypoint == None:
            done = True
            reward += self.terminated*self.ego_spd*self.dt
            print('Off road!')
        #elif self.terminate_off_lane and ((observation[2] < 0.1 or observation[3] < 0.1) or (self.curr_lane_id != self.curr_lane_waypoint.lane_id)):
        elif self.terminate_off_lane and (self.prev_waypoint.transform.location.distance(self.curr_lane_waypoint.transform.location)>2.0):
            done = True
            reward += self.terminated*self.ego_spd*self.dt - 10
            print('Off lane!')

        self.episode_reward += reward
        self.prev_waypoint = self.curr_lane_waypoint


        self.total_return += reward

        view_transform = carla.Transform()
        view_transform.location = self.ego_vehicle.get_transform().location
        view_transform.location.z = view_transform.location.z + self.spectator_altitude
        view_transform.rotation.pitch = self.spectator_bird_pitch
        self.spectator.set_transform(view_transform)

        return observation, reward, done, info

    def reset(self):
        if not self.use_scenario_runner:
            dest_idx = np.random.randint(len(self.spawn_points))
            '''
            self.destination = carla.Location(x=self.destinations[dest_idx][0], y=self.destinations[dest_idx][1],
                                              z=self.destinations[dest_idx][2])
            '''
            self.spawn_point = carla.Location(x=self.spawn_points[dest_idx][0], y=self.spawn_points[dest_idx][1],
                                              z=self.spawn_points[dest_idx][2])
            init_location = self.spawn_point
            init_rotation = carla.Rotation(yaw=self.spawn_points[dest_idx][3], roll=self.spawn_points[dest_idx][4],
                                           pitch=self.spawn_points[dest_idx][5])
            self.ego_vehicle.set_transform(carla.Transform(init_location, init_rotation))

            self.ego_vehicle.set_target_velocity(carla.Vector3D())
            self.ego_vehicle.set_target_angular_velocity(carla.Vector3D())
            self.curr_control = carla.VehicleControl(throttle=0, steer=0, brake=0, hand_brake=False,
                                                     reverse=False, manual_gear_shift=False, gear=0)
            self.ego_vehicle.apply_control(self.curr_control)
            self.gameworld.tick()
        else:
            ti = time.perf_counter()
            self.scenario_runner.terminate_scenario()
            self.scenario_runner._cleanup()
            config = np.random.choice(self.scenario_runner.scenario_configurations)
            self.scenario_runner.start_scenario(config)
            self.ego_vehicle = self.scenario_runner.ego_vehicles[0]
            #if self.spectator is not None:
            #    self.spectator.destroy()
            self.spectator = self.ego_vehicle.get_world().get_spectator()
            self.spectator_altitude = 50
            self.spectator_bird_pitch = -90
            print("reset time: "+str(time.perf_counter() - ti))
            '''
            self.ego_vehicle = None
            self.num_actors = 0
            while self.ego_vehicle is None or self.num_actors < self.scenario_params.scenario_runner.num_actors:
                print("Waiting for the actors")
                time.sleep(2)
                vehicles = self.world.get_actors().filter('vehicle.*')
                walkers = self.world.get_actors().filter('walker.*')
                for vehicle in vehicles:
                    if vehicle.attributes['role_name'] == 'hero':
                        print("Ego vehicle found")
                        self.ego_vehicle = vehicle
                self.num_actors = len(vehicles) + len(walkers)
            print(f'Found all {self.num_actors} actors')
            '''

            self.spawn_point = self.ego_vehicle.get_transform().location

        self.ego_transform = self.ego_vehicle.get_transform()
        self.ego_location = self.ego_transform.location
        lane_waypoint = self.map.get_waypoint(self.ego_location)
        self.ego_prev_acc = self.ego_acc = 0
        self.prev_waypoint = lane_waypoint
        observation = self._get_obs()
        self.episode_start = time.perf_counter()
        self.t = 0
        self.episode_reward = 0
        print('-----------------')
        print('reward_prog: '+str(self.reward_prog))
        print('average speed: ' + str(self.average_speed))
        print('reward: ' + str(self.total_return))
        self.reward_prog = 0
        self.penalty_lane = 0
        self.total_return = 0
        self.average_speed = 0
        self.reward_lane_prog = 0
        self.episodes += 1
        self.curr_lane_id = self.curr_lane_waypoint.lane_id

        return observation  # reward, done, info can't be included

    def render(self, mode="human"):
        return



    def _get_obs(self):
        self.ego_transform = self.ego_vehicle.get_transform()
        self.ego_location = self.ego_transform.location
        ego_yaw = self.ego_transform.rotation.yaw
        self.ego_spd = self.ego_vehicle.get_velocity().length()
        self.ego_prev_acc = self.ego_acc
        self.ego_acc = self.ego_vehicle.get_acceleration().length()
        if self.t>0:
            self.average_speed = (self.average_speed*(self.t-1) + self.ego_spd)/self.t

        # Gets the waypoint that is located in the center of lane the where the ego vehicle is located
        lane_waypoint = self.map.get_waypoint(self.ego_location)
        center_location = lane_waypoint.transform.location
        lane_width = lane_waypoint.lane_width
        right_vector = lane_waypoint.transform.rotation.get_right_vector()
        right_marking = center_location + lane_width / 2 * right_vector
        left_marking = center_location - lane_width / 2 * right_vector
        right_offset = self.ego_location.distance(right_marking) / lane_width
        left_offset = self.ego_location.distance(left_marking) / lane_width
        lane_offset = self.ego_location.distance(center_location) / (lane_width/2)
        self.curr_lane_waypoint = lane_waypoint
        if self.render_waypoints:
            self.gameworld.world.debug.draw_point(center_location, size=0.2, life_time=0.15)
            self.gameworld.world.debug.draw_point(right_marking, size=0.2, life_time=0.15)
            self.gameworld.world.debug.draw_point(left_marking, size=0.2, life_time=0.15)
        self.waypoint_buffer.clear()
        nxt = self.curr_lane_waypoint
        for idx in range(self.waypoint_horizon+1):
            self.waypoint_buffer.append(nxt)
            if nxt != None:
                nxt = list(nxt.next(self.lane_resolution))[0]
                if self.render_waypoints:
                    self.gameworld.world.debug.draw_point(nxt.transform.location, size=0.2,
                                                          life_time=0.15)

        angles = []
        for i in range(self.waypoint_horizon):
            # Compute the vector from point i to point i+1
            if self.waypoint_buffer[i] == None or self.waypoint_buffer[i+1] == None:
                angles.append(0.0)
            else:
                loc_i = np.array(
                    [self.waypoint_buffer[i].transform.location.x, self.waypoint_buffer[i].transform.location.y])
                loc_i_1 = np.array([self.waypoint_buffer[i + 1].transform.location.x,
                                    self.waypoint_buffer[i + 1].transform.location.y])
                v = loc_i_1 - loc_i

                # Compute the angle between v and the directional vector of point i
                angle_rad = math.atan2(loc_i_1[1]-loc_i[1], loc_i_1[0]-loc_i[0])
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0:
                    angle_deg += 360
                # angle_deg = (angle_deg + 90) % 360

                #angle = np.degrees(np.arctan2(np.linalg.det([loc_i, v]), np.dot(loc_i, v)))
                direction_diff = self.waypoint_buffer[i].transform.rotation.yaw - angle_deg
                direction_diff = np.mod(direction_diff + 180, 360) - 180
                angles.append(-1*direction_diff)

        # Compute the difference between the actual ego yaw and the direction of the first waypoint
        ego_yaw_diff = ego_yaw - self.waypoint_buffer[0].transform.rotation.yaw

        # Ensure that the ego yaw difference is between -180 and 180 degrees
        ego_yaw_diff = np.mod(ego_yaw_diff + 180, 360) - 180


        if self.local_steering:
            observation = np.concatenate((np.array(
                [ego_yaw_diff, self.ego_spd, right_offset,
                 left_offset, lane_offset, self.curr_control.steer]), angles), dtype=np.float32).flatten()
        else:
            observation = np.concatenate((np.array(
            [ego_yaw_diff, self.ego_spd, right_offset,
             left_offset, lane_offset]), angles), dtype=np.float32).flatten()
        return observation

    def _get_reward(self, observation):
        lane_waypoint = self.map.get_waypoint(self.ego_location)
        r_lane = observation[4]  # this is lane offset, 0 means in center of lane, 1 means on right or left lane marking

        dx = self.waypoint_buffer[1].transform.location.x - self.waypoint_buffer[0].transform.location.x+0.001
        dy = self.waypoint_buffer[1].transform.location.y - self.waypoint_buffer[0].transform.location.y

        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360

        wp_yaw = angle_deg
        wp_yaw = lane_waypoint.transform.rotation.yaw
        yaw_diff = wp_yaw - self.ego_transform.rotation.yaw
        yaw_diff = np.mod(yaw_diff + 180, 360) - 180

        # Compute the distance travelled along the direction of the first waypoint during timestep dt
        rew_spd = self.ego_spd
        if self.ego_spd > self.target_speed*0.447:
            rew_spd = 0
        speed_along_direction = rew_spd * np.cos(np.radians(yaw_diff))
        distance_along_direction = speed_along_direction * self.dt
        self.reward_lane_prog += distance_along_direction

        speed_along_direction = rew_spd * np.cos(np.radians(yaw_diff))
        distance_along_direction = speed_along_direction * self.dt

        reward_progress = distance_along_direction
        self.reward_prog += reward_progress
        penalty_acc = abs(self.ego_prev_acc - self.ego_acc)
        r_steer = abs(self.ego_vehicle.get_control().steer)

        r_turn = 0
        if np.sin(math.pi/2*r_steer)*self.ego_spd > 20:
            print("too sharp!")
            r_turn = 20
        #-0.25* rew_spd * abs(np.sin(np.radians(yaw_diff)))* self.dt
        reward = self.reward_progress*reward_progress- self.crosstrack_error*r_lane*self.ego_spd*self.dt-self.brake_penalty*self.ego_vehicle.get_control().brake*self.ego_spd*self.dt -  r_turn -0.2 * abs(self.curr_control.steer - self.prev_control.steer) * self.ego_spd * self.dt# - penalty_lane_offset
        reward -= 0.2
        return reward

    def save_fig(self):
        plt.savefig('SAC_lane_keeping_rewards2.svg')

    def close(self):
        self.gameworld.close()
        self.scenario_runner._cleanup()
        return


