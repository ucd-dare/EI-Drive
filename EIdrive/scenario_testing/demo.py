import carla
import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from EIdrive.core.common.misc import get_speed
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import math
import pandas as pd
import sys
import time

from pynput import keyboard


player_ids = []


def get_latency(single_cav_list):
    latencies = []
    for vm in single_cav_list:
        loc_vehicle = vm.vehicle.get_transform()
        x = loc_vehicle.location.x
        y = loc_vehicle.location.y
        latency = 2 * math.sqrt(x ** 2 + (y - 45.5) ** 2)
        latencies.append(latency)
    return latencies


def run_scenario(scenario_params):
    try:
        if scenario_params.scenario.edge:
            path = scenario_params.scenario.manual_path
            frames = [pd.read_csv(f'{i}{path}') for i in range(6)]
            df = pd.concat(frames)

        # create CAV world
        cav_world = CavWorld()
        # create scenario manager
        gameworld = sim_api.GameWorld(scenario_params,
                                      scenario_params.scenario.edge,
                                      town='Town06',
                                      cav_world=cav_world)
        if scenario_params.common_params.record:
            gameworld.client. \
                start_recorder("single_town06_carla.log", True)
        single_cav_list = \
            gameworld.create_vehicle_agent(application=['single'], data_dump=False)

        # Record the id of players
        for vm in single_cav_list:
            player_ids.append(vm.vehicle.id)
        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            gameworld.create_traffic_carla()
        # create evaluation manager
        eval_manager = \
            EvaluationManager(gameworld.cav_world,
                              script_name='demo',
                              current_time='')
        spectator = gameworld.world.get_spectator()

        # run steps
        v_speeds_list = []
        for i in range(len(single_cav_list)):
            v_speeds_list.append([])

        latency_list = []
        for i in range(0, 3):
            latency_list.append([])

        t = 0
        z_location = 75
        kl = KeyListener()
        kl.start()

        while True:
            if kl.keys['esc']:
                exit(0)
            if kl.keys['p']:
                continue
            # draw edge
            if t % 5 == 1 and scenario_params.scenario.edge:
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=32, z=0.2),
                    size=0.5,  # initial 0.2
                    color=carla.Color(0, 255, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=57, z=0.2),
                    size=0.5,
                    color=carla.Color(0, 255, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=22, z=0.2),
                    size=0.5,
                    color=carla.Color(0, 255, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=67, z=0.2),
                    size=0.5,
                    color=carla.Color(0, 255, 0),
                    life_time=0.2)
            elif not scenario_params.scenario.edge:
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=32, z=0.2),
                    size=0.5,
                    color=carla.Color(255, 0, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=57, z=0.2),
                    size=0.5,
                    color=carla.Color(255, 0, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=22, z=0.2),
                    size=0.5,
                    color=carla.Color(255, 0, 0),
                    life_time=0.2)
                gameworld.world.debug.draw_point(
                    carla.Location(
                        x=-1.7, y=67, z=0.2),
                    size=0.5,
                    color=carla.Color(255, 0, 0),
                    life_time=0.2)

            gameworld.tick(single_cav_list)

            # zoom in/out spectator
            if t < 60:
                z_location = z_location - 0.55
            elif t < 100:
                z_location = z_location + 0.25
            spectator.set_transform(carla.Transform(
                carla.Location(
                    x=0, y=45.5, z=z_location),
                carla.Rotation(
                    pitch=-
                    90)))

            # draw trajectory

            if scenario_params.scenario.edge:
                if t < 185:
                    df_temp = df[(df['tick'] == t + 8) | (df['tick'] == t + 11) | (df['tick'] == t + 14)]
                    for k in range(18):
                        gameworld.world.debug.draw_point(
                            carla.Location(
                                x=df_temp.iloc[k][0], y=df_temp.iloc[k][1],
                                z=0.2),
                            size=0.2,  # initially 0.1
                            color=carla.Color(0, 0, 255),
                            life_time=0.1)
                if t > 14:
                    df_temp2 = df[(df['tick'] == t - 8) | (df['tick'] == t - 11) | (df['tick'] == t - 14)]
                    for k in range(18):
                        gameworld.world.debug.draw_point(
                            carla.Location(
                                x=df_temp2.iloc[k][0], y=df_temp2.iloc[k][1],
                                z=0.2),
                            size=0.2,
                            color=carla.Color(255, 0, 255),
                            life_time=0.1)
                if t < 185:
                    df_temp3 = df[(df['tick'] > t) & (df['tick'] < t + 16)]
                    for k in range(90):
                        gameworld.world.debug.draw_point(
                            carla.Location(
                                x=df_temp3.iloc[k][0], y=df_temp3.iloc[k][1],
                                z=0.2),
                            size=0.2,
                            color=carla.Color(0, 255, 0),
                            life_time=0.1)

                # compute latency
                car0 = single_cav_list[0].vehicle.get_transform()
                car1 = single_cav_list[1].vehicle.get_transform()
                car2 = single_cav_list[2].vehicle.get_transform()
                x0 = car0.location.x
                y0 = car0.location.y
                x1 = car1.location.x
                y1 = car1.location.y
                x2 = car2.location.x
                y2 = car2.location.y
                latency01 = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
                latency02 = math.sqrt((x0 - x2) ** 2 + (y0 - y2) ** 2)
                latency12 = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                latency_list[0].append(latency01)
                latency_list[1].append(latency02)
                latency_list[2].append(latency12)

            # run step
            for i, single_cav in enumerate(single_cav_list):
                v_speeds_list[i].append(get_speed(single_cav.vehicle))

            # draw figures
            matplotlib.use('TkAgg')
            plt.cla()

            fig_v = plt.figure(1)
            if t == 0:
                mngr = plt.get_current_fig_manager()
                mngr.window.wm_geometry("1000x1000+3000+0")  # initially 503x519+1404+19
            plt.axis('off')
            ax = fig_v.subplots()
            ax.set_title('Speed of Vehicles', fontsize=30)
            ax.plot(np.arange(len(v_speeds_list[0])),
                    v_speeds_list[0], "-b", linewidth=5, label=f'Actor {player_ids[0]}')
            ax.plot(np.arange(len(v_speeds_list[1])),
                    v_speeds_list[1], "-r", linewidth=5, label=f'Actor {player_ids[1]}')
            ax.plot(np.arange(len(v_speeds_list[2])),
                    v_speeds_list[2], "-g", linewidth=5, label=f'Actor {player_ids[2]}')
            ax.axis("equal")
            ax.grid(True)
            ax.legend(fontsize=20)
            ax.set_xlabel("Game World Time(s)", fontsize=20)
            ax.set_ylabel("Vehicle Speed(km/h)", fontsize=20)

            if scenario_params.scenario.edge:
                plt.figure(2)
                if t == 0:
                    mngr = plt.get_current_fig_manager()
                    mngr.window.wm_geometry("1000x1000+3000+1500")  # initially 503x459+1404+576
                plt.title('Communication Latency between Vehicles', fontsize=30)
                plt.plot(np.arange(len(latency_list[0])), latency_list[0], "-b", linewidth=5, label=f'Latency between Actor {player_ids[0]} and {player_ids[1]}')
                plt.plot(np.arange(len(latency_list[1])), latency_list[1], "-r", linewidth=5, label=f'Latency between Actor {player_ids[0]} and {player_ids[2]}')
                plt.plot(np.arange(len(latency_list[2])), latency_list[2], "-g", linewidth=5, label=f'Latency between Actor {player_ids[1]} and {player_ids[2]}')
                plt.axis("equal")
                plt.grid(True)
                plt.legend(fontsize=20)
                plt.xlabel("Game World Time(s)", fontsize=20)
                plt.ylabel("Latency(ms)", fontsize=20)
            plt.pause(0.01)
            ax.cla()

            t = t + 1

            if 200 == t:
                sys.exit(0)

    finally:
        # eval_manager.evaluate()
        if scenario_params.common_params.record:
            gameworld.client.stop_recorder()

        gameworld.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()
