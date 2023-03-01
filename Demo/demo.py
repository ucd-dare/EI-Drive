# -*- coding: utf-8 -*-
"""
Script to run different scenarios.
"""

# Author: Wei Shao <phdweishao@gmail.com>

import argparse
import importlib
import os
import sys


def arg_parse():
    parser = argparse.ArgumentParser(description="EI-Drive scenario runner.")
    parser.add_argument('-t', "--test_scenario", required=True, type=str,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml.')
    parser.add_argument("--record", action='store_true', help='whether to record and save the simulation process to'
                                                              '.log file')
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-v', "--version", type=str, default='0.9.13',
                        help='Specify the CARLA simulator version, default'
                             'is 0.9.11, 0.9.12 is also supported.')

    opt = parser.parse_args()
    return opt


def main():
    opt = arg_parse()
    testing_scenario = importlib.import_module("%s" % opt.test_scenario)

    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               '%s.yaml' % opt.test_scenario)
    if not os.path.isfile(config_yaml):
        sys.exit("%s.yaml not found!" % opt.test_scenario)

    scenario_runner = getattr(testing_scenario, 'run_scenario')
    # run scenario testing
    scenario_runner(opt, config_yaml)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')



# # -*- coding: utf-8 -*-
# # Author: Wei Shao <phdweishao@gmail.com>
# # License: TDG-Attribution-NonCommercial-NoDistribute
#
# import carla
# import opencda.scenario_testing.utils.sim_api as sim_api
# from opencda.core.common.cav_world import CavWorld
# from opencda.scenario_testing.evaluations.evaluate_manager import \
#     EvaluationManager
# from opencda.scenario_testing.utils.yaml_utils import load_yaml
# from opencda.core.common.misc import get_speed
#
# import matplotlib
# import matplotlib.pyplot as plt
# import numpy as np
# import math
# import sys
# import time
# # import keyboard
#
#
# # class PlayerID:
# #     def __init__(self, player_id):
# #         self.players = player_id
# #
# player_ids = []
#
#
# def run_scenario(opt, config_yaml):
#     try:
#         scenario_params = load_yaml(config_yaml)
#
#         # create CAV world
#         cav_world = CavWorld(opt.apply_ml)
#         # create scenario manager
#         scenario_manager = sim_api.ScenarioManager(scenario_params,
#                                                    opt.apply_ml,
#                                                    opt.version,
#                                                    town='Town06',
#                                                    cav_world=cav_world)
#         if opt.record:
#             scenario_manager.client. \
#                 start_recorder("single_town06_carla.log", True)
#         single_cav_list = \
#             scenario_manager.create_vehicle_manager(application=['single'])
#
#         for vm in single_cav_list:
#             player_ids.append(vm.vehicle.id)
#         # create background traffic in carla
#         traffic_manager, bg_veh_list = \
#             scenario_manager.create_traffic_carla()
#         # create evaluation manager
#         eval_manager = \
#             EvaluationManager(scenario_manager.cav_world,
#                               script_name='demo',
#                               current_time=scenario_params['current_time'])
#         spectator = scenario_manager.world.get_spectator()
#
#         # run steps
#         v_speeds_list = []
#         for i in range(len(single_cav_list)):
#             v_speeds_list.append([])
#
#         latency_list = []
#         for i in range(0, 3):
#             latency_list.append([])
#
#         # press Q to stop
#         # keyboard.wait('q')
#         # input()
#         t = 0
#         z_location = 75
#         while True:
#             scenario_manager.tick()
#             if t < 62:
#                 z_location = z_location - 0.55
#             else:
#                 z_location = z_location + 0.3
#
#             spectator.set_transform(carla.Transform(
#                 carla.Location(
#                     x=0, y=45.5, z=z_location),
#                 carla.Rotation(
#                     pitch=-
#                     90)))
#
#             # time.sleep(0.02)
#
#
#             # compute latency
#             car0 = single_cav_list[0].vehicle.get_transform()
#             car1 = single_cav_list[1].vehicle.get_transform()
#             car2 = single_cav_list[2].vehicle.get_transform()
#             x0 = car0.location.x
#             y0 = car0.location.y
#             x1 = car1.location.x
#             y1 = car1.location.y
#             x2 = car2.location.x
#             y2 = car2.location.y
#             latency01 = math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
#             latency02 = math.sqrt((x0 - x2) ** 2 + (y0 - y2) ** 2)
#             latency12 = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
#             latency_list[0].append(latency01)
#             latency_list[1].append(latency02)
#             latency_list[2].append(latency12)
#
#             for i, single_cav in enumerate (single_cav_list):
#                 single_cav.update_info()
#                 v_speeds_list[i].append(get_speed(single_cav.vehicle))
#                 control = single_cav.run_step()
#
#                 single_cav.vehicle.apply_control(control)
#                 # call backend setting here to solve the conflict between cv2 pyqt5
#                 # and pyplot qtagg
#             # try:
#             #     matplotlib.use('TkAgg')
#             # except ImportError:
#             #     pass
#             matplotlib.use('TkAgg')
#             plt.cla()
#
#             fig_v = plt.figure(1)
#             if t == 0:
#                 mngr = plt.get_current_fig_manager()
#                 mngr.window.wm_geometry("503x519+1404+19")
#             # else:
#             #     mngr = plt.get_current_fig_manager()
#             #     geom = mngr.window.geometry()
#             #     print(f"1st figure:{geom}")
#             plt.axis('off')
#             ax = fig_v.subplots()
#             ax.set_title('Speed of Vehicles')
#             ax.plot(np.arange(len(v_speeds_list[0])),
#                 v_speeds_list[0], "-b", label='Car 0')
#             ax.plot(np.arange(len(v_speeds_list[1])),
#                 v_speeds_list[1], "-r", label='Car 1')
#             ax.plot(np.arange(len(v_speeds_list[2])),
#                 v_speeds_list[2], "-g", label='Car 2')
#             ax.axis("equal")
#             ax.grid(True)
#             ax.legend()
#             ax.set_xlabel("Game World Time(s)")
#             ax.set_ylabel("Vehicle Speed(km/h)")
#
#             plt.figure(2)
#             if t == 0:
#                 mngr = plt.get_current_fig_manager()
#                 mngr.window.wm_geometry("503x459+1404+576")
#             # else:
#             #     mngr = plt.get_current_fig_manager()
#             #     geom = mngr.window.geometry()
#             #     print(f"2nd figure:{geom}")
#             plt.title('Communication Latency between Vehicles')
#             plt.plot(np.arange(len(latency_list[0])), latency_list[0], "-b", label='Latency between Car0 and Car1')
#             plt.plot(np.arange(len(latency_list[1])), latency_list[1], "-r", label='Latency between Car0 and Car2')
#             plt.plot(np.arange(len(latency_list[2])), latency_list[2], "-g", label='Latency between Car1 and Car2')
#             plt.axis("equal")
#             plt.grid(True)
#             plt.legend()
#             plt.xlabel("Game World Time(s)")
#             plt.ylabel("Latency(ms)")
#             plt.pause(0.01)
#
#             ax.cla()
#
#             t = t + 1
#             # if(t == 1):
#             #     #time.sleep(120)
#             #     input()
#             #     time.sleep(0.5)
#
#         # window = tk.Tk()
#         # window.title('Info Hud')
#         # window.geometry('500x300')
#         # l = tk.Label(window, text='speed', bg='green', font=('Arial', 12), width=30, height=2)
#         # l.pack()
#         # window.mainloop()
#         # input()
#
#     finally:
#         # eval_manager.evaluate()
#         if opt.record:
#             scenario_manager.client.stop_recorder()
#
#         scenario_manager.close()
#
#         for v in single_cav_list:
#             v.destroy()
#         for v in bg_veh_list:
#             v.destroy()
#

