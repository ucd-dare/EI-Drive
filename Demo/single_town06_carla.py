# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import glob
import os
import sys
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.common.misc import get_speed
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time
import tkinter as tk

def run_scenario(opt, config_yaml):
    scenario_params = load_yaml(config_yaml)

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)
    # create scenario manager
    scenario_manager = sim_api.ScenarioManager(scenario_params,
                                               opt.apply_ml,
                                               opt.version,
                                               town='Town06',
                                               cav_world=cav_world)
    if opt.record:
        scenario_manager.client. \
            start_recorder("single_town06_carla.log", True)
    single_cav_list = \
        scenario_manager.create_vehicle_manager(application=['single'])
    # create background traffic in carla
    traffic_manager, bg_veh_list = \
        scenario_manager.create_traffic_carla()
    # create evaluation manager
    eval_manager = \
        EvaluationManager(scenario_manager.cav_world,
                          script_name='single_town06_carla',
                          current_time=scenario_params['current_time'])
    spectator = scenario_manager.world.get_spectator()

    # run steps
    v_speeds_list = []
    for i in range(len(single_cav_list)):
        v_speeds_list.append([])

    #press Q to stop
    #keyboard.wait('q')
    #input()
    t=0
    while True:
        scenario_manager.tick()
        transform = single_cav_list[0].vehicle.get_transform()
        #print(transform.location)
        spectator.set_transform(carla.Transform(
            carla.Location(
                x=0, y=45.5, z=100),
            carla.Rotation(
                pitch=-
                90)))
        # time.sleep(0.2)
        for i, single_cav in enumerate(single_cav_list):
            single_cav.update_info()
            v_speeds_list[i].append(get_speed(single_cav.vehicle))
            control = single_cav.run_step()

            single_cav.vehicle.apply_control(control)
            # call backend setting here to solve the conflict between cv2 pyqt5
            # and pyplot qtagg
        # try:
        #     matplotlib.use('TkAgg')
        # except ImportError:
        #     pass
        matplotlib.use('TkAgg')
        plt.cla()
        plt.title('Speed of Vehicles')
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event', lambda event: [
                plt.close() if event.key == 'escape' else None])
        plt.plot(np.arange(len(v_speeds_list[0])),
            v_speeds_list[0], "-b", label='Car 0')
        plt.plot(np.arange(len(v_speeds_list[1])),
            v_speeds_list[1], "-r", label='Car 1')
        plt.plot(np.arange(len(v_speeds_list[2])),
            v_speeds_list[2], "-g", label='Car 2')
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.xlabel("Game World time(s)")
        plt.ylabel("Vehicle Speed(km/h)")
        plt.pause(0.001)
        t = t+1

        if(t == 1):
            input()

        # eval_manager.evaluate()

        # class HelpText(object):
        #     def __init__(self, font, width, height):
        #         self.font = font
        #         self.line_space = 18
        #         self.dim = (780, len(lines) * self.line_space + 12)
        #         self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        #         self.seconds_left = 0
        #         self.surface = pygame.Surface(self.dim)
        #         self.surface.fill((0, 0, 0, 0))
        #         for n, line in enumerate(lines):
        #             text_texture = self.font.render(line, True, (255, 255, 255))
        #             self.surface.blit(text_texture, (22, n * self.line_space))
        #             self._render = False
        #         self.surface.set_alpha(220)
        #     def render(self, display):
        #         if self._render:
        #             display.blit(self.surface, self.pos)
        # pygame.font.init()
        # font = pygame.font.Font(pygame.font.get_default_font(), 20)
        # font_name = 'courier' if os.name == 'nt' else 'mono'
        # fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        # default_font = 'ubuntumono'
        # mono = default_font if default_font in fonts else fonts[0]
        # mono = pygame.font.match_font(mono)
        # HelpText(pygame.font.Font(mono, 16), 200, 200).render(display)

    # window = tk.Tk()
    # window.title('Info Hud')
    # window.geometry('500x300')
    # l = tk.Label(window, text='speed', bg='green', font=('Arial', 12), width=30, height=2)
    # l.pack()
    #window.mainloop()
    # input()
    if opt.record:
        scenario_manager.client.stop_recorder()

    scenario_manager.close()

    for v in single_cav_list:
        v.destroy()
    for v in bg_veh_list:
        v.destroy()
