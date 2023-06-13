# -*- coding: utf-8 -*-
"""
Script to run different scenarios.
"""

# Author: Wei Shao <phdweishao@gmail.com>
# License: TDG-Attribution-NonCommercial-NoDistrib

import importlib
import sys
import hydra
from omegaconf import DictConfig, OmegaConf
from omegaconf import OmegaConf

from EIdrive.scenario_testing.utils.yaml_utils import load_yaml
from EIdrive.version import __version__
import os


@hydra.main(version_base=None, config_path='EIdrive/scenario_testing/config_yaml', config_name='default')
def main(cfg: DictConfig) -> None:
    # set the default yaml file
    default_yaml = config_yaml = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'EIdrive/scenario_testing/config_yaml/default.yaml')
    # set the yaml file for the specific testing scenario
    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'EIdrive/scenario_testing/config_yaml/%s.yaml' % cfg.test_scenario)
    # load the default yaml file and the scenario yaml file as dictionaries
    default_dict = OmegaConf.load(default_yaml)
    scene_dict = OmegaConf.load(config_yaml)
    # merge the dictionaries
    scene_dict = OmegaConf.merge(default_dict, scene_dict)

    # import the testing script
    testing_scenario = importlib.import_module(
        "EIdrive.scenario_testing.%s" % cfg.test_scenario)
    # check if the yaml file for the specific testing scenario exists
    if not os.path.isfile(config_yaml):
        sys.exit(
            "EIdrive/scenario_testing/config_yaml/%s.yaml not found!" % cfg.test_cenario)

    # get the function for running the scenario from the testing script
    scenario_runner = getattr(testing_scenario, 'run_scenario')
    # run the scenario testing
    scenario_runner(cfg, scene_dict)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
