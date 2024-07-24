"""
Script to run different test_scenario.
"""

import argparse
import importlib
import hydra
from omegaconf import DictConfig, OmegaConf
from omegaconf import OmegaConf

from EIdrive.version import __version__


def arg_parse():
    parser = argparse.ArgumentParser(description="EI-Drive scenario runner.")
    parser.add_argument('-t', "--test_scenario", required=True, type=str,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'EIdrive/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in EIdrive/scenario_testing/config_yaml.')
    parser.add_argument("--record", action='store_true', help='whether to record and save the simulation process to'
                                                              '.log file')
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-v', "--version", type=str, default='0.9.11',
                        help='Specify the CARLA simulator version, default'
                             'is 0.9.11, 0.9.12 is also supported.')
    parser.add_argument('--edge', action='store_true',
                        help='whether to enable edge')

    opt = parser.parse_args()
    return opt


@hydra.main(version_base=None, config_path='EIdrive/scenario_testing/config_yaml', config_name='config')
def main(cfg: DictConfig) -> None:
    scene_dict = OmegaConf.create(cfg.test_scenario)
    scene_dict = OmegaConf.merge(scene_dict, OmegaConf.create(cfg.world))

    # import the testing script
    testing_scenario = importlib.import_module(
        "EIdrive.scenario_testing.%s" % scene_dict.scenario.test_scenario)

    # get the function for running the scenario from the testing script
    scenario_runner = getattr(testing_scenario, 'run_scenario')
    # run the scenario testing
    scenario_runner(scene_dict)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
