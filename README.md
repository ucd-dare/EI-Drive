# 📷 Cooperative Perception with Edge Communication: EI-Drive 🚕

<div align="center">
    <a href="https://ei-drive-docs.readthedocs.io/en/latest/#">
        <img src="https://img.icons8.com/nolan/32/api.png" alt="CarDreamer API Documents" />
        API Documents
    </a>
    &nbsp;|&nbsp;
    <a href="https://arxiv.org/abs/2412.09782">
        <img src="https://img.icons8.com/?size=32&id=48326&format=png" alt="Pre-print" />
        ArXiv Pre-print
    </a>
    &nbsp;|&nbsp;
    <a href="https://ucd-dare.github.io/eidrive.github.io/">
        <img src="https://img.icons8.com/?size=32&id=X-WB1cntO5xU&format=png&color=000000" alt="Project Page" />
        Project Page
    </a>

</div>


______________________________________________________________________
<!-- 
> \[!NOTE\]
>
> - **August 2024:** Support transmission error in intention sharing. -->


## **Looking for an advanced autonomous driving platform that seamlessly simulates cooperative perception and realistic communication models? Choose EI-Drive!**

Integrating cooperative perception with realistic communication models, EI-Drive allows to explore the impact of communication latency and errors on not only cooperative perception but also the overall performance of autonomous driving.

A streamlined implementation with an end-to-end pipeline and built-in scenarios. Highly customizable components allow you to tailor your experiments.

Clear visualization makes the results highly visible and easy to interpret.

## 🚗 Edge-Intelligence Autonomous Driving Platform

An open-source platform provides solution for joint simulation of edge communication and cooperative perception, aimed at safe and efficient cooperative driving automation.

- 🌞 **AV pipeline**: An end-to-end pipeline encompassing environment, sensing, perception, planning, and control.
- 📸 **Cooperative perception**: Flexible cooperative perception with customizable agents, methods, tasks, and visualization.
- 📡 **Communication model**: Realistic communication models with latency and errors that interacts seamlessly with the perception module.

![EI-Drive framework](.assets/framework.png)

**Check our paper here! [Paper link](https://arxiv.org/abs/2412.09782)**

## 🎯 Experiment Demo

### Cooperative perception
EI-Drive has various built-in scenarios tailored for cooperative perception experiments, where the spectator vehicles and RSUs share perception information with the ego vehicle. Multiple cooperative perception tasks include collision avoidance and traffic flow detection.

The perception results have significant influences on the ego vehicle's behavior. The ego vehicle benefits from cooperative perception with less blind spots (rows 1 & 2) and wider detection range (row 3).

<table style="border-collapse: collapse;">
  <tr style="border: none;">
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/6_coop2_coop_CarPOV.gif" alt="Gif 1" style="height: 180px;">
      <br><b>Ego vehicle</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none;">➕</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/6_coop2_coop_CameraPOVgif.gif" alt="Gif 2" style="height: 180px;">
      <br><b>Spectator vehicle</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none;">➡️</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/6_coop2_coop_POV.gif" alt="Gif 3" style="height: 180px;">
      <br><b>Cooperative perception</b>
    </td>
  </tr>
</table>

<table style="border-collapse: collapse;">
  <tr style="border: none;">
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/10_coop6_both_BlackCarPOV.gif" alt="Gif 1" style="height: 180px;">
      <br><b>Ego vehicle</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none;">➕</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/10_coop6_both_CameraPOV.gif" alt="Gif 2" style="height: 180px;">
      <br><b>RSU</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none;">➡️</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/10_coop6_bothPOV.gif" alt="Gif 3" style="height: 180px;">
      <br><b>Cooperative perception</b>
    </td>
  </tr>
</table>

<table style="border-collapse: collapse;">
  <tr style="border: none;">
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/8_coop4_coop_BlackCarPOV.gif" alt="Gif 1" style="height: 180px;">
      <br><b>Ego vehicle</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none; padding: 17px;">➕</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/8_coop4_coop_TopCameraPOV.gif" alt="Gif 2" style="height: 180px;">
      <br><b>RSU</b>
    </td>
    <td align="center" style="font-size: 20px; line-height: 1.2; font-weight: bold; border: none;">➡️</td>
    <td align="center" style="border: none; padding: 0;">
      <img src=".assets/8_coop4_coop_BirdPOV.gif" alt="Gif 3" style="height: 180px;">
      <br><b>Cooperative perception</b>
    </td>
  </tr>
</table>


### Communication latency and errors
EI-Drive has great flexibility to simply apply communication latency and errors to any perception processes, allowing the researches across both communication and autonomous driving. The results show that communication latency and errors sinificantly impair the performance of cooperative perception.

<table style="border-collapse: collapse; width: 100%;">
  <tr style="border: none;">
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/5_coop1_coop_POV.gif" alt="Gif 1" style="height: 180px;">
      <br><b>Cooperative perception</b>
      <br>✅
    </td>
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/5_coop1_coopError_POV.gif" alt="Gif 2" style="height: 180px;">
      <br><b>Cooperative perception + Communication errors</b>
      <br>❌
    </td>
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/5_coop1_coopLatency_POV.gif" alt="Gif 3" style="height: 180px;">
      <br><b>Cooperative perception + Communication latency</b>
      <br>❌
    </td>
  </tr>
</table>

<table style="border-collapse: collapse; width: 100%;">
  <tr style="border: none;">
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/8_server/8_coop4_coop_POV.gif" alt="Gif 1" style="height: 180px;">
      <br><b>Cooperative perception</b>
    </td>
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/8_server/8_coop4_coopError_BirdPOV.gif" alt="Gif 2" style="height: 180px;">
      <br><b>Cooperative perception + Communication errors</b>
    </td>
    <td align="center" valign="top" style="border: none; padding: 0px 10px;">
      <img src=".assets/8_server/8_coop4_coopLatency_BirdPOV.gif" alt="Gif 3" style="height: 180px;">
      <br><b>Cooperative perception + Communication latency</b>
    </td>
  </tr>
</table>




### Perception methods and multi-modality
EI-Drive supports multiple perception methods and multi-modal sensor inputs, greatly enriches the experiment settings.
<table style="border-collapse: collapse; width: 100%;">
  <tr style="border: none;">
    <td align="center" style="border: none; padding: 0px 10px;">
      <img src=".assets/1_OvertakeImg_Multimodel.png" alt="Gif 1" style="height: 220px;">
      <br><b>Multi-modal inputs</b>
    </td>
    <td align="center" style="border: none; padding: 0px 10px;">
      <img src=".assets/1_OvertakeImg_Perception.png" alt="Gif 2" style="height: 220px;">
      <br><b>Multiple perception methods</b>
    </td>
  </tr>
</table>









## 💻 Prerequisites

**First, CARLA needs to be installed**. Download [CARLA release](https://github.com/carla-simulator/carla/releases) of version ``0.9.14`` as we experiemented with this version.

Set the following environment variables:
```bash
export CARLA_ROOT=/path/to/carla 
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla/":"$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg":${PYTHONPATH}
```

To verify that CARLA has been correctly installed, run the command:
```bash
cd carla/
./CarlaUE4.sh
```

**Second, setup the environment for EI-Drive**. Clone the repository:

```bash
git clone https://github.com/ucd-dare/EI-Drive
cd EI-Drive
```

Create the EI-Drive environment using conda:

```bash
conda env create -f environment.yml
conda activate EI-Drive
```

**To run EI-Drive**, make sure the CARLA is running in the meanwhile. You may use two terminals:
```bash
cd carla/
./CarlaUE4.sh
```
```bash
cd EI-Drive/
python EI_Drive.py
```

## 🏇 Quick Start

### Scenarios
To run a specific scenario, use

```bash
python EI_Drive.py test_scenario=coop_perception_1
```

The command runs the script ``coop_perception_1.py``, following the configuration file in the folder ``scenario_testing/config_yaml/config.yaml``. The config files in EI-Drive are structured hierarchically:

```bash
config.yaml/
├── test_scenario/              (Designate the scenario)
|   ├── common_params
|   ├── vehicle_perception      (Perception method)
|   ├── vehicle_localization    (Localization method)
|   ├── game_map
|   ├── behavior
|   ├── controller
|   ├── traffic_manager
|   └── scenario
└── world/                      (Designate the weather)
    ├── sunny.yaml
    └── ... 

```

The default perception method is oracle with blue bounding boxes, to enable object detection by YOLOv5

```bash
python EI_Drive.py test_scenario=coop_perception_1 test_scenario.vehicle_perception.perception.activate=true test_scenario.vehicle_perception.perception.model=yolo
```

To simplify the usage of lengthy commands, we have packaged common configurations as modules. It is **recommended** to utilize these modules in the configuration file for specific scenarios. For instance, to achieve the same outcome as the command mentioned above, you can set ``vehicle_perception: perception_true`` in the config file ``coop_perception_1.yaml``, where config module ``perception_true.yaml`` is applied.

### Cooperative Perception
To enable cooperative perception, open the config file in ``test_scenario/scenario/coop_perception_1.yaml`` that defines the details of the scenario. Set ``coop_perception: true`` for the ego vehicle with ``id=0`` and the participant (RSU in this scenario) with ``id=-1``. To disable it, set ``coop_perception: false`` for the ego vehicle.

Then run the simulation
```bash
python EI_Drive.py test_scenario=coop_perception_1
```

💡 Please note that this config file is a different file with ``test_scenario/coop_perception_1.yaml`` abovementioned, even they have the same name.

### Communication Latency and Errors
To enable latency and errors, open the config file in ``test_scenario/scenario/coop_perception_1.yaml`` that defines the details of the scenario. Set ``transmission_latency: true`` and ``errors: true`` for the ego vehicle with ``id=0`` and the participant (RSU in this scenario) with ``id=-1``.

Then run the simulation
```bash
python EI_Drive.py test_scenario=coop_perception_1
```

💡 Please ensure ``coop_perception: true`` has been set for the RSU and the ego vehicle, since the latency and errors work when the data communication exists.

## 🌟 Citation

If you find this repository useful, please cite this paper:

**[Paper link]()**
```
@article{zhou2024ei,
  title={EI-Drive: A Platform for Cooperative Perception with Realistic Communication Models},
  author={Zhou, Hanchu and Xie, Edward and Shao, Wei and Gao, Dechen and Dong, Michelle and Zhang, Junshan},
  journal={arXiv preprint arXiv:2412.09782},
  year={2024}
}
```

## 👥 Contributors

Special thanks to the community for your valuable contributions and support in making EI-Drive better for everyone!

<!-- readme: contributors -start -->
<table>
	<tbody>
		<tr>
            <td align="center">
                <a href="https://github.com/HanchuZhou">
                    <img src="https://avatars.githubusercontent.com/u/99316745?v=4" width="100;" alt="HanchuZhou"/>
                    <br />
                    <sub><b>Hanchu Zhou</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/edheadd">
                    <img src="https://avatars.githubusercontent.com/u/106692483?v=4" width="100;" alt="Ed Xie"/>
                    <br />
                    <sub><b>Ed Xie</b></sub>
                </a>
            </td>
            <td align="center">
                <a href="https://github.com/gaodechen">
                    <img src="https://avatars.githubusercontent.com/u/2103562?v=4" width="100;" alt="gaodechen"/>
                    <br />
                    <sub><b>GaoDechen</b></sub>
                </a>
            </td>
		</tr>
	<tbody>
</table>
<!-- readme: contributors -end -->
