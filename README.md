# 📷 Cooperative Perception with Edge Communication: EI-Drive 🚕

<div align="center">
    <a href="">
        <img src="https://img.icons8.com/nolan/32/api.png" alt="CarDreamer API Documents" />
        EI-Drive API Documents
    </a>
    &nbsp;|&nbsp;
    <a href="">
        <img src="https://img.icons8.com/?size=32&id=48326&format=png" alt="ArXiv Pre-print" />
        ArXiv Pre-print
    </a>
    &nbsp;|&nbsp;
    <a href="">
        <img src="https://img.icons8.com/?size=32&id=X-WB1cntO5xU&format=png&color=000000" alt="Project Page" />
        Project Page
    </a>

</div>

______________________________________________________________________

Experience the seamless integration of **cooperative perception** and **edge communication** in autonomous driving simulation.
<!-- 
> \[!NOTE\]
>
> - **August 2024:** Support transmission error in intention sharing. -->


## **Looking for a full-stack autonomous driving platform that seamlessly simulates cooperative perception and edge communication? Choose EI-Drive!**

Integrating cooperative perception with edge communication, EI-Drive allows the exploration how communication latency and error affect not only cooperative perception but also the overall performance of autonomous vehicles.

A streamlined implementation with a full-stack pipeline and built-in scenarios. Highly customizable components allow you to tailor your experiments.

Clear visualization makes the results highly visible and easy to interpret.

## 🚗 Open-Source Edge-Intelligence Autonomous Driving Platform

An open-source platform provides solution for joint simulation of edge communication and cooperative perception, aimed at safe and efficient cooperative driving automation.

- 🏙️ **Full-stack AV**: A full pipeline encompassing environment, sensing, perception, planning, and control.
- 🔧 **Cooperative perception**: Flexible cooperative perception with customizable agents, methods, tasks, and visualization.
- 🌍 **Transmission model**: Simulate the key characteristics of data transmission between the edge agents, interacting seamlessly with the perception module.

**Documentation:** [EI-Drive API Documents]().

**Looking for more techincal details? Check our report here! [Paper link]()**

## Prerequisites

Clone the repository:

```bash
git clone https://github.com/ucd-dare/EI-Drive
cd EI-Drive
```

Download [CARLA release](https://github.com/carla-simulator/carla/releases) of version ``0.9.14`` as we experiemented with this version.

Set the following environment variables:
```bash
export CARLA_ROOT=/path/to/carla 
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla/":"$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg":${PYTHONPATH}
```

And run this command in your terminal:
```bash
source ~/.bashrc
```

Create the EI-Drive environment using conda:

```bash
conda env create -f environment.yml
conda activate EI-Drive
python setup.py develop
```