# EI-Drive

Edge AI Drive (EI-Drive) is an open-source research/engineering platform integrated with a 3D autonomous driving physical simulation engine and a variety of autonomous reinforcement learning-based driving support modules. EI-drive is the first open-source platform that provides the end-to-end simulator with edge computing capability for multiple computational tasks in autonomous driving.

In collaboration with the UCLA OpenCDA research team, EI-Drive, as an open-source project, is designed and built upon the OpenCDA platform. Besides the functionality OpenCDA provides, we also offer some unique features to the autonomous driving community to support more advanced simulation testing of reinforcement learning-based autonomous algorithms.

The key features of EI-Drive are:
* <strong>Dynamic Deadline-driven Edge Computing</strong>: The edge-computing module is a novel task-scheduling model that centralizes task management and allows applications to conduct their computation in different edge units with different computational capacities, together with the potential transmission latency and noise. We employ the idea that the deadline varies with changes of the environment and the running time of tasks varies with the computational complexity of different models, thereby designing an environment-dependent dynamic deadline estimation model for the downstream tasks and proactively selecting proper learning models and computational edge units.
* <strong>Reinforcement Learning-based Behavior Planning Module</strong>: EI-drive integrates the traditional autonomous driving planning modules with RL-based approaches to solve the behavior planning problem.
* <strong>Realistic Sensing and Perception Module</strong>: Besides the sensing and perception system in CARLA and OpenCDA, EI-drive provides various ways to simulate the noise and latency in different sensing modes based on the empirical results. EI-drive also provides various approaches to fuse heterogeneous sensing sources and representative approaches to simplified high-dimensional perception data.
* <strong>RL-friendly Support</strong>: EI-drive provides multiple RL-friendly modules to support the execution and evaluation of different RL algorithms. For example, EI-drive provides a data center module to record the trajectories of agents. All collected data can be stored in various formats and easily be queried for downstream applications such as offline RL and inverse RL. EI-drive also provides a human-control module to collect human-driven behavior trajectories for studying mixed-road driving.


## Contributors

### Lab Principal Investigator:
- Prof. Junshan Zhang

### Project Lead: <br>
 - Dr. Wei Shao ([linkedin](https://www.linkedin.com/in/wei-shao-94972295?original_referer=https%3A%2F%2Fwww.google.com%2F), [github](https://github.com/swsamleo))  <br>

### Team Members:
 - Chengyu Zhou
