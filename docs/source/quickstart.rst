Quickstart
==========


.. _installation:

Installation
------------

**First, install CARLA**. 

Download the `CARLA release <https://github.com/carla-simulator/carla/releases>`_ of version ``0.9.14`` as we experimented with this version.

Set the following environment variables:

.. code-block:: bash

   export CARLA_ROOT=/path/to/carla
   export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla/":"$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg":${PYTHONPATH}

To verify that CARLA has been correctly installed, run the following commands:

.. code-block:: bash

   cd carla/
   ./CarlaUE4.sh

**Second, setup the environment for EI-Drive**.

Clone the repository:

.. code-block:: bash

   git clone https://github.com/ucd-dare/EI-Drive
   cd EI-Drive

Create the EI-Drive environment using Conda:

.. code-block:: bash

   conda env create -f environment.yml
   conda activate EI-Drive
   cd EI-Drive
   python setup.py develop
   .setup.sh

**To run EI-Drive**, ensure that CARLA is running. You may use two terminals:

Terminal 1 for Carla:

.. code-block:: bash

   cd carla/
   ./CarlaUE4.sh

Terminal 2 for EI-Drive script:

.. code-block:: bash

   cd EI-Drive/
   python EI_Drive.py


Running a scenario
------------------

Scenarios
^^^^^^^^^

To run a specific scenario, use:

.. code-block:: bash

   python EI_Drive.py test_scenario=coop_perception_1

The command runs the script ``coop_perception_1.py``, following the configuration file in the folder ``scenario_testing/config_yaml/config.yaml``. The configuration files in EI-Drive are structured hierarchically:

.. code-block:: bash

   config.yaml/
   ├── test_scenario/              (Designate the scenario)
   │   ├── common_params
   │   ├── vehicle_perception      (Perception method)
   │   ├── vehicle_localization    (Localization method)
   │   ├── game_map
   │   ├── behavior
   │   ├── controller
   │   ├── traffic_manager
   │   └── scenario
   └── world/                      (Designate the weather)
       ├── sunny.yaml
       └── ...

The default perception method is ``oracle`` with blue bounding boxes. To enable object detection by YOLOv5, use:

.. code-block:: bash

   python EI_Drive.py test_scenario=coop_perception_1 test_scenario.vehicle_perception.perception.activate=true test_scenario.vehicle_perception.perception.model=yolo

To simplify the usage of lengthy commands, common configurations are packaged as modules. It is **recommended** to utilize these modules in the configuration file for specific scenarios. For example, to achieve the same outcome as the command mentioned above, set ``vehicle_perception: perception_true`` in the config file ``coop_perception_1.yaml``, where the config module ``perception_true.yaml`` is applied.

Cooperative Perception
^^^^^^^^^^^^^^^^^^^^^^

To enable cooperative perception, open the config file in ``test_scenario/scenario/coop_perception_1.yaml`` that defines the details of the scenario. Set ``coop_perception: true`` for the ego vehicle with ``id=0`` and the participant (RSU in this scenario) with ``id=-1``. To disable it, set ``coop_perception: false`` for the ego vehicle.

Then run the simulation:

.. code-block:: bash

   python EI_Drive.py test_scenario=coop_perception_1

.. note::

   This config file is different from ``test_scenario/coop_perception_1.yaml`` mentioned above, even though they share the same name.

Communication Latency and Errors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To enable latency and errors, open the config file in ``test_scenario/scenario/coop_perception_1.yaml`` that defines the details of the scenario. Set ``transmission_latency: true`` and ``errors: true`` for the ego vehicle with ``id=0`` and the participant (RSU in this scenario) with ``id=-1``.

Then run the simulation:

.. code-block:: bash

   python EI_Drive.py test_scenario=coop_perception_1

.. note::

   Ensure that ``coop_perception: true`` has been set for both the RSU and the ego vehicle, as latency and errors only work when data communication exists.

