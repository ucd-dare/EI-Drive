Build your scenario
==========

To build your customized scenario, you need to prepare one **scenario script** and two **yaml files**.

Scenario script
-------------------

To easily build the the scenario script, you can copy from ``EIdrive/scenario_testing/module_test_1.py`` for simple scenario. At the beginning of the script, the game world, background vehicles, and spectator are generated.

.. code-block:: python

    gameworld = sim_api.GameWorld(scenario_params, map_name='town06')
    vehicle_list = gameworld.create_vehicle_agent()
    spectator = gameworld.world.get_spectator()

To customized the vehicle make and color, you may use:

.. code-block:: python

    def customized_bp(world):
    """
    Provide customized vehicle blueprints.
    """
        vehicle_blueprints = []
        vehicle_model = 'vehicle.lincoln.mkz_2020'
        vehicle_blueprint = world.get_blueprint_library().find(vehicle_model)
        vehicle_blueprint.set_attribute('color', '255, 0, 0')
        vehicle_blueprints.append(vehicle_blueprint)
        return vehicle_blueprints
    
    vehicle_blueprints = customized_bp(gameworld.world)

Then the environmental information is updated to the ego vehicle pipeline. The planning and control modules produce control signals and applied to vehicle in the code:

.. code-block:: python

    for vehicle_agent in vehicle_list:
        vehicle_agent.update_info()
        sim_api.gamemap_visualize(vehicle_agent)
        control = sim_api.calculate_control(vehicle_agent)
        vehicle_agent.vehicle.apply_control(control)

For cooperative perception, you can copy the template from ``EIdrive/scenario_testing/coop_perception_1.py``. In the cooperative perception script, ``pygame`` is used for better visualization. The late fusion method is implemented and visualized by:

.. code-block:: python

    bbx_list = manage_bbx_list(vehicle_list, rsu_list)
    control_tick_temp = bbx_visualizer.VisualizeBBX(cam, vehicles, bbx_list, t, text_viz)

Configuration files
-------------------

We use the scenario ``coop_perception_1`` as an template.

1. The config file ``EIdrive/scenario_testing/config_yaml/test_scenario/basic_movement.yaml`` needs to be copied and renamed with the same name as your new scenario script.

.. code-block:: yaml

    defaults:
     - common_params: common_param
     - vehicle_perception: perception_false
     - vehicle_localization: localization_false
     - game_map: map_activated
     - behavior: behavior_default
     - controller: pid_controller
     - traffic_manager: lane_change_false
     - scenario: coop_perception_1

This yaml file defines the configuration in a modularized way. You can switch the setting but simply replace the modules. Make sure that the name of the `scenario` should be same as the script name.

2. The config file ``EIdrive/scenario_testing/config_yaml/test_scenario/scenario/coop_perception_1.yaml`` needs to be copied and renamed with the same name as your new scenario script. This script defines the details of the scenario, including the settings of the sensors, cooperative perception, and agents.

The vehicle agents are defined by:

.. code-block:: yaml

    - id: 0
        spawn_position: [-70.4, -133.9, 0.3, 0, -90, 0 ]
        destination: [[ -9.6, 254.8, 0.3 ]]
        perception:
            coop_perception: true
            transmission_latency: false
            transmission_latency_in_sec: 0.3
            errors: false
            error_rate: 0.3
            camera:
                visualize: 0
                num: 0

The ego vehicle is defined by ``id:0``. The camera and LiDAR can be turned off by set the ``visualized: 0``. The RSU agents are defined in the similar way, while their id should be negative.

To find the coordinate of a specific point in the map, please use the locator  with WASD to control the movement and Q/E for zoom in and out:

.. code-block:: bash

   python locator.py --map town06

To enable the cooperative perception between the ego vehicle and the other agents, set ``coop_perception: true`` for all agents that in cooperative percetion. The ``transmission_latency`` and ``errors`` can be enabled to simulate realistic communication.



