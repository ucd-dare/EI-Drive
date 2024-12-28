Configuration
=============

Scenario Configuration Overview
------------------------

The general scenario config files, such as **demo.yaml**, define various parameters for the simulation. Below is a detailed explanation of each item:

1. **common_params**

   Purpose:  
   Defines common parameters shared across all modules.

   Modules:  

   - ``common_param``  
     The basic version of parameters.  
   - ``common_params_edge_on``  
     Parameters for edge units being activated.  

2. **vehicle_perception**

   Purpose:  
   Configures how the vehicle perceives its environment.

   Modules:  

   - ``perception_false``  
     Object positions are retrieved directly from the server.  
   - ``perception_true``  
     Object positions are retrieved through perception.  

3. **vehicle_localization**

   Purpose:  
   Manages vehicle localization for determining its precise position.

   Modules:  

   - ``localization_false``  
     Ego position is retrieved from the server.  
   - ``localization_true``  
     Ego position is retrieved via localization algorithms.  

4. **game_map**

   Purpose:  
   Controls the traffic map state.

   Modules:  

   - ``map_activated``  
     Activates the traffic map.  
   - ``map_deactivated``  
     Deactivates the traffic map.  

5. **behavior**

   Purpose:  
   Configures vehicle behavior, including planning and maximum speed.

   Modules:  

   - ``behavior_default``  
     Enables automated planning for vehicles.  
   - ``behavior_demo_manual``  
     Uses manually provided trajectories from files.  
   - ``behavior_scenario``  
     Configures the behavior for OpenScenario.  

6. **controller**

   Purpose:  
   Handles vehicle control based on the planned trajectory.

   Modules:  

   - ``pid_controller``  
     Uses a PID controller for vehicle movement.  

7. **traffic_manager**

   Purpose:  
   Manages interactions with traffic, including lane changes and traffic light rules.

   Modules:  

   - ``lane_change_false``  
     Disables automatic lane changes.  
   - ``lane_change_true``  
     Enables automatic lane changes.  

8. **scenario**

   Purpose:  
   Defines testing scenarios with pre-defined destinations and perception info.

   Modules:  

   The name should be consistent with the name of the scenario.

Behavior Configuration
------------------------

The behavior config files, such as **behavior_coop_perception.yaml**, specify detailed configurations for vehicle behavior and planning.

- ``is_manually``  
  Specifies if the behavior is manual (``False`` for automated).  
- ``max_speed``  
  Maximum vehicle speed in km/h.  
- ``tailgate_speed``  
  Speed for approaching another vehicle ASAP.  
- ``max_speed_margin``  
  Margin for calculating the target speed (``max_speed - margin``).  
- ``speed_decrease``  
  Speed reduction during car-following mode for distance keeping.  
- ``safety_time``  
  Time-to-collision (TTC) threshold for reducing speed.  
- ``emergency_param``  
  Threshold for initiating an emergency stop.  
- ``ignore_traffic_light``  
  Allows ignoring traffic lights.  
- ``overtake_allowed``  
  Indicates if overtaking is allowed.  
- ``collision_time_ahead``  
  Time for collision checking.  
- ``overtake_counter_recover``  
  Cooldown period for overtaking.  
- ``sample_resolution``  
  Distance between two waypoints.  
- ``buffer_size``  
  Number of waypoints in the planning buffer.  
- ``trajectory_update_freq``  
  Frequency of trajectory point updates.  
- ``trajectory_sample_horizon``  
  Time horizon for trajectory sampling.  
- ``global_route_update_freq``  
  Frequency for updating the global route.  
- ``trajectory_dt``  
  Time step for sampling trajectory points.  
- ``min_dist``  
  Minimum distance to remove close waypoints.  
- ``debug``  
  Enables visual debugging of waypoints.  
- ``debug_trajectory``  
  Enables visual debugging of trajectory points and paths.  


Controller Configuration
----------------------------

The controller config details the parameters for PID controller.

- ``controller.type``  
  Specifies the controller type. Supported: ``PID``.
- ``controller.args.lat.k_p``  
  Proportional gain for lateral control.
- ``controller.args.lat.k_d``  
  Derivative gain for lateral control.
- ``controller.args.lat.k_i``  
  Integral gain for lateral control.
- ``controller.args.lon.k_p``  
  Proportional gain for longitudinal control.
- ``controller.args.lon.k_d``  
  Derivative gain for longitudinal control.
- ``controller.args.lon.k_i``  
  Integral gain for longitudinal control.
- ``controller.args.dt``  
  Time step for PID computation. Should match ``fixed_delta_seconds``.
- ``controller.args.max_brake``  
  Maximum braking force (0 to 1.0).
- ``controller.args.max_throttle``  
  Maximum throttle force (0 to 1.0).
- ``controller.args.max_steering``  
  Maximum steering angle in radians.
- ``controller.args.max_steering_rate``  
  Maximum rate of steering change.
- ``controller.args.time_interval``  
  Time interval for applying control signals.

Localization Configuration
--------------------------

The localization configuration file contains settings for positioning and motion tracking of the ego vehicle, including GNSS-based localization and Kalman filter parameters.

- ``localization.activate``  
  Activates localization. If ``False``, ego position is retrieved from the server.
- ``localization.dt``  
  Time step for Kalman filter. Matches ``fixed_delta_seconds``.
- ``localization.gnss.noise_alt_stddev``  
  Standard deviation of altitude noise in meters.
- ``localization.gnss.noise_lat_stddev``  
  Standard deviation of latitude noise in radians.
- ``localization.gnss.noise_lon_stddev``  
  Standard deviation of longitude noise in radians.
- ``localization.gnss.heading_direction_stddev``  
  Standard deviation of heading noise in degrees.
- ``localization.gnss.speed_stddev``  
  Standard deviation of speed noise in m/s.
- ``localization.debug_helper.show_animation``  
  Enables real-time trajectory visualization.
- ``localization.debug_helper.x_scale``  
  Multiplier for x-coordinate visualization.
- ``localization.debug_helper.y_scale``  
  Multiplier for y-coordinate visualization.

Map Configuration
-----------------

The map config defines the settings for the road map visualization.

- ``game_map.pixels_per_meter``  
  Number of pixels per meter in the map raster.
- ``game_map.raster_size``  
  Raster dimensions in pixels [width, height].
- ``game_map.lane_sample_resolution``  
  Distance between sampled waypoints along a lane.
- ``game_map.visualize``  
  Enables visualization of the map.
- ``game_map.activate``  
  Activates the map for planning and visualization.

Perception Configuration
------------------------

The perception configuration file sets parameters for sensing, including cameras, LiDAR, and the YOLO model for object detection.

- ``perception.activate``  
  Enables perception module. If ``False``, uses oracle data.
- ``perception.model``  
  Perception model type (e.g., ``yolo``).
- ``perception.camera.visualize``  
  Number of camera images to visualize (0 for none).
- ``perception.camera.num``  
  Number of cameras mounted (up to 3: frontal, left, and right).
- ``perception.lidar.visualize``  
  Enables LIDAR visualization using Open3D.
- ``perception.lidar.channels``  
  Number of LIDAR channels.
- ``perception.lidar.range``  
  Maximum range of LIDAR in meters.
- ``perception.lidar.points_per_second``  
  Number of points emitted per second.
- ``perception.lidar.rotation_frequency``  
  LIDAR rotation frequency in Hz.
- ``perception.lidar.upper_fov``  
  Upper field-of-view limit in degrees.
- ``perception.lidar.lower_fov``  
  Lower field-of-view limit in degrees.
- ``perception.lidar.dropoff_general_rate``  
  General drop-off rate for point intensity.
- ``perception.lidar.dropoff_intensity_limit``  
  Minimum intensity before dropping points.
- ``perception.lidar.dropoff_zero_intensity``  
  Intensity value treated as zero.
- ``perception.lidar.noise_stddev``  
  Standard deviation of noise in LIDAR measurements.

Traffic Manager Configuration
-----------------------------

The traffic manager configuration file controls the background vehicles, including speed adjustments, range of traffic flow, and lane-changing policies.

- ``carla_traffic_manager.sync_mode``  
  Synchronizes with the world setting.
- ``carla_traffic_manager.deterministic_mode``  
  Enables deterministic behavior for repeatability.
- ``carla_traffic_manager.deterministic_seed``  
  Seed value for deterministic mode.
- ``carla_traffic_manager.global_distance``  
  Minimum distance in meters to maintain between vehicles.
- ``carla_traffic_manager.global_speed_perc``  
  Speed adjustment relative to the default speed.
- ``carla_traffic_manager.set_osm_mode``  
  Enables OpenStreetMap (OSM) mode for navigation.
- ``carla_traffic_manager.auto_lane_change``  
  Allows automatic lane changes. Set to ``False`` in this configuration.
- ``carla_traffic_manager.random``  
  Randomizes vehicle colors and models.
- ``carla_traffic_manager.ignore_lights_percentage``  
  Percentage of vehicles ignoring traffic lights.
- ``carla_traffic_manager.vehicle_list``  
  Defines vehicles for simulation (can be a list or number).
- ``carla_traffic_manager.range``  
  Vehicle spawn range: ``[x_min, x_max, y_min, y_max, x_step, y_step]``.


World Settings
--------------

The world configuration file defines the environmental and simulation settings, including rendering, weather, and randomness.

- ``world.sync_mode``  
  Enables synchronization mode for the world.
- ``world.client_host``  
  Hostname or IP address of the CARLA client (default: ``localhost``).
- ``world.client_port``  
  Port number for the CARLA client connection (default: ``2000``).
- ``world.no_rendering_mode``  
  Disables rendering to improve simulation performance.
- ``world.fixed_delta_seconds``  
  Fixed time step for the simulation in seconds.
- ``world.seed``  
  Random seed for reproducibility.
- ``world.weather.sun_altitude_angle``  
  Sun altitude angle in degrees. ``90`` is midday, and ``-90`` is midnight.
- ``world.weather.cloudiness``  
  Cloud cover percentage. ``0`` represents a clear sky, and ``100`` represents thick clouds.
- ``world.weather.precipitation``  
  Rain intensity percentage. ``0`` means no rain, and ``100`` means the heaviest rain.
- ``world.weather.precipitation_deposits``  
  Amount of water puddles created on the road. ``0`` represents no puddles, and ``100`` represents roads completely covered with water.
- ``world.weather.wind_intensity``  
  Wind intensity percentage, influencing the behavior of rain. ``0`` means no wind.
- ``world.weather.fog_density``  
  Thickness of the fog. ``100`` is the thickest fog.
- ``world.weather.fog_distance``  
  Distance in meters at which fog begins. ``0`` means fog starts immediately.
- ``world.weather.fog_falloff``  
  Density of the fog. Higher values make the fog denser and heavier, limiting its height.
- ``world.weather.wetness``  
  Wetness level of the environment. ``0`` means completely dry conditions.

