## Route-based Dataset Collection

Offline dataset is produced by a driving expert have access to environment groundtruth. We use a PID controller by default as the expert.

We have a record script that collect a single route given.

```bash
python EI_Drive.py -t record
```

To collect multiple routes, a ```collect.sh``` bash script is provided within ```./EIdrive/dataset_utils```:

```
bash collect.sh
```

The scripts will monitor the collection process and Carla process and restart them whenever they are broken or timing out.

An interative demo is provided for dataset preprocessing and animation/visualization in ```./EIdrive/dataset_utils/preprocess.ipynb```.

### Route-based Configurations

Configure pre-defined routes using a series of waypoints in your ```.xml``` file along with map and weather settings.

```bash
<routes>
  <route id="0" town="Town01">
    <weather
      cloudiness="0"
      precipitation="0"
      precipitation_deposits="0"
      wind_intensity="0"
      sun_azimuth_angle="0"
      sun_altitude_angle="70"
      fog_density="0"
      fog_distance="0"
      wetness="0"
    />
    <waypoint pitch="360.0" roll="0.0" x="338.7027893066406" y="226.75003051757812"
      yaw="269.9790954589844" z="0.0" />
    <waypoint pitch="360.0" roll="0.0" x="321.98931884765625" y="194.67242431640625"
      yaw="179.83230590820312" z="0.0" />
</routes>
```


Then, configure ```record.py``` for the specific route file.

```bash
scenario_runner:
  routes_config: '/path/to/your/routes.xml'
  # Absolute path needed here
  route_id: '0'
  # Set 0 to record permanently
  max_record_ticks: 7000
```