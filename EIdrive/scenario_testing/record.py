import os
import re
import time
import carla
import random
import pandas as pd
from tqdm import tqdm
from multiprocessing import Pool
from multiprocessing.pool import ThreadPool as Pool

from srunner.tools.route_parser import RouteParser
from srunner.tools.route_manipulation import interpolate_trajectory
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters)
               if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


class Tracer():
    def __init__(self, opt, scenario_manager, scenario_params, route_config):
        random.seed(time.time())
        self.scenario_manager = scenario_manager
        self.route_config = route_config
        self.max_record_ticks = scenario_params.scenario_runner.max_record_ticks
        cav_params = scenario_params['scenario']['single_cav_list'][0]

        self.world = scenario_manager.world

        CarlaDataProvider.set_world(self.world)
        _, self.route = interpolate_trajectory(route_config.trajectory)
        src = self.route[0][0]
        cav_params['spawn_position'] = [src.location.x, src.location.y, src.location.z + 0.5,
                                        src.rotation.roll, src.rotation.yaw, src.rotation.pitch]
        cav_params['destination'] = [
            (waypoint[0].location.x, waypoint[0].location.y, waypoint[0].location.z) for waypoint in self.route[1:]]
        single_cav_list = self.scenario_manager.create_vehicle_manager()
        self.single_cav = single_cav_list[0]
        self.ego_vehicle = self.single_cav.vehicle

        self.spectator = self.world.get_spectator()
        self.spectator_altitude = 50
        self.spectator_bird_pitch = -90

        # Data collected from clients: vehicle control, sensor
        self.dataset_client = {}
        self.dataset_sensor = {}
        # Data collected from servers: movement
        self.dataset_server = {}
        self.EXPORT_PATH_PREFIX = f'./out_/{opt.route_id}_{opt.weather}'
        if os.path.exists(self.EXPORT_PATH_PREFIX):
            os.system(f'rm -rf {self.EXPORT_PATH_PREFIX}')

        weathers = find_weather_presets()
        self.world.set_weather(weathers[int(opt.weather)][0])
        self._init_background_actors()
        self.world.on_tick(self._capture_vehicle_movement)

    def _init_sensors(self):
        left_camera_trans = carla.Transform(
            carla.Location(x=1.0, z=2.1), carla.Rotation(yaw=-35))
        right_camera_trans = carla.Transform(
            carla.Location(x=1.0, z=2.1), carla.Rotation(yaw=35))

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '320')
        camera_bp.set_attribute('image_size_y', '240')
        camera_bp.set_attribute('sensor_tick', '0.05')
        self.left_camera = self.world.spawn_actor(
            camera_bp, left_camera_trans, attach_to=self.ego_vehicle)
        self.right_camera = self.world.spawn_actor(
            camera_bp, right_camera_trans, attach_to=self.ego_vehicle)

        self.sensor_queue = []
        self.sensor_list = [self.left_camera, self.right_camera]
        self.sensor_names = ['left_camera', 'right_camera']

        self.left_camera.listen(
            lambda data: self._camera_listener(self.sensor_names[0])(data))
        self.right_camera.listen(
            lambda data: self._camera_listener(self.sensor_names[1])(data))

    def _init_background_actors(self):
        spawn_points = self.world.get_map().get_spawn_points()
        vehicle_blueprints = [vehicle for vehicle in self.world.get_blueprint_library().filter('*vehicle*')
                                if int(vehicle.get_attribute('number_of_wheels')) > 3]
        
        for actor in self.world.get_actors():
            if 'vehicle' in actor.type_id and actor.id != self.ego_vehicle.id:
                actor.destroy()

        for _ in range(0, 10):
            vehicle = self.world.try_spawn_actor(random.choice(
                vehicle_blueprints), random.choice(spawn_points))
            if vehicle is not None:
                vehicle.set_autopilot(True)
        for actor in self.world.get_actors().filter('*vehicle*'):
            if actor.id != self.ego_vehicle.id:
                actor.set_autopilot(True)

    def _magnitute(self, vector):
        return (vector.x**2 + vector.y**2 + vector.z**2)**0.5

    def _capture_vehicle_movement(self, world_snapshot):
        if world_snapshot is None:
            print('World snapshot is currently unavailable. Skip the frame.')
            return
        if self.ego_vehicle is None:
            print('Ego vehicle is currently unavailable. Skip the frame.')
            return
        vehicle = world_snapshot.find(self.ego_vehicle.id)
        if vehicle is None:
            print('Ego vehicle snapshot is currently unavailable. Skip the frame.')
            return

        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        angular_velocity = vehicle.get_angular_velocity()
        transform = vehicle.get_transform()
        velocity_magnitude = self._magnitute(velocity)
        acceleration_magnitude = self._magnitute(acceleration)

        movement_data = {
            'frame': world_snapshot.frame,
            'location_x': transform.location.x,
            'location_y': transform.location.y,
            'location_z': transform.location.z,
            'rotation_roll': transform.rotation.roll,
            'rotation_pitch': transform.rotation.pitch,
            'rotation_yaw': transform.rotation.yaw,
            'velocity_x': velocity.x,
            'velocity_y': velocity.y,
            'velocity_z': velocity.z,
            'velocity_magnitude': velocity_magnitude,
            'acceleration_x': acceleration.x,
            'acceleration_y': acceleration.y,
            'acceleration_z': acceleration.z,
            'acceleration_magnitude': acceleration_magnitude,
            'angular_velocity_x': angular_velocity.x,
            'angular_velocity_y': angular_velocity.y,
            'angular_velocity_z': angular_velocity.z,
        }

        self.dataset_server[world_snapshot.frame] = movement_data

        return movement_data

    def _capture_vehicle_control(self, vehicle, frame):
        control = vehicle.get_control()
        control_data = {
            'frame': frame,
            'throttle': control.throttle,
            'steer': control.steer,
            'brake': control.brake,
            'hand_brake': control.hand_brake,
            'reverse': control.reverse,
            'manual_gear_shift': control.manual_gear_shift,
            'gear': control.gear,
        }

        self.dataset_client[frame] = control_data

        return control_data

    def _camera_listener(self, sensor_name):
        def _callback(sensor_data):
            self._capture_sensor_data(sensor_name, sensor_data)
        return _callback

    def _capture_sensor_data(self, sensor_name, sensor_data):
        self.sensor_queue.append((sensor_name, sensor_data))
    
    def _update_sensor_data(self, sensor_name, sensor_data):
        if not sensor_data.frame in self.dataset_sensor:
            self.dataset_sensor[sensor_data.frame] = {
                'frame': sensor_data.frame,
                sensor_name: sensor_data,
            }
            for sensor_name in self.sensor_names:
                if not sensor_name in self.dataset_sensor[sensor_data.frame]:
                    self.dataset_sensor[sensor_data.frame][sensor_name] = None
        else:
            self.dataset_sensor[sensor_data.frame][sensor_name] = sensor_data

        return self.dataset_sensor[sensor_data.frame][sensor_name]

    def _reset_bird_view(self):
        view_transform = carla.Transform()
        view_transform.location = self.ego_vehicle.get_transform().location
        view_transform.location.z = view_transform.location.z + self.spectator_altitude
        view_transform.rotation.pitch = self.spectator_bird_pitch
        self.spectator.set_transform(view_transform)

    def _get_save_path(self, sensor_name, frame):
        return os.path.join(self.EXPORT_PATH_PREFIX, f'{sensor_name}/%06d.png' % frame)

    def _save_sensor_image(self, args):
        sensor_data, save_path = args
        if sensor_data is None or save_path is None:
            return
        sensor_data.save_to_disk(save_path)

    def _save_sensor_images(self, dataset_sensor):
        print('Saving sensor images...')
        process_args = []
        for frame, data in tqdm(dataset_sensor.items()):
            for sensor_name in self.sensor_names:
                sensor_data = self.dataset_sensor[frame][sensor_name]
                save_path = self._get_save_path(sensor_name, frame)
                process_args.append((sensor_data, save_path))
        with Pool(processes=24) as pool:
            pool.map(self._save_sensor_image, process_args)

        for frame, data in tqdm(dataset_sensor.items()):
            for sensor_name in self.sensor_names:
                data[sensor_name] = self._get_save_path(sensor_name, frame)

        print('Sensor images saved.')

    def export_dataset(self):
        MOVEMENT_DATA = ['frame', 'location_x', 'location_y', 'location_z',
                         'rotation_roll', 'rotation_pitch', 'rotation_yaw',
                         'velocity_x', 'velocity_y', 'velocity_z', 'velocity_magnitude',
                         'acceleration_x', 'acceleration_y', 'acceleration_z', 'acceleration_magnitude',
                         'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
        CONTROL_DATA = ['throttle', 'steer', 'brake', 'hand_brake', 'reverse',
                        'manual_gear_shift', 'gear']
        SENSOR_DATA = self.sensor_names
        
        print('Client frame count:', len(self.dataset_server))
        print('Server frame count:', len(self.dataset_client))
        print('Sensor frame count:', len(self.sensor_queue))
        
        for (sensor_name, sensor_data) in self.sensor_queue:
            self._update_sensor_data(sensor_name, sensor_data)

        self._save_sensor_images(self.dataset_sensor)

        print('Saving dataset...')
        df = pd.DataFrame(columns=MOVEMENT_DATA+CONTROL_DATA+SENSOR_DATA)
        for frame, data in tqdm(self.dataset_client.items()):
            if frame in self.dataset_server and frame in self.dataset_sensor:
                data = {
                    **self.dataset_server[frame], **self.dataset_client[frame], **self.dataset_sensor[frame]}
                df = df.append(data, ignore_index=True)

        df.to_csv(os.path.join(self.EXPORT_PATH_PREFIX, 'dataset.csv'), index=False)
        print('Dataset exported.')

    def trace_route(self):
        key_listener = KeyListener()
        key_listener.start()
        self._init_sensors()

        timestep = 0

        while True:
            if key_listener.keys['esc']:
                return

            self.scenario_manager.tick()
            self._reset_bird_view()

            world_snapshot = self.world.get_snapshot()
            self._capture_vehicle_control(
                self.ego_vehicle, world_snapshot.frame)

            self.single_cav.update_info()
            control = self.single_cav.run_step()
            self.single_cav.vehicle.apply_control(control)

            timestep += 1

            if (self.max_record_ticks > 0 and timestep == self.max_record_ticks):
                print(f'{self.EXPORT_PATH_PREFIX} max record ticks reached')
                break


def run_scenario(opt, scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None
    tracer = None

    try:
        route_config = RouteParser.parse_routes_file(
            scenario_params.scenario_runner.routes_config,
            None,
            opt.route_id)[0]
        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=route_config.town,
                                                   cav_world=cav_world)

        tracer = Tracer(opt, scenario_manager, scenario_params, route_config)
        tracer.trace_route()

    finally:
        if tracer != None:
            tracer.export_dataset()
            
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.close()
        print("Destroyed scenario_manager")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
        