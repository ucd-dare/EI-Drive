import carla
import pandas as pd
from tqdm import tqdm
from multiprocessing import Pool
from multiprocessing.pool import ThreadPool as Pool

from scenario_runner.srunner.tools.route_parser import RouteParser
from scenario_runner.srunner.tools.route_manipulation import interpolate_trajectory

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener


class Tracer():
    def __init__(self, scenario_manager, scenario_params, route_config):
        self.scenario_manager = scenario_manager
        self.route_config = route_config
        self.max_record_ticks = scenario_params.scenario_runner.max_record_ticks
        vehicle_config = scenario_params['scenario']['vehicle_list'][0]

        self.world = scenario_manager.world
        self.world.on_tick(self._capture_vehicle_movement)
        _, self.route = interpolate_trajectory(
            self.world, route_config.trajectory)
        src = self.route[0][0]

        vehicle_config['spawn_position'] = [src.location.x, src.location.y, src.location.z + 0.5,
                                            src.rotation.roll, src.rotation.yaw, src.rotation.pitch]
        vehicle_config['destination'] = [
            (waypoint[0].location.x, waypoint[0].location.y, waypoint[0].location.z) for waypoint in self.route[1:]]

        vehicle_list = self.scenario_manager.create_vehicle_agent()
        self.single_vehicle = vehicle_list[0]
        self.ego_vehicle = self.single_vehicle.vehicle

        self.spectator = self.world.get_spectator()
        self.spectator_altitude = 50
        self.spectator_bird_pitch = -90

        # Data collected from clients: vehicle control, sensor
        self.dataset_client = {}
        self.dataset_sensor = {}
        # Data collected from servers: movement
        self.dataset_server = {}

    def _init_sensors(self):
        left_camera_trans = carla.Transform(
            carla.Location(x=0.5, z=2.1), carla.Rotation(yaw=-35))
        right_camera_trans = carla.Transform(
            carla.Location(x=0.5, z=2.1), carla.Rotation(yaw=35))

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('sensor_tick', '0.5')
        self.left_camera = self.world.spawn_actor(
            camera_bp, left_camera_trans, attach_to=self.ego_vehicle)
        self.right_camera = self.world.spawn_actor(
            camera_bp, right_camera_trans, attach_to=self.ego_vehicle)

        self.sensor_list = [self.left_camera, self.right_camera]
        self.sensor_names = ['left_camera', 'right_camera']

        self.left_camera.listen(
            lambda data: self._camera_listener(self.sensor_names[0])(data))
        self.right_camera.listen(
            lambda data: self._camera_listener(self.sensor_names[1])(data))

    def _magnitute(self, vector):
        return (vector.x ** 2 + vector.y ** 2 + vector.z ** 2) ** 0.5

    def _capture_vehicle_movement(self, world_snapshot):
        if world_snapshot is None:
            print('World snapshot is currently unavailable. Skip the frame.')
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
        if not sensor_data.frame in self.dataset_sensor:
            self.dataset_sensor[sensor_data.frame] = {
                'frame': sensor_data.frame,
                sensor_name: sensor_data,
            }
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
        return f'./out/{sensor_name}/%06d.png' % frame

    def _save_sensor_image(self, args):
        sensor_data, save_path = args
        sensor_data.save_to_disk(save_path)

    def _save_sensor_images(self, dataset_sensor):
        print('Saving sensor images...')
        process_args = []
        for frame, data in tqdm(dataset_sensor.items()):
            for sensor_name in self.sensor_names:
                sensor_data = self.dataset_sensor[frame][sensor_name]
                save_path = self._get_save_path(sensor_name, frame)
                process_args.append((sensor_data, save_path))
        with Pool() as pool:
            pool.map(self._save_sensor_image, process_args)

        for frame, data in tqdm(dataset_sensor.items()):
            for sensor_name in self.sensor_names:
                data[sensor_name] = self._get_save_path(sensor_name, frame)

        print('Sensor images saved.')

    def _export_dataset(self):
        MOVEMENT_DATA = ['frame', 'location_x', 'location_y', 'location_z',
                         'rotation_roll', 'rotation_pitch', 'rotation_yaw',
                         'velocity_x', 'velocity_y', 'velocity_z', 'velocity_magnitude',
                         'acceleration_x', 'acceleration_y', 'acceleration_z', 'acceleration_magnitude',
                         'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
        CONTROL_DATA = ['throttle', 'steer', 'brake', 'hand_brake', 'reverse',
                        'manual_gear_shift', 'gear']
        SENSOR_DATA = self.sensor_names

        self._save_sensor_images(self.dataset_sensor)

        print('Saving dataset...')
        df = pd.DataFrame(columns=MOVEMENT_DATA + CONTROL_DATA + SENSOR_DATA)
        for frame, data in tqdm(self.dataset_client.items()):
            if frame in self.dataset_server and frame in self.dataset_sensor:
                data = {
                    **self.dataset_server[frame], **self.dataset_client[frame], **self.dataset_sensor[frame]}
                df = df.append(data, ignore_index=True)

        df.to_csv(f'./out/dataset.csv', index=False)
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

            self.single_vehicle.update_info()
            control = self.single_vehicle.run_step()
            self.single_vehicle.vehicle.apply_control(control)

            timestep += 1

            if (not self.max_record_ticks <= 0 and timestep == self.max_record_ticks):
                break

        self._export_dataset()


def run_scenario(opt, scenario_params):
    scenario_runner = None
    gameworld = None

    try:
        route_config = RouteParser.parse_routes_file(
            scenario_params.scenario_runner.routesConfig,
            None,
            scenario_params.scenario_runner.routeId)[0]

        # Create game world
        gameworld = sim_api.GameWorld(scenario_params,
                                      opt.apply_ml,
                                      opt.version,
                                      map_name=route_config.town)

        tracer = Tracer(gameworld, scenario_params, route_config)
        tracer.trace_route()

    finally:
        print("Destroyed ml_model")
        if gameworld is not None:
            gameworld.close()
        print("Destroyed gameworld")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
