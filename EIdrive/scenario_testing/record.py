import carla
import pandas as pd

from srunner.tools.route_parser import RouteParser
from srunner.tools.route_manipulation import interpolate_trajectory

import EIdrive.scenario_testing.utils.sim_api as sim_api
from EIdrive.core.common.cav_world import CavWorld
from EIdrive.scenario_testing.utils.keyboard_listener import KeyListener

from queue import Queue


class Tracer():
    def __init__(self, scenario_manager, scenario_params, route_config):
        self.scenario_manager = scenario_manager
        self.route_config = route_config

        self.world = scenario_manager.world
        cav_params = scenario_params['scenario']['single_cav_list'][0]
        _, self.route = interpolate_trajectory(
            self.world, route_config.trajectory)

        src = self.route[0][0]
        cav_params['spawn_position'] = [src.location.x, src.location.y, src.location.z + 0.5,
                                        src.rotation.roll, src.rotation.yaw, src.rotation.pitch]
        cav_params['destination'] = [
            # (waypoint.x, waypoint.y, waypoint.z) for waypoint in route_config.trajectory[1:]]
            (waypoint[0].location.x, waypoint[0].location.y, waypoint[0].location.z) for waypoint in self.route[1:]]
        single_cav_list = self.scenario_manager.create_vehicle_manager()
        self.single_cav = single_cav_list[0]
        self.ego_vehicle = self.single_cav.vehicle

        self.spectator = self.world.get_spectator()
        self.spectator_altitude = 50
        self.spectator_bird_pitch = -90

        self.dataset = {}
        self.sensor_queue = Queue()

    def _init_cameras(self):
        left_camera_trans = carla.Transform(
            carla.Location(z=2.1), carla.Rotation(yaw=35))
        right_camera_trans = carla.Transform(
            carla.Location(z=2.1), carla.Rotation(yaw=-35))

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # camera_bp.fov = 120
        self.left_camera = self.world.spawn_actor(
            camera_bp, left_camera_trans, attach_to=self.ego_vehicle)
        self.right_camera = self.world.spawn_actor(
            camera_bp, right_camera_trans, attach_to=self.ego_vehicle)

        self.sensor_list = [self.left_camera, self.right_camera]
        self.sensor_name = ['left_camera', 'right_camera']

        self.left_camera.listen(
            lambda data: self._camera_listener(self.sensor_name[0])(data))
        self.right_camera.listen(
            lambda data: self._camera_listener(self.sensor_name[1])(data))

    def _magnitute(self, vector):
        return (vector.x**2 + vector.y**2 + vector.z**2)**0.5

    def _capture_vehicle_info(self, sensor_data, vehicle):
        velocity = vehicle.get_velocity()
        acceleration = vehicle.get_acceleration()
        angular_velocity = vehicle.get_angular_velocity()
        transform = vehicle.get_transform()
        control = vehicle.get_control()

        velocity_magnitude = self._magnitute(velocity)
        acceleration_magnitude = self._magnitute(acceleration)

        image_info = {
            'frame': sensor_data.frame,
            'timestamp': sensor_data.timestamp,
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
            'throttle': control.throttle,
            'steer': control.steer,
            'brake': control.brake,
            'hand_brake': control.hand_brake,
            'reverse': control.reverse,
            'manual_gear_shift': control.manual_gear_shift,
            'gear': control.gear,
            'left_camera': None,
            'right_camera': None,
        }
        return image_info

    def _camera_listener(self, sensor_name):
        def _callback(sensor_data):
            self.sensor_queue.put((sensor_name, sensor_data))
        return _callback

    def _reset_bird_view(self):
        view_transform = carla.Transform()
        view_transform.location = self.ego_vehicle.get_transform().location
        view_transform.location.z = view_transform.location.z + self.spectator_altitude
        view_transform.rotation.pitch = self.spectator_bird_pitch
        self.spectator.set_transform(view_transform)

    def _export_df(self):
        COLUMNS = ['frame', 'timestamp', 'location_x', 'location_y', 'location_z',
                   'rotation_roll', 'rotation_pitch', 'rotation_yaw',
                   'velocity_x', 'velocity_y', 'velocity_z', 'velocity_magnitude',
                   'acceleration_x', 'acceleration_y', 'acceleration_z', 'acceleration_magnitude',
                   'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                   'throttle', 'steer', 'brake', 'hand_brake', 'reverse',
                   'manual_gear_shift', 'gear'] + self.sensor_name
        df = pd.DataFrame(columns=COLUMNS)
        for _, value in self.dataset.items():
            df = df.append(value, ignore_index=True)
        df.to_csv('./out/data.csv', index=False)

    def trace_route(self):
        key_listener = KeyListener()
        key_listener.start()
        self._init_cameras()

        timestep = 0

        while True:
            if key_listener.keys['esc']:
                return

            self.scenario_manager.tick()
            self._reset_bird_view()

            world_snapshot = self.world.get_snapshot()
            print(f'World Frame: {world_snapshot.frame}')

            for actor_snapshot in world_snapshot:
                if actor_snapshot.id != self.ego_vehicle.id:
                    continue
            for _ in range(len(self.sensor_list)):
                if self.sensor_queue.empty():
                    continue
                (sensor_name, sensor_data) = self.sensor_queue.get(True, 1.0)
                self.dataset[sensor_data.frame] = self._capture_vehicle_info(
                    sensor_data, self.ego_vehicle)
                save_path = f'./out/{sensor_name}/%06d.png' % sensor_data.frame
                self.dataset[sensor_data.frame][sensor_name] = save_path
                sensor_data.save_to_disk(save_path)

            self.single_cav.update_info()
            control = self.single_cav.run_step()
            self.single_cav.vehicle.apply_control(control)

            timestep += 1
            if (timestep == 200):
                break

        self._export_df()


def run_scenario(opt, scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None

    try:
        route_config = RouteParser.parse_routes_file(
            scenario_params.scenario_runner.routesConfig,
            None,
            scenario_params.scenario_runner.routeId)[0]
        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=route_config.town,
                                                   cav_world=cav_world)

        tracer = Tracer(scenario_manager, scenario_params, route_config)
        tracer.trace_route()

    finally:
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.close()
        print("Destroyed scenario_manager")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
