# -*- coding: utf-8 -*-
"""
Dumping sensor data.
"""
import os

import cv2
import open3d as o3d
import numpy as np

from EIdrive.core.common.misc import get_speed
from EIdrive.core.sensing.perception import sensor_transformation as st
from EIdrive.scenario_testing.utils.yaml_utils import save_yaml


class DataDumper(object):
    """
    Data dumper class to save data in local disk.

    Parameters
    ----------
    perception_manager : EIdrive object
        The perception manager contains rgb camera data and lidar data.

    vehicle_id : int
        The carla.Vehicle id.

    save_time : str
        The timestamp at the beginning of the simulation.

    Attributes
    ----------
    rgb_camera : list
        A list of EIdrive.CameraSensor that containing all rgb sensor data
        of the managed vehicle.

    lidar ; EIdrive object
        The lidar manager from perception manager.

    save_folder_path : str
        The parent folder to save all data related to a specific vehicle.

    count : int
        Used to count how many steps have been executed. We dump data
        every 10 steps.

    """

    def __init__(self,
                 perception_manager,
                 vehicle_id,
                 save_time):

        self.rgb_camera = perception_manager.rgb_camera
        self.lidar = perception_manager.lidar

        self.save_time = save_time
        self.vehicle_id = vehicle_id
        self.count = 0

        current_path = os.path.dirname(os.path.realpath(__file__))
        self.save_folder_path = \
            os.path.join(current_path, '../../../data_dumping', save_time, str(self.vehicle_id))

        if not os.path.exists(self.save_folder_path):
            os.makedirs(self.save_folder_path)

    def save_data(self,
                  perception_manager,
                  localization_manager,
                  behavior_agent):
        """
        Dump data at running time.

        Parameters
        ----------
        perception_manager : EIdrive object
            OpenCDA perception manager.

        localization_manager : EIdrive object
            OpenCDA localization manager.

        behavior_agent : EIdrive object
            Open
        """
        self.count = self.count + 1

        # we ignore the first 60 steps and make it 10hz
        if self.count < 60 or self.count % 2 != 0:
            return

        self.save_rgb_image()
        self.save_lidar_points()
        self.save_yaml_file(perception_manager,
                            localization_manager,
                            behavior_agent)

    def save_rgb_image(self):
        """
        Save camera rgb images to disk.
        """
        for (i, camera) in enumerate(self.rgb_camera):
            image_name = '%06d' % camera.frame + '_' + 'camera%d' % i + '.png'
            cv2.imwrite(os.path.join(self.save_folder_path, image_name), camera.image)

    def save_lidar_points(self):
        """
        Save 3D lidar points to disk.
        """
        position = self.lidar.data[:, :-1]
        intensity = self.lidar.data[:, -1]
        intensity = np.c_[
            intensity,
            np.zeros_like(intensity),
            np.zeros_like(intensity)
        ]

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(position)
        o3d_pcd.colors = o3d.utility.Vector3dVector(intensity)

        # save in pcd file
        pcd_name = '%06d' % self.lidar.frame + '.pcd'
        o3d.io.write_point_cloud(os.path.join(self.save_folder_path,
                                              pcd_name),
                                 pointcloud=o3d_pcd,
                                 write_ascii=True)

    def save_yaml_file(self,
                       perception_manager,
                       localization_manager,
                       behavior_agent):
        """
        Save objects positions/speed, true ego position,
        predicted ego position, sensor transformations.

        Parameters
        ----------
        perception_manager : EIdrive object
            OpenCDA perception manager.

        localization_manager : EIdrive object
            OpenCDA localization manager.

        behavior_agent : EIdrive object
            OpenCDA behavior agent.
        """
        dump_yml = {}
        vehicle_dict = {}

        # dump obstacle vehicles first
        vehicle_list = perception_manager.objects['vehicles']

        for veh in vehicle_list:
            veh_carla_id = veh.carla_id
            veh_pos = veh.get_transform()
            veh_bbx = veh.bounding_box
            veh_speed = get_speed(veh)

            assert veh_carla_id != -1, "Please turn off perception active" \
                                       "mode if you are dumping data"

            vehicle_dict.update({veh_carla_id: {
                'bp_id': veh.type_id,
                'color': veh.color,
                "location": [veh_pos.location.x,
                             veh_pos.location.y,
                             veh_pos.location.z],
                "center": [veh_bbx.location.x,
                           veh_bbx.location.y,
                           veh_bbx.location.z],
                "angle": [veh_pos.rotation.roll,
                          veh_pos.rotation.yaw,
                          veh_pos.rotation.pitch],
                "extent": [veh_bbx.extent.x,
                           veh_bbx.extent.y,
                           veh_bbx.extent.z],
                "speed": veh_speed
            }})

        dump_yml.update({'vehicles': vehicle_dict})

        # dump ego pose and speed, if vehicle does not exist, then it is
        # a rsu(road side unit).
        predicted_ego_pos = localization_manager.get_ego_pos()
        true_ego_pos = localization_manager.vehicle.get_transform() \
            if hasattr(localization_manager, 'vehicle') \
            else localization_manager.true_ego_pos

        dump_yml.update({'predicted_ego_pos': [
            predicted_ego_pos.location.x,
            predicted_ego_pos.location.y,
            predicted_ego_pos.location.z,
            predicted_ego_pos.rotation.roll,
            predicted_ego_pos.rotation.yaw,
            predicted_ego_pos.rotation.pitch]})
        dump_yml.update({'true_ego_pos': [
            true_ego_pos.location.x,
            true_ego_pos.location.y,
            true_ego_pos.location.z,
            true_ego_pos.rotation.roll,
            true_ego_pos.rotation.yaw,
            true_ego_pos.rotation.pitch]})
        dump_yml.update({'ego_speed':
                             float(localization_manager.get_ego_spd())})

        # dump lidar sensor coordinates under world coordinate system
        lidar_transformation = self.lidar.sensor.get_transform()
        dump_yml.update({'lidar_pose': [
            lidar_transformation.location.x,
            lidar_transformation.location.y,
            lidar_transformation.location.z,
            lidar_transformation.rotation.roll,
            lidar_transformation.rotation.yaw,
            lidar_transformation.rotation.pitch]})

        # dump camera sensor coordinates under world coordinate system
        for (i, camera) in enumerate(self.rgb_camera):
            camera_param = {}
            camera_transformation = camera.sensor.get_transform()
            camera_param.update({'cords': [
                camera_transformation.location.x,
                camera_transformation.location.y,
                camera_transformation.location.z,
                camera_transformation.rotation.roll,
                camera_transformation.rotation.yaw,
                camera_transformation.rotation.pitch
            ]})

            # dump intrinsic matrix
            camera_intrinsic = st.get_camera_intrinsic(camera.sensor)
            camera_intrinsic = self.matrix2list(camera_intrinsic)
            camera_param.update({'intrinsic': camera_intrinsic})

            # dump extrinsic matrix lidar2camera
            lidar2world = \
                st.x_to_world_transformation(self.lidar.sensor.get_transform())
            camera2world = \
                st.x_to_world_transformation(camera.sensor.get_transform())

            world2camera = np.linalg.inv(camera2world)
            lidar2camera = np.dot(world2camera, lidar2world)
            lidar2camera = self.matrix2list(lidar2camera)
            camera_param.update({'extrinsic': lidar2camera})
            dump_yml.update({'camera%d' % i: camera_param})

        dump_yml.update({'RSU': True})
        # dump the planned trajectory if it exisit.
        if behavior_agent is not None:
            trajectory_deque = \
                behavior_agent.get_local_planner().get_trajectory()
            trajectory_list = []

            for i in range(len(trajectory_deque)):
                tmp_buffer = trajectory_deque.popleft()
                x = tmp_buffer[0].location.x
                y = tmp_buffer[0].location.y
                spd = tmp_buffer[1]

                trajectory_list.append([x, y, spd])

            dump_yml.update({'plan_trajectory': trajectory_list})
            dump_yml.update({'RSU': False})

        yml_name = '%06d' % self.lidar.frame + '.yaml'
        save_path = os.path.join(self.save_folder_path,
                                 yml_name)

        save_yaml(dump_yml, save_path)

    @staticmethod
    def matrix2list(matrix):
        """
        To generate readable yaml file, we need to convert the matrix
        to list format.

        Parameters
        ----------
        matrix : np.ndarray
            The extrinsic/intrinsic matrix.

        Returns
        -------
        matrix_list : list
            The matrix represents in list format.
        """

        assert len(matrix.shape) == 2
        return matrix.tolist()
