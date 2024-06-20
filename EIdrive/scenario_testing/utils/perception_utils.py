"""
Function for cooperative perception and split learning..
"""

import numpy as np
from EIdrive.core.sensing.perception.dynamic_obstacle import BoundingBox
import open3d as o3d
import carla
import pygame
import weakref

VIEW_WIDTH = 2560
VIEW_HEIGHT = 1440
VIEW_FOV = 90


class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_bounding_boxes(vehicles, camera):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """

        bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box(
            vehicle, camera) for vehicle in vehicles]
        # filter objects behind camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        return bounding_boxes

    @staticmethod
    
    def draw_ground_truth_bbx(display, bounding_boxes, ego_vehicle_position, vehicle_info, ego_bbox=None,
                              line_between_vehicle=True):
        """
        Draws bounding boxes on pygame display.
        """
        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))

        # For ego vehicle:
        points = [(int(ego_bbox[i, 0]), int(ego_bbox[i, 1]))
                  for i in range(8)]
        color = (0, 255, 0)  # Green for ego
        font = pygame.font.SysFont('Roboto', 60)
        text_surface = font.render("Ego", True, color)
        text_position = (points[0][0], points[0][1] - 110)
        bb_surface.blit(text_surface, text_position)

        for vehicle_id, bbox in bounding_boxes:
            color = (0, 255, 0)  # Green for HVs
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]

            # Displaying real-time location
            x, y = vehicle_info[vehicle_id]['location']
            location_text_surface = pygame.font.SysFont(
                'Roboto', 50).render(f"({x:.1f}, {y:.1f})", True, color)
            location_text_position = (points[0][0], points[0][1] - 10)
            bb_surface.blit(location_text_surface, location_text_position)

            # Displaying speed
            speed = vehicle_info[vehicle_id]['speed']
            speed_text_surface = pygame.font.SysFont(
                'Roboto', 50).render(f"{speed:.1f} km/h", True, color)
            speed_text_position = (points[0][0], points[0][1] - 60)
            bb_surface.blit(speed_text_surface, speed_text_position)

            # Drawing the link between the vehicles
            if line_between_vehicle:
                pygame.draw.aalines(bb_surface, color, False, [
                    points[0], ego_vehicle_position], 1)
            # Draw lines
            # base
            pygame.draw.line(bb_surface, color, points[0], points[1])
            pygame.draw.line(bb_surface, color, points[0], points[1])
            pygame.draw.line(bb_surface, color, points[1], points[2])
            pygame.draw.line(bb_surface, color, points[2], points[3])
            pygame.draw.line(bb_surface, color, points[3], points[0])
            # top
            pygame.draw.line(bb_surface, color, points[4], points[5])
            pygame.draw.line(bb_surface, color, points[5], points[6])
            pygame.draw.line(bb_surface, color, points[6], points[7])
            pygame.draw.line(bb_surface, color, points[7], points[4])
            # base-top
            pygame.draw.line(bb_surface, color, points[0], points[4])
            pygame.draw.line(bb_surface, color, points[1], points[5])
            pygame.draw.line(bb_surface, color, points[2], points[6])
            pygame.draw.line(bb_surface, color, points[3], points[7])

            font = pygame.font.SysFont('Roboto', 60)
            text_surface = font.render('Other vehicle', True, color)
            text_position = (points[0][0], points[0][1] - 110)
            bb_surface.blit(text_surface, text_position)

        display.blit(bb_surface, (0, 0))

#bbx is being merged when there is no other bbx to merge to

    @staticmethod
    def draw_only_bbx(display, bounding_boxes, vehicle, calibration, sensor, rsu_locations, corner_form=False):
        """
        Draws bounding boxes on pygame display.
        """
        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        color = (255, 100, 0)  # Color for all bounding boxes

        # Compute transformation matrices
        transform = vehicle.get_transform()

        # Create rsu location matrix
        if rsu_locations is not None:
            rsu_locations_matrix = np.zeros((len(rsu_locations)+1, 3))
            for idx, rsu_loc in enumerate(rsu_locations):
                rsu_locations_matrix[idx] = [rsu_loc.x, rsu_loc.y, rsu_loc.z]

            # To transform the ego vehicle location, add it to the end of rsu location matrix
            rsu_locations_matrix[-1] = [transform.location.x, transform.location.y, transform.location.z]

            # Convert the location from world coordinate to camera coordinate
            rsu_locations_matrix = np.vstack((rsu_locations_matrix.T, np.ones(rsu_locations_matrix.shape[0])))
            cords_x_y_z = ClientSideBoundingBoxes._world_to_sensor(rsu_locations_matrix, sensor)
            cords_y_minus_z_x = np.concatenate(
                [cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])0
            rsu_point = np.transpose(np.dot(calibration, cords_y_minus_z_x))
            print(rsu_point)
            rsu_point = np.concatenate([rsu_point[:, 0] / rsu_point[:, 2], rsu_point[:, 1] / rsu_point[:, 2]], axis=1)
            ego_vehicle_position = (int(rsu_point[-1, 0]), int(rsu_point[-1, 1]))

            # Draw lines from rsu to vehicle
            for i in range(len(rsu_locations)):
                pygame.draw.aalines(bb_surface, color, False, [(rsu_point[i, 0], rsu_point[i, 1]), ego_vehicle_position], 5)

        for bbox in bounding_boxes:
            if not corner_form:
                corners = bbox.corners
                corners = np.vstack((corners.T, np.ones(corners.shape[0])))
            else:
                corners = np.vstack((bbox.T, np.ones(bbox.shape[0])))
            cords_x_y_z = ClientSideBoundingBoxes._world_to_sensor(corners, sensor)
            cords_y_minus_z_x = np.concatenate(
                [cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
            bbox = np.transpose(np.dot(calibration, cords_y_minus_z_x))
            camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)

            points = [(int(camera_bbox[i, 0]), int(camera_bbox[i, 1])) for i in range(8)]

            # Drawing the bounding box
            width = 3
            # Base
            pygame.draw.line(bb_surface, color, points[0], points[1], width)
            pygame.draw.line(bb_surface, color, points[1], points[7], width)
            pygame.draw.line(bb_surface, color, points[7], points[2], width)
            pygame.draw.line(bb_surface, color, points[2], points[0], width)
            # Top
            pygame.draw.line(bb_surface, color, points[3], points[5], width)
            pygame.draw.line(bb_surface, color, points[5], points[4], width)
            pygame.draw.line(bb_surface, color, points[4], points[6], width)
            pygame.draw.line(bb_surface, color, points[6], points[3], width)
            # Base-Top connections
            pygame.draw.line(bb_surface, color, points[0], points[3], width)
            pygame.draw.line(bb_surface, color, points[2], points[5], width)
            pygame.draw.line(bb_surface, color, points[1], points[6], width)
            pygame.draw.line(bb_surface, color, points[7], points[4], width)

        display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(
            bb_cords, vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate(
            [cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate(
            [bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(
            world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(
            vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(
            sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


class PygameCamera:
    def __init__(self, world, vehicle, display):
        self.camera_transform = None
        self.camera_actor = None
        self.vehicle = vehicle
        self.world = world
        self.display = display
        self.image = None
        self.capture = True

        # Initialize the camera here or call another method to do so
        self.setup_camera()

    def camera_blueprint(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def setup_camera(self):
        self.camera_transform = carla.Transform(carla.Location(x=-7.5, z=2.4))
        self.camera_actor = self.world.spawn_actor(self.camera_blueprint(), self.camera_transform,
                                                   attach_to=self.vehicle)

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera_actor.calibration = calibration

        weak_self = weakref.ref(self)
        self.camera_actor.listen(lambda image: PygameCamera.process_image(weak_self, image))

    @staticmethod
    def process_image(weak_self, image):
        self = weak_self()
        if self.capture:
            self.image = image
            self.capture = False

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

    def is_in_sight(self, vehicle, camera):
        # Check within camera's view
        bbox = ClientSideBoundingBoxes.get_bounding_box(vehicle, camera)
        points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
        in_view = [0 <= pt[0] <= VIEW_WIDTH and 0 <=
                   pt[1] <= VIEW_HEIGHT for pt in points]

        # Check if vehicle is within a reasonable range (let's say 50 meters for now)
        in_range = self.get_distance(vehicle, self.vehicle) <= 50

        return any(in_view) and in_range

    def get_distance(self, vehicle1, vehicle2):
        loc1 = vehicle1.get_location()
        loc2 = vehicle2.get_location()
        return loc1.distance(loc2)

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        settings.fixed_delta_seconds = 0.03
        self.world.apply_settings(settings)

    def destroy(self):
        if self.camera_actor:
            self.camera_actor.destroy()


def distance(loc1, loc2):
    return ((loc1.x - loc2.x) ** 2 + (loc1.y - loc2.y) ** 2 + (loc1.z - loc2.z) ** 2) ** 0.5


def average_bbx(bbx1, bbx2):
    """
    Average the bounding box.

    Parameters
    ----------
    bbx1: (8, 3) narray
    bbx2: (8, 3) narray
    """
    # Average the corners
    avg_corners = np.mean(np.array([bbx1.corners, bbx2.corners]), axis=0)

    # Create and return the merged bounding box
    return BoundingBox(avg_corners)


def merge_bbx_list(bbx_list):
    """
    Merge the bounding box of the same object according to distance.

    Parameters
    ----------
    bbx_list : List
        List of bbx.

    Returns
    -------
    new_bbx_list : List
        List of merge bbx.
    """
    new_bbx_list = []
    processed = set()
    threshold_distance = 4

    for i, bbx1 in enumerate(bbx_list):
        if i in processed:
            continue
        merged = False
        for j, bbx2 in enumerate(bbx_list[i + 1:]):
            if distance(bbx1.location, bbx2.location) < threshold_distance:
                merged_bbx = average_bbx(bbx1, bbx2)
                new_bbx_list.append(merged_bbx)
                processed.add(i)
                processed.add(i + 1 + j)
                merged = True
                break
        if not merged:
            new_bbx_list.append(bbx1)
    return new_bbx_list


def visualize_bbx_by_open3d(bbx_list, vis, true_extent=None, true_transform=None):
    """
    Visualize the bounding box by Open3D.
    Warning: Open3D has conflict with pygame. Two visualization method cannot run at same time.

    Parameters
    ----------
    bbx_list : List
        The result of object detection.

    vis : Visual object

    true_extent : List
        The extent of ground true bbx.

    true_transform : List
        The transform of ground true bbx.

    """
    vis.clear_geometries()

    lines = [
        [0, 1], [1, 7], [7, 2], [2, 0],  # Bottom face
        [3, 6], [6, 4], [4, 5], [5, 3],  # Top face
        [0, 3], [1, 6], [7, 4], [2, 5]  # Vertical connections
    ]

    for bbx in bbx_list:
        bbx.corners[:, 0] *= -1
        bbx.location.x *= -1
        colors = [[1, 0, 0] for i in range(len(lines))]  # Set detection lines to red color

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(bbx.corners),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        vis.add_geometry(line_set)

        # Draw the bounding box location as a point
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector([(bbx.location.x, bbx.location.y, bbx.location.z)])
        point_cloud.paint_uniform_color([1, 0, 0])  # Set point color to red
        vis.add_geometry(point_cloud)

    for i in range(len(true_extent)):
        true_extent[i].x *= -1
        true_transform[i].location.x *= -1
        true_transform[i].rotation.yaw *= -1
        # Add true vehicle bbx
        true_corners = np.zeros((8, 3))
        offsets = [
            [-true_extent[i].x, -true_extent[i].y, -true_extent[i].z],
            [-true_extent[i].x, true_extent[i].y, -true_extent[i].z],
            [true_extent[i].x, -true_extent[i].y, -true_extent[i].z],
            [-true_extent[i].x, -true_extent[i].y, true_extent[i].z],
            [true_extent[i].x, true_extent[i].y, true_extent[i].z],
            [true_extent[i].x, -true_extent[i].y, true_extent[i].z],
            [-true_extent[i].x, true_extent[i].y, true_extent[i].z],
            [true_extent[i].x, true_extent[i].y, -true_extent[i].z]
        ]

        # Convert RPY to rotation matrix for the current transform
        rotation_matrix = rpy_to_rotation_matrix(true_transform[i].rotation.roll,
                                                 true_transform[i].rotation.pitch,
                                                 true_transform[i].rotation.yaw)

        # Rotate and translate each corner
        for j, offset in enumerate(offsets):
            rotated_offset = np.dot(rotation_matrix, offset)
            true_corners[j] = [
                rotated_offset[0] + true_transform[i].location.x,
                rotated_offset[1] + true_transform[i].location.y,
                rotated_offset[2] + true_transform[i].location.z
            ]

        colors = [[0, 1, 0] for i in range(len(lines))]  # Set ground truth lines to green color

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(true_corners),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        vis.add_geometry(line_set)

        set_bev_view(vis, -10, 244.8, 40)

    vis.poll_events()
    vis.update_renderer()


def set_bev_view(vis, center_x, center_y, center_z):
    """
    Set BEV for camera.
    """
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()

    camera_params_extrinsic = [[1, 0, 0, center_x],
                               [0, -1, 0, center_y],
                               [0, 0, -1, center_z + 5],  # Adding a height offset for BEV
                               [0, 0, 0, 1]]

    R_rotation = np.array([[0, -1, 0, 0],
                           [1, 0, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    camera_params.extrinsic = np.dot(R_rotation, camera_params_extrinsic)

    ctr.convert_from_pinhole_camera_parameters(camera_params)


def rpy_to_rotation_matrix(roll, pitch, yaw):
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Roll
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    # Pitch
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    # Yaw
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R
