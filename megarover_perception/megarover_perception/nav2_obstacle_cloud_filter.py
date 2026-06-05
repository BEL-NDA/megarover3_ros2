import math
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def quaternion_to_matrix(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


def pack_xy_indices(indices):
    int64_indices = indices.astype(np.int64, copy=False)
    return (int64_indices[:, 0] << 32) ^ (int64_indices[:, 1] & 0xffffffff)


class Nav2ObstacleCloudFilter(Node):
    def __init__(self):
        super().__init__('nav2_obstacle_cloud_filter')

        self.declare_parameter('input_topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('output_topic', '/perception/nav2_obstacle_cloud')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('min_x', 0.20)
        self.declare_parameter('max_x', 4.0)
        self.declare_parameter('min_y', -2.5)
        self.declare_parameter('max_y', 2.5)
        self.declare_parameter('min_z', 0.15)
        self.declare_parameter('max_z', 1.8)
        self.declare_parameter('ground_filter_enabled', True)
        self.declare_parameter('ground_sample_min_z', -0.20)
        self.declare_parameter('ground_sample_max_z', 0.45)
        self.declare_parameter('ground_xy_leaf_size', 0.20)
        self.declare_parameter('ground_clearance', 0.16)
        self.declare_parameter('voxel_leaf_size', 0.06)
        self.declare_parameter('radius_outlier_radius', 0.18)
        self.declare_parameter('radius_outlier_min_neighbors', 2)
        self.declare_parameter('max_publish_hz', 8.0)
        self.declare_parameter('log_stats', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.max_publish_hz = float(self.get_parameter('max_publish_hz').value)
        self.min_publish_period = 1.0 / self.max_publish_hz if self.max_publish_hz > 0.0 else 0.0
        self.last_publish_time = 0.0
        self.last_stats_time = 0.0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(PointCloud2, input_topic, self.point_cloud_callback, 10)

        self.get_logger().info(
            f'Filtering Nav2 obstacle cloud: {input_topic} -> {output_topic}, '
            f'target_frame={self.target_frame}')

    def point_cloud_callback(self, msg):
        now_wall = time.monotonic()
        if self.min_publish_period > 0.0 and now_wall - self.last_publish_time < self.min_publish_period:
            return
        self.last_publish_time = now_wall

        start = time.perf_counter()
        xyz = self.extract_xyz(msg)
        if xyz.size == 0:
            self.publish_xyz(msg.header, xyz)
            return

        finite_mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite_mask]
        if xyz.size == 0:
            self.publish_xyz(msg.header, xyz)
            return

        transform = self.lookup_transform(msg)
        if transform is None:
            return

        base_xyz = self.transform_points(xyz, transform)
        finite_mask = np.isfinite(base_xyz).all(axis=1)
        xyz = xyz[finite_mask]
        base_xyz = base_xyz[finite_mask]

        cropped_xyz, cropped_base_xyz = self.crop_points(xyz, base_xyz)
        obstacle_xyz, obstacle_base_xyz = self.remove_ground_points(cropped_xyz, cropped_base_xyz)
        voxel_xyz, voxel_base_xyz = self.voxel_downsample(obstacle_xyz, obstacle_base_xyz)
        filtered_xyz = self.radius_outlier_filter(voxel_xyz, voxel_base_xyz)

        self.publish_xyz(msg.header, filtered_xyz)
        self.log_stats(
            len(xyz),
            len(cropped_xyz),
            len(obstacle_xyz),
            len(voxel_xyz),
            len(filtered_xyz),
            (time.perf_counter() - start) * 1000.0)

    def extract_xyz(self, msg):
        field_names = {field.name for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(field_names):
            self.get_logger().warning('PointCloud2 has no x/y/z fields.')
            return np.empty((0, 3), dtype=np.float32)

        dtype = point_cloud2.dtype_from_fields(msg.fields, msg.point_step)
        cloud = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        ).reshape(-1)
        return np.column_stack((cloud['x'], cloud['y'], cloud['z'])).astype(np.float32, copy=False)

    def lookup_transform(self, msg):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'TF lookup failed: {self.target_frame} <- {msg.header.frame_id}: {exc}')
                return None

    def transform_points(self, xyz, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return xyz @ matrix.T + offset

    def crop_points(self, xyz, base_xyz):
        min_x = float(self.get_parameter('min_x').value)
        max_x = float(self.get_parameter('max_x').value)
        min_y = float(self.get_parameter('min_y').value)
        max_y = float(self.get_parameter('max_y').value)
        max_z = float(self.get_parameter('max_z').value)
        if bool(self.get_parameter('ground_filter_enabled').value):
            min_z = float(self.get_parameter('ground_sample_min_z').value)
        else:
            min_z = float(self.get_parameter('min_z').value)

        mask = (
            (base_xyz[:, 0] >= min_x) &
            (base_xyz[:, 0] <= max_x) &
            (base_xyz[:, 1] >= min_y) &
            (base_xyz[:, 1] <= max_y) &
            (base_xyz[:, 2] >= min_z) &
            (base_xyz[:, 2] <= max_z)
        )
        return xyz[mask], base_xyz[mask]

    def remove_ground_points(self, xyz, base_xyz):
        if len(base_xyz) == 0:
            return xyz, base_xyz

        min_z = float(self.get_parameter('min_z').value)
        if not bool(self.get_parameter('ground_filter_enabled').value):
            mask = base_xyz[:, 2] >= min_z
            return xyz[mask], base_xyz[mask]

        sample_min_z = float(self.get_parameter('ground_sample_min_z').value)
        sample_max_z = float(self.get_parameter('ground_sample_max_z').value)
        xy_leaf_size = float(self.get_parameter('ground_xy_leaf_size').value)
        clearance = float(self.get_parameter('ground_clearance').value)
        if xy_leaf_size <= 0.0:
            mask = base_xyz[:, 2] >= min_z
            return xyz[mask], base_xyz[mask]

        sample_mask = (base_xyz[:, 2] >= sample_min_z) & (base_xyz[:, 2] <= sample_max_z)
        if not np.any(sample_mask):
            mask = base_xyz[:, 2] >= min_z
            return xyz[mask], base_xyz[mask]

        sample_xy_indices = np.floor(base_xyz[sample_mask, :2] / xy_leaf_size).astype(np.int32)
        sample_z = base_xyz[sample_mask, 2]
        sample_keys = pack_xy_indices(sample_xy_indices)
        unique_keys, inverse_indices = np.unique(sample_keys, return_inverse=True)
        ground_z_by_cell = np.full(len(unique_keys), np.inf, dtype=np.float32)
        np.minimum.at(ground_z_by_cell, inverse_indices, sample_z)

        fallback_ground_z = float(np.percentile(sample_z, 5.0))
        cell_indices = np.floor(base_xyz[:, :2] / xy_leaf_size).astype(np.int32)
        point_keys = pack_xy_indices(cell_indices)
        positions = np.searchsorted(unique_keys, point_keys)
        valid_positions = positions < len(unique_keys)
        matched = np.zeros(len(point_keys), dtype=bool)
        matched[valid_positions] = unique_keys[positions[valid_positions]] == point_keys[valid_positions]
        local_ground_z = np.full(len(base_xyz), fallback_ground_z, dtype=np.float32)
        local_ground_z[matched] = ground_z_by_cell[positions[matched]]
        obstacle_min_z = np.maximum(min_z, local_ground_z + clearance)
        keep_mask = base_xyz[:, 2] >= obstacle_min_z

        return xyz[keep_mask], base_xyz[keep_mask]

    def voxel_downsample(self, xyz, base_xyz):
        leaf_size = float(self.get_parameter('voxel_leaf_size').value)
        if leaf_size <= 0.0 or len(base_xyz) == 0:
            return xyz, base_xyz

        voxel_indices = np.floor(base_xyz / leaf_size).astype(np.int32)
        _, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)
        selected_indices = np.full(int(inverse_indices.max()) + 1, -1, dtype=np.int32)
        selected_z = np.full(len(selected_indices), np.inf, dtype=np.float32)
        for index, voxel_index in enumerate(inverse_indices):
            z_value = base_xyz[index, 2]
            if z_value < selected_z[voxel_index]:
                selected_z[voxel_index] = z_value
                selected_indices[voxel_index] = index
        selected_indices.sort()
        return xyz[selected_indices], base_xyz[selected_indices]

    def radius_outlier_filter(self, xyz, base_xyz):
        radius = float(self.get_parameter('radius_outlier_radius').value)
        min_neighbors = int(self.get_parameter('radius_outlier_min_neighbors').value)
        if radius <= 0.0 or min_neighbors <= 0 or len(base_xyz) == 0:
            return xyz

        cell_size = radius
        cell_indices = np.floor(base_xyz / cell_size).astype(np.int32)
        cells = defaultdict(list)
        for index, key in enumerate(map(tuple, cell_indices)):
            cells[key].append(index)

        radius_squared = radius * radius
        keep_mask = np.zeros(len(base_xyz), dtype=bool)
        neighbor_offsets = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        ]

        for index, cell in enumerate(map(tuple, cell_indices)):
            neighbor_count = 0
            point = base_xyz[index]
            for offset in neighbor_offsets:
                neighbor_cell = (cell[0] + offset[0], cell[1] + offset[1], cell[2] + offset[2])
                for candidate in cells.get(neighbor_cell, []):
                    if candidate == index:
                        continue
                    delta = base_xyz[candidate] - point
                    if float(delta @ delta) <= radius_squared:
                        neighbor_count += 1
                        if neighbor_count >= min_neighbors:
                            keep_mask[index] = True
                            break
                if keep_mask[index]:
                    break

        return xyz[keep_mask]

    def publish_xyz(self, header, xyz):
        output = point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32, copy=False))
        self.publisher.publish(output)

    def log_stats(self, finite_count, cropped_count, obstacle_count, voxel_count, output_count, elapsed_ms):
        if not bool(self.get_parameter('log_stats').value):
            return
        now = time.monotonic()
        if now - self.last_stats_time < 5.0:
            return
        self.last_stats_time = now
        self.get_logger().info(
            'nav2 obstacle cloud filter: '
            f'finite={finite_count}, cropped={cropped_count}, '
            f'obstacle={obstacle_count}, '
            f'voxel={voxel_count}, output={output_count}, elapsed={elapsed_ms:.1f}ms')


def main():
    rclpy.init()
    node = Nav2ObstacleCloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
