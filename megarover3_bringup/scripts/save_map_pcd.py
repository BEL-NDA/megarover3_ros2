#!/usr/bin/env python3
"""Accumulate ZED point cloud during mapping and save as PCD on shutdown."""
import os
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Empty


class SaveMapPcd(Node):
    def __init__(self):
        super().__init__('save_map_pcd')
        self.declare_parameter('output_path', '')
        self.declare_parameter('voxel_size', 0.05)  # m

        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value

        if not self.output_path:
            raise RuntimeError('output_path parameter is required')

        self.points = None  # Nx3 float32

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered',
                                 self._callback, qos)
        self.create_service(Empty, '~/save', self._save_srv)
        self.get_logger().info(f'Accumulating point cloud → {self.output_path}')

    def _callback(self, msg: PointCloud2):
        pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)),
                       dtype=np.float32)
        if pts.size == 0:
            return
        pts = pts.reshape(-1, 3)
        if self.points is None:
            self.points = pts
        else:
            self.points = np.vstack([self.points, pts])
        # voxel downsample every 500k points to keep memory manageable
        if len(self.points) > 500_000:
            self.points = self._voxel_downsample(self.points)
        self.get_logger().info(f'Points: {len(self.points)}', throttle_duration_sec=5.0)

    def _voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        voxel = (pts / self.voxel_size).astype(np.int32)
        _, idx = np.unique(voxel, axis=0, return_index=True)
        return pts[idx]

    def _save_srv(self, _req, res):
        self._write_pcd()
        return res

    def _write_pcd(self):
        if self.points is None or len(self.points) == 0:
            self.get_logger().warn('No points to save.')
            return
        pts = self._voxel_downsample(self.points)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        n = len(pts)
        with open(self.output_path, 'w') as f:
            f.write('# .PCD v0.7\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {n}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {n}\n')
            f.write('DATA ascii\n')
            np.savetxt(f, pts, fmt='%.4f')
        self.get_logger().info(f'Saved {n} points to {self.output_path}')

    def destroy_node(self):
        self._write_pcd()
        super().destroy_node()


def main():
    rclpy.init()
    node = SaveMapPcd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
