#!/usr/bin/env python3
"""Load a PCD file and publish as a latched PointCloud2 (map frame)."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class PublishMapPcd(Node):
    def __init__(self):
        super().__init__('publish_map_pcd')
        self.declare_parameter('pcd_path', '')
        self.declare_parameter('frame_id', 'map')

        pcd_path = self.get_parameter('pcd_path').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        if not pcd_path:
            raise RuntimeError('pcd_path parameter is required')

        pts = self._load_pcd(pcd_path)

        # transient local (latched) so late subscribers still get it
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(PointCloud2, '/map_pointcloud', qos)
        msg = self._to_msg(pts, frame_id)
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(pts)} points from {pcd_path}')

        # keep republishing every 5 seconds for new subscribers
        self._msg = msg
        self.create_timer(5.0, lambda: self.pub.publish(self._msg))

    def _load_pcd(self, path: str) -> np.ndarray:
        pts = []
        data_section = False
        with open(path) as f:
            for line in f:
                if data_section:
                    vals = line.strip().split()
                    if len(vals) >= 3:
                        pts.append([float(v) for v in vals[:3]])
                elif line.startswith('DATA'):
                    data_section = True
        return np.array(pts, dtype=np.float32)

    def _to_msg(self, pts: np.ndarray, frame_id: str) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32
        msg.row_step = msg.point_step * len(pts)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pts.tobytes()
        return msg


def main():
    rclpy.init()
    node = PublishMapPcd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
