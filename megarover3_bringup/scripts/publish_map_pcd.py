#!/usr/bin/env python3
"""Load a XYZRGB PCD file and publish as a latched PointCloud2 (map frame)."""
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

        pts, has_rgb, rgb_packed = self._load_pcd(pcd_path)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(PointCloud2, '/map_pointcloud', qos)
        msg = self._to_msg(pts, rgb_packed, has_rgb, frame_id)
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(pts)} points from {pcd_path} (rgb={has_rgb})')

        self._msg = msg
        self.create_timer(5.0, lambda: self.pub.publish(self._msg))

    def _load_pcd(self, path: str):
        fields = []
        has_rgb = False
        data_section = False
        rows = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if data_section:
                    vals = line.split()
                    if vals:
                        rows.append(vals)
                elif line.startswith('FIELDS'):
                    fields = line.split()[1:]
                    has_rgb = 'rgb' in fields
                elif line.startswith('DATA'):
                    data_section = True

        data = np.array(rows, dtype=object)
        pts = data[:, :3].astype(np.float32)
        rgb_packed = None
        if has_rgb and data.shape[1] >= 4:
            rgb_packed = data[:, 3].astype(np.uint32)
        return pts, has_rgb, rgb_packed

    def _to_msg(self, pts, rgb_packed, has_rgb, frame_id):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False

        if has_rgb and rgb_packed is not None:
            msg.point_step = 16  # x y z _ rgb (with padding for alignment)
            msg.fields = [
                PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            # pack: x, y, z, rgb_as_float
            rgb_float = rgb_packed.view(np.float32)
            data = np.column_stack([pts, rgb_float.reshape(-1, 1)]).astype(np.float32)
        else:
            msg.point_step = 12
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            data = pts

        msg.row_step = msg.point_step * len(pts)
        msg.data = data.tobytes()
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
