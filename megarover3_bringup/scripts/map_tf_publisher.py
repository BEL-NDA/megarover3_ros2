#!/usr/bin/env python3
"""Publish map→odom TF from ZED pose topic (for SLAM landmarks visualization)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


CAMERA_HEIGHT = 1.20  # base_link から ZED カメラまでの高さ [m]


class MapTfPublisher(Node):
    def __init__(self):
        super().__init__('map_tf_publisher')
        self.br = TransformBroadcaster(self)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseStamped, '/zed/zed_node/pose', self._callback, qos)
        self.get_logger().info('Publishing map→odom TF from /zed/zed_node/pose')

    def _callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        # ZED pose は map 内のカメラ位置。odom はカメラより CAMERA_HEIGHT 低い位置
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z - CAMERA_HEIGHT
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = MapTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
