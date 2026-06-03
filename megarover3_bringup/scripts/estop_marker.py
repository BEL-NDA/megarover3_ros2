#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker


class EstopMarker(Node):
    def __init__(self):
        super().__init__('estop_marker')
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Marker, '/estop_marker', 10)
        self.sub = self.create_subscription(
            Bool,
            '/rover_estop',
            self.on_estop,
            best_effort_qos,
        )
        self.estop = False
        self.timer = self.create_timer(0.5, self.publish_marker)

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_footprint'
        marker.ns = 'estop'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=0.0, y=0.0, z=1.6)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        if self.estop:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        else:
            marker.color = ColorRGBA(r=0.0, g=0.9, b=0.0, a=0.8)
        marker.lifetime.sec = 1
        self.pub.publish(marker)


def main():
    rclpy.init()
    node = EstopMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
