#!/usr/bin/env python3
"""Publish a colored sphere marker indicating e-stop state."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


class EstopMarker(Node):
    def __init__(self):
        super().__init__('estop_marker')
        self.pub = self.create_publisher(Marker, '/estop_marker', 10)
        self.create_subscription(Bool, '/rover_estop', self._callback, 10)
        # publish initial state
        self.create_timer(0.5, self._publish_current)
        self._estop = False

    def _callback(self, msg: Bool):
        self._estop = msg.data
        self._publish_current()

    def _publish_current(self):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.ns = 'estop'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position = Point(x=0.0, y=0.0, z=1.6)
        m.pose.orientation.w = 1.0

        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15

        if self._estop:
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 赤
        else:
            m.color = ColorRGBA(r=0.0, g=0.9, b=0.0, a=0.8)  # 緑

        m.lifetime.sec = 1

        self.pub.publish(m)


def main():
    rclpy.init()
    node = EstopMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
