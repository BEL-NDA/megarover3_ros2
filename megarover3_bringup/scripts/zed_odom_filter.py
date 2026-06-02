#!/usr/bin/env python3
"""Filter ZED visual odometry: drop measurements that imply unrealistic velocities."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class ZedOdomFilter(Node):
    def __init__(self):
        super().__init__('zed_odom_filter')
        self.declare_parameter('max_linear_vel',  1.0)   # m/s
        self.declare_parameter('max_angular_vel', 2.0)   # rad/s

        self.max_v = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_vel').get_parameter_value().double_value

        self.prev_msg: Odometry | None = None

        self.pub = self.create_publisher(Odometry, '/zed/odom/filtered', 10)
        self.create_subscription(Odometry, '/zed/zed_node/odom', self._callback, 10)

    def _callback(self, msg: Odometry):
        if self.prev_msg is None:
            self.prev_msg = msg
            self.pub.publish(msg)
            return

        dt = (
            msg.header.stamp.sec - self.prev_msg.header.stamp.sec
            + (msg.header.stamp.nanosec - self.prev_msg.header.stamp.nanosec) * 1e-9
        )
        if dt <= 0.0:
            return

        dx = msg.pose.pose.position.x - self.prev_msg.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.prev_msg.pose.pose.position.y
        dist = math.hypot(dx, dy)
        v = dist / dt

        # yaw difference (simplified — works for 2D)
        def yaw(q):
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        dyaw = abs(yaw(msg.pose.pose.orientation) - yaw(self.prev_msg.pose.pose.orientation))
        if dyaw > math.pi:
            dyaw = abs(dyaw - 2 * math.pi)
        w = dyaw / dt

        if v > self.max_v or w > self.max_w:
            self.get_logger().warn(
                f'ZED odom jump rejected: v={v:.2f} m/s w={w:.2f} rad/s (dt={dt:.3f}s)',
                throttle_duration_sec=1.0,
            )
            return

        self.prev_msg = msg
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ZedOdomFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
