#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('max_poses', 10000)
        self.declare_parameter('start_delay', 5.0)  # wait for EKF convergence

        self.max_poses = self.get_parameter('max_poses').get_parameter_value().integer_value
        start_delay = self.get_parameter('start_delay').get_parameter_value().double_value

        self.path = Path()
        self.ready = False
        self.pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.callback, 10)

        if start_delay > 0.0:
            self.create_timer(start_delay, self._on_ready)
        else:
            self.ready = True

    def _on_ready(self):
        self.ready = True
        self.get_logger().info('EKF convergence delay done — path recording started.')

    def callback(self, msg: Odometry):
        if not self.ready:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header = msg.header
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)
        self.pub.publish(self.path)


def main():
    rclpy.init()
    rclpy.spin(OdomToPath())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
