#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('max_poses', 10000)
        self.max_poses = self.get_parameter('max_poses').get_parameter_value().integer_value

        self.path = Path()
        self.pub = self.create_publisher(Path, '/path', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.callback, 10)

    def callback(self, msg: Odometry):
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
