#!/usr/bin/env python3

import math
import random

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class RandomGoalSender(Node):
    def __init__(self):
        super().__init__('random_goal_sender')

        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('x_min', -4.0)
        self.declare_parameter('x_max', 4.0)
        self.declare_parameter('y_min', -4.0)
        self.declare_parameter('y_max', 4.0)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('goal_pause_sec', 2.0)
        self.declare_parameter('initial_delay_sec', 5.0)
        self.declare_parameter('random_yaw', True)
        self.declare_parameter('seed', 0)

        action_name = self.get_parameter('action_name').value
        self._frame_id = self.get_parameter('frame_id').value
        self._x_min = float(self.get_parameter('x_min').value)
        self._x_max = float(self.get_parameter('x_max').value)
        self._y_min = float(self.get_parameter('y_min').value)
        self._y_max = float(self.get_parameter('y_max').value)
        self._goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self._goal_pause_sec = float(self.get_parameter('goal_pause_sec').value)
        self._random_yaw = bool(self.get_parameter('random_yaw').value)

        seed = int(self.get_parameter('seed').value)
        self._rng = random.Random(seed if seed != 0 else None)

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._active_goal_handle = None
        self._goal_started_time = None
        self._sending_goal = False
        self._next_goal_time = self.get_clock().now().nanoseconds * 1e-9 + float(
            self.get_parameter('initial_delay_sec').value
        )

        self._timer = self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            'Random Nav2 goals enabled: '
            f'action={action_name}, frame={self._frame_id}, '
            f'x=[{self._x_min}, {self._x_max}], y=[{self._y_min}, {self._y_max}]'
        )

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._active_goal_handle is not None:
            if now - self._goal_started_time > self._goal_timeout_sec:
                self.get_logger().warn('Navigation goal timed out; canceling current goal.')
                self._active_goal_handle.cancel_goal_async()
                self._active_goal_handle = None
                self._goal_started_time = None
                self._next_goal_time = now + self._goal_pause_sec
            return

        if self._sending_goal or now < self._next_goal_time:
            return

        if not self._action_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 NavigateToPose action server...')
            return

        self._send_goal()

    def _send_goal(self):
        pose = self._random_pose()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._sending_goal = True
        self.get_logger().info(
            f'Sending random goal: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}, frame={pose.header.frame_id}'
        )
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self._sending_goal = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected.')
            self._next_goal_time = self.get_clock().now().nanoseconds * 1e-9 + self._goal_pause_sec
            return

        self._active_goal_handle = goal_handle
        self._goal_started_time = self.get_clock().now().nanoseconds * 1e-9
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal reached.')
        else:
            self.get_logger().warn(f'Navigation goal finished with status={status}.')

        self._active_goal_handle = None
        self._goal_started_time = None
        self._next_goal_time = self.get_clock().now().nanoseconds * 1e-9 + self._goal_pause_sec

    def _random_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = self._rng.uniform(self._x_min, self._x_max)
        pose.pose.position.y = self._rng.uniform(self._y_min, self._y_max)
        pose.pose.position.z = 0.0

        yaw = self._rng.uniform(-math.pi, math.pi) if self._random_yaw else 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
