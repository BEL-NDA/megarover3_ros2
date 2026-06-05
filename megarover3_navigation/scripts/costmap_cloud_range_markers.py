#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class CostmapCloudRangeMarkers(Node):
    def __init__(self):
        super().__init__('costmap_cloud_range_markers')

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('height_min', 0.15)
        self.declare_parameter('height_max', 1.8)
        self.declare_parameter('local_width', 5.0)
        self.declare_parameter('local_height', 5.0)
        self.declare_parameter('local_obstacle_min_range', 0.20)
        self.declare_parameter('local_obstacle_max_range', 3.5)
        self.declare_parameter('local_raytrace_max_range', 4.0)
        self.declare_parameter('global_width', 12.0)
        self.declare_parameter('global_height', 12.0)
        self.declare_parameter('global_obstacle_min_range', 0.20)
        self.declare_parameter('global_obstacle_max_range', 4.0)
        self.declare_parameter('global_raytrace_max_range', 5.0)

        self.publisher = self.create_publisher(MarkerArray, 'costmap_cloud_ranges', 1)
        publish_rate = max(0.1, self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_markers)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        height_min = float(self.get_parameter('height_min').value)
        height_max = float(self.get_parameter('height_max').value)
        local_width = float(self.get_parameter('local_width').value)
        local_height = float(self.get_parameter('local_height').value)
        local_obstacle_min = float(self.get_parameter('local_obstacle_min_range').value)
        local_obstacle_max = float(self.get_parameter('local_obstacle_max_range').value)
        local_raytrace_max = float(self.get_parameter('local_raytrace_max_range').value)
        global_width = float(self.get_parameter('global_width').value)
        global_height = float(self.get_parameter('global_height').value)
        global_obstacle_min = float(self.get_parameter('global_obstacle_min_range').value)
        global_obstacle_max = float(self.get_parameter('global_obstacle_max_range').value)
        global_raytrace_max = float(self.get_parameter('global_raytrace_max_range').value)

        markers = [
            self.box_marker(
                now, frame_id, 'local_costmap_window', 0,
                -local_width / 2.0, local_width / 2.0,
                -local_height / 2.0, local_height / 2.0,
                height_min, height_max,
                (0.1, 0.85, 1.0, 0.9), 0.04),
            self.box_marker(
                now, frame_id, 'local_marking_range_front_box', 1,
                local_obstacle_min, min(local_obstacle_max, local_width / 2.0),
                -local_height / 2.0, local_height / 2.0,
                height_min, height_max,
                (0.1, 1.0, 0.25, 0.9), 0.05),
            self.box_marker(
                now, frame_id, 'global_costmap_window', 2,
                -global_width / 2.0, global_width / 2.0,
                -global_height / 2.0, global_height / 2.0,
                height_min, height_max,
                (0.75, 0.75, 0.75, 0.55), 0.025),
            self.box_marker(
                now, frame_id, 'global_marking_range_front_box', 3,
                global_obstacle_min, min(global_obstacle_max, global_width / 2.0),
                -min(global_obstacle_max, global_height / 2.0),
                min(global_obstacle_max, global_height / 2.0),
                height_min, height_max,
                (1.0, 0.85, 0.1, 0.85), 0.04),
            self.circle_marker(
                now, frame_id, 'local_raytrace_range', 4,
                local_raytrace_max, height_min,
                (0.1, 0.85, 1.0, 0.65), 0.03),
            self.circle_marker(
                now, frame_id, 'global_raytrace_range', 5,
                global_raytrace_max, height_min,
                (1.0, 0.85, 0.1, 0.55), 0.03),
        ]

        self.publisher.publish(MarkerArray(markers=markers))

    def base_marker(self, stamp, frame_id, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        return marker

    def box_marker(self, stamp, frame_id, namespace, marker_id,
                   min_x, max_x, min_y, max_y, min_z, max_z, color, line_width):
        marker = self.base_marker(stamp, frame_id, namespace, marker_id, Marker.LINE_LIST)
        marker.scale.x = line_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        corners = [
            Point(x=min_x, y=min_y, z=min_z),
            Point(x=max_x, y=min_y, z=min_z),
            Point(x=max_x, y=max_y, z=min_z),
            Point(x=min_x, y=max_y, z=min_z),
            Point(x=min_x, y=min_y, z=max_z),
            Point(x=max_x, y=min_y, z=max_z),
            Point(x=max_x, y=max_y, z=max_z),
            Point(x=min_x, y=max_y, z=max_z),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for a, b in edges:
            marker.points.append(corners[a])
            marker.points.append(corners[b])
        return marker

    def circle_marker(self, stamp, frame_id, namespace, marker_id,
                      radius, z, color, line_width):
        marker = self.base_marker(stamp, frame_id, namespace, marker_id, Marker.LINE_STRIP)
        marker.scale.x = line_width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        segments = 96
        for index in range(segments + 1):
            angle = 2.0 * math.pi * index / segments
            marker.points.append(Point(
                x=radius * math.cos(angle),
                y=radius * math.sin(angle),
                z=z,
            ))
        return marker


def main():
    rclpy.init()
    node = CostmapCloudRangeMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
