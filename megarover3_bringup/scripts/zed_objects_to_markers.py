#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import ObjectsStamped


class ZedObjectsToMarkers(Node):
    def __init__(self):
        super().__init__('zed_objects_to_markers')
        self.declare_parameter('objects_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('markers_topic', '/zed/obj_det/markers')
        self.declare_parameter('marker_lifetime_sec', 0.4)

        objects_topic = self.get_parameter('objects_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)

        self.pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.sub = self.create_subscription(
            ObjectsStamped,
            objects_topic,
            self.on_objects,
            10,
        )
        self.get_logger().info(f'publishing ZED object markers: {objects_topic} -> {markers_topic}')

    def on_objects(self, msg):
        markers = MarkerArray()
        active_ids = set()

        for index, obj in enumerate(msg.objects):
            cube_id = index * 2
            text_id = cube_id + 1
            active_ids.add(cube_id)
            active_ids.add(text_id)

            label = self.object_label(obj)
            color = self.object_color(label)
            position = self.object_position(obj)
            dimensions = self.object_dimensions(obj)

            cube = Marker()
            cube.header = msg.header
            cube.ns = 'zed_obj_det_boxes'
            cube.id = cube_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = position[0]
            cube.pose.position.y = position[1]
            cube.pose.position.z = position[2]
            cube.pose.orientation.w = 1.0
            cube.scale.x = max(dimensions[0], 0.05)
            cube.scale.y = max(dimensions[1], 0.05)
            cube.scale.z = max(dimensions[2], 0.05)
            cube.color.r = color[0]
            cube.color.g = color[1]
            cube.color.b = color[2]
            cube.color.a = 0.35
            cube.lifetime.sec = int(self.marker_lifetime_sec)
            cube.lifetime.nanosec = int((self.marker_lifetime_sec % 1.0) * 1e9)
            markers.markers.append(cube)

            text = Marker()
            text.header = msg.header
            text.ns = 'zed_obj_det_labels'
            text.id = text_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = position[0]
            text.pose.position.y = position[1]
            text.pose.position.z = position[2] + max(dimensions[2] * 0.5, 0.15)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.14
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 1.0
            text.text = label
            text.lifetime = cube.lifetime
            markers.markers.append(text)

        for stale_id in range(len(msg.objects) * 2, 80):
            marker = Marker()
            marker.header = msg.header
            marker.ns = 'zed_obj_det_boxes' if stale_id % 2 == 0 else 'zed_obj_det_labels'
            marker.id = stale_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)

        self.pub.publish(markers)

    @staticmethod
    def object_label(obj):
        label = getattr(obj, 'label', '')
        sublabel = getattr(obj, 'sublabel', '')
        confidence = getattr(obj, 'confidence', 0.0)
        if sublabel:
            label = f'{label}/{sublabel}'
        return f'{label} {confidence:.0f}%'

    @staticmethod
    def object_position(obj):
        position = list(getattr(obj, 'position', []))
        if len(position) >= 3 and all(math.isfinite(v) for v in position[:3]):
            return [float(v) for v in position[:3]]
        return [0.0, 0.0, 0.0]

    @staticmethod
    def object_dimensions(obj):
        dimensions = list(getattr(obj, 'dimensions_3d', []))
        if len(dimensions) >= 3 and all(math.isfinite(v) and v > 0.0 for v in dimensions[:3]):
            return [float(v) for v in dimensions[:3]]
        return [0.3, 0.3, 0.3]

    @staticmethod
    def object_color(label):
        if label.startswith('PERSON') or label.startswith('PEOPLE'):
            return (0.0, 1.0, 0.0)
        if label.startswith('ELECTRONICS'):
            return (0.2, 0.6, 1.0)
        if label.startswith('VEHICLE'):
            return (1.0, 0.6, 0.0)
        return (1.0, 1.0, 0.0)


def main():
    rclpy.init()
    node = ZedObjectsToMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
