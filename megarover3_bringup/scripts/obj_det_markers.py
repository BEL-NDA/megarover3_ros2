#!/usr/bin/env python3
"""Convert ZED ObjectsStamped to MarkerArray for RViz visualization."""
import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# RGBA per label class
_CLASS_COLOR = {
    'Person':      ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.8),
    'Vehicle':     ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.8),
    'Bag':         ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8),
    'Animal':      ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.8),
    'Electronics': ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.8),
    'Fruit':       ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),
    'Sport':       ColorRGBA(r=0.0, g=0.8, b=0.8, a=0.8),
}
_DEFAULT_COLOR = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.8)

# 12 edges of a bounding box by corner index pairs
# Corner layout (ZED convention):
#      1 ------- 2
#     /.        /|
#    0 ------- 3 |
#    | 5 ......| 6
#    |.        |/
#    4 ------- 7
_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # top face
    (4, 5), (5, 6), (6, 7), (7, 4),  # bottom face
    (0, 4), (1, 5), (2, 6), (3, 7),  # vertical
]


def _color_for(label: str) -> ColorRGBA:
    for key, color in _CLASS_COLOR.items():
        if key.lower() in label.lower():
            return color
    return _DEFAULT_COLOR


def _box_marker(obj, marker_id: int, frame_id: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'obj_bbox'
    m.id = marker_id
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.03
    m.color = _color_for(obj.label)
    m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()

    corners = obj.bounding_box_3d.corners
    for a, b in _EDGES:
        ca, cb = corners[a], corners[b]
        m.points.append(Point(x=float(ca.kp[0]), y=float(ca.kp[1]), z=float(ca.kp[2])))
        m.points.append(Point(x=float(cb.kp[0]), y=float(cb.kp[1]), z=float(cb.kp[2])))
    return m


def _text_marker(obj, marker_id: int, frame_id: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'obj_label'
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.2
    m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    m.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()

    pos = obj.position
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2]) + 0.15
    m.pose.orientation.w = 1.0

    conf = int(obj.confidence)
    m.text = f'{obj.label} {conf}%'
    return m


class ObjDetMarkers(Node):
    def __init__(self):
        super().__init__('obj_det_markers')
        self.pub = self.create_publisher(MarkerArray, '/zed/obj_det/markers', 10)
        self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self._callback,
            10,
        )

    def _callback(self, msg: ObjectsStamped):
        array = MarkerArray()
        frame_id = msg.header.frame_id
        stamp = msg.header.stamp

        for idx, obj in enumerate(msg.objects):
            if obj.tracking_state == 0:  # OFF — not valid
                continue
            array.markers.append(_box_marker(obj, idx * 2,     frame_id, stamp))
            array.markers.append(_text_marker(obj, idx * 2 + 1, frame_id, stamp))

        self.pub.publish(array)


def main():
    rclpy.init()
    node = ObjDetMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
