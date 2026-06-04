import math
import rclpy
from megarover_perception_msgs.msg import PersonTrackArray
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class PersonTracksToMarkers(Node):
    def __init__(self):
        super().__init__('person_tracks_to_markers')

        self.declare_parameter('tracks_topic', '/perception/people/tracks')
        self.declare_parameter('markers_topic', '/perception/people/markers')
        self.declare_parameter('marker_lifetime_sec', 0.0)

        tracks_topic = self.get_parameter('tracks_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self._marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self._active_marker_keys = set()

        self._pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._sub = self.create_subscription(
            PersonTrackArray,
            tracks_topic,
            self._on_tracks,
            10,
        )

        self.get_logger().info(
            f'Publishing person track markers: {tracks_topic} -> {markers_topic}'
        )

    def _on_tracks(self, msg):
        markers = MarkerArray()
        active_marker_keys = set()

        for track in msg.tracks:
            marker_id = int(track.track_id) * 4
            position_key = ('person_track_position', marker_id)
            label_key = ('person_track_label', marker_id + 1)

            active_marker_keys.add(position_key)
            active_marker_keys.add(label_key)
            markers.markers.append(self._sphere_marker(msg, track, marker_id))
            markers.markers.append(self._text_marker(msg, track, marker_id + 1))

            if track.has_velocity_3d and self._has_nonzero_velocity(track):
                active_marker_keys.add(('person_track_velocity', marker_id + 2))
                markers.markers.append(self._velocity_marker(msg, track, marker_id + 2))

            if track.has_bbox_3d:
                active_marker_keys.add(('person_track_bbox_3d', marker_id + 3))
                markers.markers.append(self._bbox_marker(msg, track, marker_id + 3))

        for namespace, marker_id in self._active_marker_keys - active_marker_keys:
            markers.markers.append(self._delete_marker(msg, namespace, marker_id))

        self._active_marker_keys = active_marker_keys
        self._pub.publish(markers)

    def _sphere_marker(self, msg, track, marker_id):
        marker = Marker()
        self._set_marker_header(marker, msg)
        marker.ns = 'person_track_position'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = track.position_3d
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.9
        marker.lifetime = self._lifetime()
        return marker

    def _text_marker(self, msg, track, marker_id):
        marker = Marker()
        self._set_marker_header(marker, msg)
        marker.ns = 'person_track_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = track.position_3d.x
        marker.pose.position.y = track.position_3d.y
        marker.pose.position.z = track.position_3d.z + 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f'id={track.track_id} {track.class_name} '
            f'{track.confidence:.0f}% state={track.tracking_state}'
        )
        marker.lifetime = self._lifetime()
        return marker

    def _velocity_marker(self, msg, track, marker_id):
        marker = Marker()
        self._set_marker_header(marker, msg)
        marker.ns = 'person_track_velocity'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points.append(track.position_3d)

        end = type(track.position_3d)()
        end.x = track.position_3d.x + track.velocity_3d.x
        end.y = track.position_3d.y + track.velocity_3d.y
        end.z = track.position_3d.z + track.velocity_3d.z
        marker.points.append(end)

        marker.scale.x = 0.04
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.7
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime = self._lifetime()
        return marker

    def _bbox_marker(self, msg, track, marker_id):
        marker = Marker()
        self._set_marker_header(marker, msg)
        marker.ns = 'person_track_bbox_3d'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = self._bbox_edges(track.bbox_3d.corners)
        marker.scale.x = 0.035
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.lifetime = self._lifetime()
        return marker

    def _delete_marker(self, msg, namespace, marker_id):
        marker = Marker()
        self._set_marker_header(marker, msg)
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _set_marker_header(marker, msg):
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

    def _lifetime(self):
        lifetime = Marker().lifetime
        lifetime.sec = int(self._marker_lifetime_sec)
        lifetime.nanosec = int((self._marker_lifetime_sec % 1.0) * 1e9)
        return lifetime

    @staticmethod
    def _has_nonzero_velocity(track):
        speed = math.sqrt(
            track.velocity_3d.x ** 2
            + track.velocity_3d.y ** 2
            + track.velocity_3d.z ** 2
        )
        return math.isfinite(speed) and speed > 1e-3

    @staticmethod
    def _bbox_edges(corners):
        if len(corners) != 8:
            return []

        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )

        points = []
        for start, end in edges:
            points.append(Point(x=corners[start].x, y=corners[start].y, z=corners[start].z))
            points.append(Point(x=corners[end].x, y=corners[end].y, z=corners[end].z))
        return points

    @staticmethod
    def _namespace_for_marker_id(marker_id):
        remainder = marker_id % 4
        if remainder == 0:
            return 'person_track_position'
        if remainder == 1:
            return 'person_track_label'
        if remainder == 2:
            return 'person_track_velocity'
        return 'person_track_bbox_3d'


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracksToMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
