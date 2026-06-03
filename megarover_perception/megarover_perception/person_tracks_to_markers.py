import math

import rclpy
from megarover_perception_msgs.msg import PersonTrackArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class PersonTracksToMarkers(Node):
    def __init__(self):
        super().__init__('person_tracks_to_markers')

        self.declare_parameter('tracks_topic', '/perception/people/tracks')
        self.declare_parameter('markers_topic', '/perception/people/markers')
        self.declare_parameter('marker_lifetime_sec', 0.5)
        self.declare_parameter('max_stale_track_id', 80)

        tracks_topic = self.get_parameter('tracks_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        self._marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)
        self._max_stale_track_id = int(self.get_parameter('max_stale_track_id').value)

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
        active_ids = set()

        for track in msg.tracks:
            marker_id = int(track.track_id) * 3
            active_ids.update((marker_id, marker_id + 1, marker_id + 2))

            markers.markers.append(self._sphere_marker(msg, track, marker_id))
            markers.markers.append(self._text_marker(msg, track, marker_id + 1))
            if track.has_velocity_3d and self._has_nonzero_velocity(track):
                markers.markers.append(self._velocity_marker(msg, track, marker_id + 2))
            else:
                markers.markers.append(self._delete_marker(msg, 'person_track_velocity', marker_id + 2))

        max_marker_id = max(self._max_stale_track_id * 3, max(active_ids, default=0) + 3)
        for stale_id in range(0, max_marker_id + 1):
            if stale_id not in active_ids:
                namespace = self._namespace_for_marker_id(stale_id)
                markers.markers.append(self._delete_marker(msg, namespace, stale_id))

        self._pub.publish(markers)

    def _sphere_marker(self, msg, track, marker_id):
        marker = Marker()
        marker.header = msg.header
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
        marker.header = msg.header
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
        marker.header = msg.header
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

    def _delete_marker(self, msg, namespace, marker_id):
        marker = Marker()
        marker.header = msg.header
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

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
    def _namespace_for_marker_id(marker_id):
        remainder = marker_id % 3
        if remainder == 0:
            return 'person_track_position'
        if remainder == 1:
            return 'person_track_label'
        return 'person_track_velocity'


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracksToMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
