import math
import copy

import rclpy
from geometry_msgs.msg import Point, Vector3
from megarover_perception_msgs.msg import (
    BoundingBox2D,
    BoundingBox3D,
    PersonTrack,
    PersonTrackArray,
)
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped


def _stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_values(values):
    return all(math.isfinite(float(value)) for value in values)


def _point_from_xyz(values):
    point = Point()
    point.x = float(values[0])
    point.y = float(values[1])
    point.z = float(values[2])
    return point


def _vector_from_xyz(values):
    vector = Vector3()
    vector.x = float(values[0])
    vector.y = float(values[1])
    vector.z = float(values[2])
    return vector


class ZedPersonTracks(Node):
    def __init__(self):
        super().__init__('zed_person_tracks')

        self.declare_parameter('input_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('output_topic', '/perception/people/tracks')
        self.declare_parameter('min_confidence', 40.0)
        self.declare_parameter('max_match_distance', 2.0)
        self.declare_parameter('max_track_age_seconds', 3.0)
        self.declare_parameter('publish_searching', False)

        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._max_match_distance = float(self.get_parameter('max_match_distance').value)
        self._max_track_age = float(self.get_parameter('max_track_age_seconds').value)
        self._publish_searching = bool(self.get_parameter('publish_searching').value)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(PersonTrackArray, output_topic, 10)
        self._sub = self.create_subscription(
            ObjectsStamped,
            input_topic,
            self._on_objects,
            10,
        )

        self._next_track_id = 1
        self._tracks = {}

        self.get_logger().info(
            f'Publishing person tracks: {input_topic} -> {output_topic}'
        )

    def _on_objects(self, msg):
        stamp_seconds = _stamp_to_seconds(msg.header.stamp)
        if stamp_seconds <= 0.0:
            stamp_seconds = self.get_clock().now().nanoseconds * 1e-9

        detections = [obj for obj in msg.objects if self._is_person(obj)]
        self._expire_tracks(stamp_seconds)

        assignments = self._assign_tracks(detections, stamp_seconds)

        output = PersonTrackArray()
        output.header = msg.header

        assigned_track_ids = set()
        for obj, track_id in assignments:
            person_track = self._to_person_track(msg.header, obj, track_id)
            self._tracks[track_id]['last_track'] = copy.deepcopy(person_track)
            assigned_track_ids.add(track_id)

            if not self._publish_searching and person_track.tracking_state != 1:
                continue
            output.tracks.append(person_track)

        if self._publish_searching:
            for track_id, track in self._tracks.items():
                if track_id in assigned_track_ids:
                    continue
                last_track = track.get('last_track')
                if last_track is None:
                    continue
                searching_track = copy.deepcopy(last_track)
                searching_track.header = msg.header
                searching_track.tracking_state = 0
                output.tracks.append(searching_track)

        self._pub.publish(output)

    def _is_person(self, obj):
        if float(obj.confidence) < self._min_confidence:
            return False

        label = str(obj.label).lower()
        sublabel = str(obj.sublabel).lower()
        return 'person' in label or 'person' in sublabel

    def _expire_tracks(self, stamp_seconds):
        expired_ids = [
            track_id for track_id, track in self._tracks.items()
            if stamp_seconds - track['last_seen'] > self._max_track_age
        ]
        for track_id in expired_ids:
            del self._tracks[track_id]

    def _assign_tracks(self, detections, stamp_seconds):
        unmatched_track_ids = set(self._tracks.keys())
        assignments = []

        for obj in detections:
            position = tuple(float(value) for value in obj.position)
            track_id = self._find_nearest_track(position, unmatched_track_ids)
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
            else:
                unmatched_track_ids.remove(track_id)

            self._tracks[track_id] = {
                'position': position,
                'last_seen': stamp_seconds,
            }
            assignments.append((obj, track_id))

        return assignments

    def _find_nearest_track(self, position, candidate_track_ids):
        best_track_id = None
        best_distance = self._max_match_distance

        for track_id in candidate_track_ids:
            track_position = self._tracks[track_id]['position']
            distance = math.sqrt(
                (position[0] - track_position[0]) ** 2
                + (position[1] - track_position[1]) ** 2
                + (position[2] - track_position[2]) ** 2
            )
            if distance < best_distance:
                best_distance = distance
                best_track_id = track_id

        return best_track_id

    def _to_person_track(self, header, obj, track_id):
        track = PersonTrack()
        track.header = header
        track.track_id = int(track_id)
        track.class_name = 'person'
        track.confidence = float(obj.confidence)
        track.bbox_2d = self._to_bbox_2d(obj.bounding_box_2d)
        track.position_3d = _point_from_xyz(obj.position)

        velocity = [float(value) for value in obj.velocity]
        track.has_velocity_3d = _finite_values(velocity)
        if track.has_velocity_3d:
            track.velocity_3d = _vector_from_xyz(velocity)

        bbox_3d = self._to_bbox_3d(obj.bounding_box_3d)
        track.has_bbox_3d = bbox_3d is not None
        if track.has_bbox_3d:
            track.bbox_3d = bbox_3d

        track.tracking_state = int(obj.tracking_state)
        return track

    def _to_bbox_2d(self, bbox):
        xs = [float(corner.kp[0]) for corner in bbox.corners]
        ys = [float(corner.kp[1]) for corner in bbox.corners]

        converted = BoundingBox2D()
        if xs and ys and _finite_values(xs + ys):
            converted.x_min = min(xs)
            converted.y_min = min(ys)
            converted.x_max = max(xs)
            converted.y_max = max(ys)
        return converted

    def _to_bbox_3d(self, bbox):
        corners = []
        for corner in bbox.corners:
            xyz = [float(value) for value in corner.kp]
            if not _finite_values(xyz):
                return None
            corners.append(_point_from_xyz(xyz))

        converted = BoundingBox3D()
        converted.corners = corners
        return converted


def main(args=None):
    rclpy.init(args=args)
    node = ZedPersonTracks()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
