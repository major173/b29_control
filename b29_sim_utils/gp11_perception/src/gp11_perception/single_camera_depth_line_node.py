"""Single stereo-depth-camera line extraction for GP11 real-world tests."""

from pathlib import Path

import cv2
import image_geometry
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker


def _depth_to_meters(msg, bridge):
    depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if msg.encoding in ("16UC1", "mono16"):
        return depth.astype(np.float32) * 0.001
    return depth.astype(np.float32)


def _transform_matrix(transform):
    matrix = quaternion_matrix(
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
    )
    matrix[0, 3] = transform.transform.translation.x
    matrix[1, 3] = transform.transform.translation.y
    matrix[2, 3] = transform.transform.translation.z
    return matrix


class SingleCameraDepthLineNode:
    def __init__(self):
        self._bridge = CvBridge()
        self._camera_model = image_geometry.PinholeCameraModel()
        self._camera_info_ready = False
        self._background_depth = None
        self._last_axis = None
        self._last_center = None
        self._last_length = None

        self._depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self._camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")
        self._target_point_topic = rospy.get_param("~target_point_topic", "/gp11_perception/target_point")
        self._moveit_target_point_topic = rospy.get_param("~moveit_target_point_topic", "/gp11_moveit/reach_arm/target_point")
        self._debug_mask_topic = rospy.get_param("~debug_mask_topic", "/gp11_perception/debug_mask")
        self._line_marker_topic = rospy.get_param("~line_marker_topic", "/gp11_perception/line_marker")
        self._publish_target_to_moveit = bool(rospy.get_param("~publish_target_to_moveit", False))
        self._output_frame = rospy.get_param("~output_frame", "")
        self._min_depth_m = float(rospy.get_param("~min_depth_m", 0.15))
        self._max_depth_m = float(rospy.get_param("~max_depth_m", 2.5))
        self._foreground_depth_delta_m = float(rospy.get_param("~foreground_depth_delta_m", 0.015))
        self._min_component_area_px = int(rospy.get_param("~min_component_area_px", 150))
        self._pixel_step = max(1, int(rospy.get_param("~pixel_step", 3)))
        self._max_points = max(100, int(rospy.get_param("~max_points", 4000)))
        self._smoothing_alpha = float(np.clip(rospy.get_param("~smoothing_alpha", 0.35), 0.0, 1.0))
        self._target_ratio = float(np.clip(rospy.get_param("~target_ratio", 0.5), 0.0, 1.0))
        self._pixel_roi = self._parse_pixel_roi(rospy.get_param("~pixel_roi", []))
        self._reference_axis_world = self._normalize(np.asarray(rospy.get_param("~reference_axis_world", [0.0, 1.0, 0.0]), dtype=float))

        background_depth_path = str(rospy.get_param("~background_depth_path", "")).strip()
        if background_depth_path:
            self._background_depth = np.load(str(Path(background_depth_path).expanduser().resolve())).astype(np.float32)
            rospy.loginfo("Loaded background depth: %s", background_depth_path)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._target_pub = rospy.Publisher(self._target_point_topic, PointStamped, queue_size=1)
        self._moveit_target_pub = rospy.Publisher(self._moveit_target_point_topic, PointStamped, queue_size=1) if self._publish_target_to_moveit else None
        self._debug_mask_pub = rospy.Publisher(self._debug_mask_topic, Image, queue_size=1)
        self._line_marker_pub = rospy.Publisher(self._line_marker_topic, Marker, queue_size=1)

        rospy.Subscriber(self._camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1)
        rospy.Subscriber(self._depth_topic, Image, self._depth_cb, queue_size=1)

        rospy.loginfo(
            "GP11 single-camera depth line node ready: depth=%s, camera_info=%s, output_frame=%s, publish_to_moveit=%s",
            self._depth_topic,
            self._camera_info_topic,
            self._output_frame or "<camera>",
            self._publish_target_to_moveit,
        )

    @staticmethod
    def _parse_pixel_roi(value):
        if not value:
            return None
        if len(value) != 4:
            raise ValueError("pixel_roi 必须是 [x_min, y_min, x_max, y_max]")
        return [int(v) for v in value]

    @staticmethod
    def _normalize(vector):
        norm = np.linalg.norm(vector)
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return vector / norm

    def _camera_info_cb(self, msg):
        self._camera_model.fromCameraInfo(msg)
        self._camera_info_ready = True

    def _depth_cb(self, msg):
        if not self._camera_info_ready:
            rospy.logwarn_throttle(5.0, "Waiting for CameraInfo before processing depth frames.")
            return

        try:
            depth_m = _depth_to_meters(msg, self._bridge)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Depth conversion failed: %s", exc)
            return

        mask = self._build_foreground_mask(depth_m)
        if mask is None or not np.any(mask):
            rospy.logwarn_throttle(2.0, "No valid foreground mask from current depth frame.")
            return

        selected_mask = self._select_best_component(mask)
        if selected_mask is None or not np.any(selected_mask):
            rospy.logwarn_throttle(2.0, "No elongated foreground component passed filtering.")
            return

        points_camera = self._mask_to_points(depth_m, selected_mask)
        if points_camera.shape[0] < 20:
            rospy.logwarn_throttle(2.0, "Too few 3D points after backprojection: %d", points_camera.shape[0])
            return

        target_frame = self._output_frame or (msg.header.frame_id or self._camera_model.tfFrame())
        points_output = self._transform_points(points_camera, msg.header.frame_id or self._camera_model.tfFrame(), target_frame, msg.header.stamp)
        if points_output is None or points_output.shape[0] < 20:
            return

        fit = self._fit_line(points_output)
        if fit is None:
            rospy.logwarn_throttle(2.0, "3D line fit failed.")
            return

        center, axis, length, p0, p1 = fit
        target = p0 + self._target_ratio * (p1 - p0)

        self._publish_target(target, target_frame, msg.header.stamp)
        self._publish_marker(p0, p1, target, target_frame, msg.header.stamp)
        self._publish_debug_mask(selected_mask, msg.header)

    def _build_foreground_mask(self, depth_m):
        valid = np.isfinite(depth_m) & (depth_m >= self._min_depth_m) & (depth_m <= self._max_depth_m)
        if self._pixel_roi is not None:
            x_min, y_min, x_max, y_max = self._pixel_roi
            roi_mask = np.zeros_like(valid, dtype=bool)
            roi_mask[max(0, y_min):max(0, y_max), max(0, x_min):max(0, x_max)] = True
            valid &= roi_mask

        if not np.any(valid):
            return None

        if self._background_depth is not None:
            if self._background_depth.shape != depth_m.shape:
                rospy.logerr_throttle(5.0, "Background depth shape %s != current depth shape %s", self._background_depth.shape, depth_m.shape)
                return None
            bg_valid = np.isfinite(self._background_depth) & (self._background_depth > 0.0)
            foreground = valid & ((~bg_valid) | ((self._background_depth - depth_m) >= self._foreground_depth_delta_m))
        else:
            nearest_depth = float(np.percentile(depth_m[valid], 10))
            foreground = valid & (depth_m <= nearest_depth + max(self._foreground_depth_delta_m, 0.02))

        mask_u8 = (foreground.astype(np.uint8)) * 255
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        return mask_u8.astype(bool)

    def _select_best_component(self, mask):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        best_mask = None
        best_score = -1.0
        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self._min_component_area_px:
                continue
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            ratio = float(max(w, h)) / float(max(1, min(w, h)))
            score = area * ratio
            if score > best_score:
                best_score = score
                best_mask = labels == label
        return best_mask

    def _mask_to_points(self, depth_m, mask):
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return np.zeros((0, 3), dtype=float)
        stride = max(1, self._pixel_step)
        indices = np.arange(0, xs.size, stride)
        if indices.size > self._max_points:
            indices = np.linspace(0, xs.size - 1, self._max_points, dtype=int)
        xs = xs[indices]
        ys = ys[indices]
        z = depth_m[ys, xs]

        fx = float(self._camera_model.fx())
        fy = float(self._camera_model.fy())
        cx = float(self._camera_model.cx())
        cy = float(self._camera_model.cy())
        x = (xs.astype(np.float32) - cx) * z / fx
        y = (ys.astype(np.float32) - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float64)
        valid = np.all(np.isfinite(points), axis=1)
        return points[valid]

    def _transform_points(self, points, source_frame, target_frame, stamp):
        if not source_frame or source_frame == target_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.2))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "TF lookup failed: %s -> %s (%s)", source_frame, target_frame, exc)
            return None
        matrix = _transform_matrix(transform)
        points_h = np.hstack((points, np.ones((points.shape[0], 1), dtype=float)))
        transformed = points_h.dot(matrix.T)
        return transformed[:, :3]

    def _fit_line(self, points):
        center = np.mean(points, axis=0)
        centered = points - center
        if centered.shape[0] < 3:
            return None
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        axis = self._normalize(vh[0])

        if self._last_axis is not None and np.dot(axis, self._last_axis) < 0.0:
            axis = -axis
        elif self._last_axis is None and np.dot(axis, self._reference_axis_world) < 0.0:
            axis = -axis

        projection = centered.dot(axis)
        lower = float(np.percentile(projection, 2))
        upper = float(np.percentile(projection, 98))
        length = max(upper - lower, 1e-4)

        if self._last_center is not None:
            alpha = self._smoothing_alpha
            center = alpha * center + (1.0 - alpha) * self._last_center
            axis = self._normalize(alpha * axis + (1.0 - alpha) * self._last_axis)
            if self._last_length is not None:
                length = alpha * length + (1.0 - alpha) * self._last_length

        p0 = center - 0.5 * length * axis
        p1 = center + 0.5 * length * axis

        self._last_center = center
        self._last_axis = axis
        self._last_length = length
        return center, axis, length, p0, p1

    def _publish_target(self, point_xyz, frame_id, stamp):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.point.x = float(point_xyz[0])
        msg.point.y = float(point_xyz[1])
        msg.point.z = float(point_xyz[2])
        self._target_pub.publish(msg)
        if self._moveit_target_pub is not None:
            self._moveit_target_pub.publish(msg)

    def _publish_marker(self, p0, p1, target, frame_id, stamp):
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = frame_id
        line.ns = "gp11_perception"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.008
        line.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.95)
        line.points = [
            Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
            Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
        ]
        self._line_marker_pub.publish(line)

        sphere = Marker()
        sphere.header.stamp = stamp
        sphere.header.frame_id = frame_id
        sphere.ns = "gp11_perception"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=float(target[0]), y=float(target[1]), z=float(target[2]))
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.03
        sphere.scale.y = 0.03
        sphere.scale.z = 0.03
        sphere.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.95)
        self._line_marker_pub.publish(sphere)

    def _publish_debug_mask(self, mask, header):
        msg = self._bridge.cv2_to_imgmsg((mask.astype(np.uint8)) * 255, encoding="mono8")
        msg.header = header
        self._debug_mask_pub.publish(msg)


def main():
    rospy.init_node("gp11_single_camera_depth_line")
    SingleCameraDepthLineNode()
    rospy.spin()
