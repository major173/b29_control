#!/usr/bin/env python3
"""Bridge the real B29 TF tree into GP11's support-anchored world frame.

The real B29 controller owns the dynamic base_link-to-links tree, while the
GP11 planning model intentionally defines the attached gripper as world.
MoveIt's interactive marker is therefore expressed in world and RViz cannot
render it against a base_link fixed frame unless the two roots are connected.

This node publishes only the previously-missing parent transform
world -> base_link. It derives it from the authoritative B29 transform
base_link -> active_anchor on every cycle, so the selected clamped gripper
remains at the configured planning-world anchor pose (the identity by
default). It neither republishes B29 link transforms nor publishes joint
states.
"""

import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from tf.transformations import (
    euler_matrix,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


class RealAnchorTfBridge(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._base_frame = rospy.get_param("~base_frame", "base_link")
        self._anchor_mode = str(rospy.get_param("~anchor_mode", "auto")).strip().lower()
        self._anchor_links = {
            "left": rospy.get_param("~left_anchor_link", "l_gripper_left_drive"),
            "right": rospy.get_param("~right_anchor_link", "r_gripper_left_drive"),
        }
        # This must match RealRobotAnchorPoseProvider exactly.  The planning
        # URDF and the live B29 TF tree then agree even when a calibrated
        # planning-world offset is supplied (identity is the normal setting).
        anchor_xyz = (
            float(rospy.get_param("~anchor_fixed_x", 0.0)),
            float(rospy.get_param("~anchor_fixed_y", 0.0)),
            float(rospy.get_param("~anchor_fixed_z", 0.0)),
        )
        anchor_rpy = (
            float(rospy.get_param("~anchor_fixed_roll", 0.0)),
            float(rospy.get_param("~anchor_fixed_pitch", 0.0)),
            float(rospy.get_param("~anchor_fixed_yaw", 0.0)),
        )
        self._world_from_anchor = translation_matrix(anchor_xyz).dot(
            euler_matrix(*anchor_rpy)
        )
        self._lookup_timeout = max(
            0.0, float(rospy.get_param("~lookup_timeout", 0.1))
        )
        self._rate_hz = max(1.0, float(rospy.get_param("~rate", 50.0)))
        self._left_attached = False
        self._right_attached = False
        self._runtime_anchor = None
        self._last_status = None

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._status_pub = rospy.Publisher(
            "/gp11_moveit/anchor_tf_status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/gp11_moveit/runtime_anchor", String, self._runtime_anchor_cb, queue_size=1
        )
        rospy.Subscriber(
            "/gp11/support_plugin/left/attached",
            Bool,
            self._left_attached_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/gp11/support_plugin/right/attached",
            Bool,
            self._right_attached_cb,
            queue_size=1,
        )

    def _runtime_anchor_cb(self, message):
        side = str(message.data).strip().lower()
        with self._lock:
            self._runtime_anchor = side if side in self._anchor_links else None

    def _left_attached_cb(self, message):
        with self._lock:
            self._left_attached = bool(message.data)

    def _right_attached_cb(self, message):
        with self._lock:
            self._right_attached = bool(message.data)

    def _selected_side(self):
        with self._lock:
            runtime_anchor = self._runtime_anchor
            left_attached = self._left_attached
            right_attached = self._right_attached

        # A previous runtime anchor is only authoritative while that side is
        # still reported attached.  Otherwise an automatic support switch
        # would leave world -> base_link pinned to the old gripper.
        if runtime_anchor == "left" and left_attached:
            return runtime_anchor
        if runtime_anchor == "right" and right_attached:
            return runtime_anchor
        if self._anchor_mode in self._anchor_links:
            return self._anchor_mode
        if left_attached and not right_attached:
            return "left"
        if right_attached and not left_attached:
            return "right"
        return None

    def _publish_status(self, message):
        if message == self._last_status:
            return
        self._last_status = message
        self._status_pub.publish(String(data=message))
        rospy.loginfo(message)

    @staticmethod
    def _matrix_from_transform(transform):
        translation = transform.translation
        rotation = transform.rotation
        return translation_matrix(
            (translation.x, translation.y, translation.z)
        ).dot(
            quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
        )

    def _publish_world_to_base(self, side):
        anchor_link = self._anchor_links[side]
        try:
            base_from_anchor = self._buffer.lookup_transform(
                self._base_frame,
                anchor_link,
                rospy.Time(0),
                rospy.Duration(self._lookup_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            rospy.ROSException,
        ) as exc:
            self._publish_status(
                "Waiting for B29 TF {} -> {}: {}".format(
                    self._base_frame, anchor_link, exc
                )
            )
            return

        world_from_base = self._world_from_anchor.dot(
            inverse_matrix(self._matrix_from_transform(base_from_anchor.transform))
        )
        xyz = translation_from_matrix(world_from_base)
        quaternion = quaternion_from_matrix(world_from_base)

        output = TransformStamped()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = self._world_frame
        output.child_frame_id = self._base_frame
        output.transform.translation.x = float(xyz[0])
        output.transform.translation.y = float(xyz[1])
        output.transform.translation.z = float(xyz[2])
        output.transform.rotation.x = float(quaternion[0])
        output.transform.rotation.y = float(quaternion[1])
        output.transform.rotation.z = float(quaternion[2])
        output.transform.rotation.w = float(quaternion[3])
        self._broadcaster.sendTransform(output)
        self._publish_status(
            "Publishing {} -> {} from {} anchor.".format(
                self._world_frame, self._base_frame, side
            )
        )

    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            side = self._selected_side()
            if side is None:
                self._publish_status(
                    "Waiting for one resolved GP11 anchor before publishing world -> base_link."
                )
            else:
                self._publish_world_to_base(side)
            rate.sleep()


def main():
    rospy.init_node("gp11_real_anchor_tf")
    RealAnchorTfBridge().spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
