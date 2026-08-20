#!/usr/bin/env python3
"""Provide the planning-world pose of the real robot's anchor gripper."""

import numpy
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import (
    quaternion_from_euler,
    quaternion_matrix,
    translation_matrix,
)


def _identity_pose():
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose


def _matrix_from_pose(pose):
    return numpy.dot(
        translation_matrix((pose.position.x, pose.position.y, pose.position.z)),
        quaternion_matrix(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ),
    )


class AnchorPoseProvider(object):
    """Abstract interface for obtaining the anchor link's world pose."""

    def get_anchor_pose(self, side, robot_model_name, anchor_link):
        """Return (anchor_matrix_4x4, geometry_msgs/Pose, rpy_tuple)."""
        raise NotImplementedError()


class RealRobotAnchorPoseProvider(AnchorPoseProvider):
    """Return the configured planning-world pose of the clamped anchor.

    On the real robot the clamped gripper IS the kinematic reference.
    No external localisation is needed — we define the anchor point as
    the planning-frame origin by default, or use a calibrated fixed offset.
    """

    def __init__(self, fixed_xyz=(0.0, 0.0, 0.0), fixed_rpy=(0.0, 0.0, 0.0)):
        self._pose = _identity_pose()
        self._pose.position.x = float(fixed_xyz[0])
        self._pose.position.y = float(fixed_xyz[1])
        self._pose.position.z = float(fixed_xyz[2])
        self._rpy = tuple(float(v) for v in fixed_rpy)
        quaternion = quaternion_from_euler(*self._rpy)
        self._pose.orientation.x = float(quaternion[0])
        self._pose.orientation.y = float(quaternion[1])
        self._pose.orientation.z = float(quaternion[2])
        self._pose.orientation.w = float(quaternion[3])
        self._matrix = _matrix_from_pose(self._pose)

    def get_anchor_pose(self, side, robot_model_name, anchor_link):
        return self._matrix, self._pose, self._rpy


def make_anchor_pose_provider(provider_name, **kwargs):
    """Create the real-robot anchor pose provider."""
    name = str(provider_name).strip().lower()
    if name in ("real_robot", "real"):
        return RealRobotAnchorPoseProvider(**kwargs)
    raise ValueError("Unknown anchor_pose_provider: {}".format(provider_name))
