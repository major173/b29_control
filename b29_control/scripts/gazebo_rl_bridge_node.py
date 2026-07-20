#!/usr/bin/env python3
"""
gazebo_rl_bridge_node.py — Gazebo 执行端（mujoco_sim_node 的替换点）

订阅 /gp11/rl/joint_targets (Float64MultiArray, 4D)，
按 active_dof_names 顺序分拆并转发到 4 个 position_controller command topic。

关节顺序（与 reach_gazebo_config.yaml 的 active_dof_names 一致）：
  [0] left_first_leg_joint
  [1] left_second_leg_joint
  [2] right_first_leg_joint
  [3] right_second_leg_joint

替换逻辑（sim→real）：
  本节点被实机执行器节点替换后，rl_inference_node 零改动。
"""

from __future__ import annotations

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

_CONTROLLER_NS = "/b29_controller"
_JOINT_NAMES = [
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
]
_JOINT_TARGETS_TOPIC = "/gp11/rl/joint_targets"


class GazeboRLBridgeNode:
    def __init__(self) -> None:
        self._pubs = [
            rospy.Publisher(
                f"{_CONTROLLER_NS}/{name}_position_controller/command",
                Float64,
                queue_size=1,
            )
            for name in _JOINT_NAMES
        ]
        self._last_target = None

        rospy.Subscriber(
            _JOINT_TARGETS_TOPIC,
            Float64MultiArray,
            self._on_joint_targets,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "[gazebo_rl_bridge] ready, subscribing %s → %d position controllers",
            _JOINT_TARGETS_TOPIC,
            len(_JOINT_NAMES),
        )

    def _on_joint_targets(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < len(_JOINT_NAMES):
            rospy.logwarn_throttle(
                2.0,
                "[gazebo_rl_bridge] expected %d values, got %d",
                len(_JOINT_NAMES),
                len(msg.data),
            )
            return
        for i, pub in enumerate(self._pubs):
            pub.publish(Float64(data=float(msg.data[i])))
        self._last_target = list(msg.data[: len(_JOINT_NAMES)])


def main() -> None:
    rospy.init_node("gazebo_rl_bridge", anonymous=False)
    node = GazeboRLBridgeNode()
    rospy.spin()
    _ = node


if __name__ == "__main__":
    main()
