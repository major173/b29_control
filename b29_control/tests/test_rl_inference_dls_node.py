#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import rl_inference_node  # noqa: E402
from reach_policy import Config  # noqa: E402


class _Float64MultiArray:
    def __init__(self, data=None):
        self.data = [] if data is None else list(data)


class _JointState:
    def __init__(self, names, position, velocity=None, effort=None):
        self.name = list(names)
        self.position = list(position)
        self.velocity = list(velocity or [0.0] * len(names))
        self.effort = list(effort or [0.0] * len(names))


class _Marker:
    SPHERE = 2
    ADD = 0

    def __init__(self):
        self.header = SimpleNamespace(frame_id="", stamp=None)
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(w=0.0),
        )
        self.scale = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeRospy:
    class Time:
        @staticmethod
        def now():
            return 0.0

    def __init__(self):
        self.publishers = {}
        self.shutdown_callbacks = []

    def Publisher(self, topic, message_type, queue_size=1):
        _ = message_type, queue_size
        publisher = _Publisher()
        self.publishers[topic] = publisher
        return publisher

    def Subscriber(self, *args, **kwargs):
        _ = args, kwargs
        return object()

    def is_shutdown(self):
        return False

    def on_shutdown(self, callback):
        self.shutdown_callbacks.append(callback)

    def loginfo(self, *args, **kwargs):
        _ = args, kwargs

    def loginfo_throttle(self, *args, **kwargs):
        _ = args, kwargs

    def logwarn(self, *args, **kwargs):
        _ = args, kwargs

    def logwarn_throttle(self, *args, **kwargs):
        _ = args, kwargs

    def logerr(self, *args, **kwargs):
        _ = args, kwargs


class DLSNodeSchedulingTest(unittest.TestCase):
    def test_callback_is_state_only_and_output_is_republished(self):
        fake_rospy = _FakeRospy()
        original_import_ros = rl_inference_node._import_ros
        rl_inference_node._import_ros = lambda: (
            fake_rospy, _JointState, _Float64MultiArray, _Marker
        )
        node = None
        try:
            cfg = Config.load(PACKAGE_DIR / "config" / "reach_rl_config.yaml")
            node = rl_inference_node.RLInferenceNode(cfg)
            node._on_target_point(_Float64MultiArray([0.12, 0.02, 0.08]))
            state = _JointState(
                cfg.runtime.active_dof_names,
                [0.0, 0.0, 0.0, 0.0],
            )

            started = time.monotonic()
            node._on_joint_states(state)
            callback_elapsed = time.monotonic() - started
            time.sleep(0.30)

            published = fake_rospy.publishers[cfg.deploy.topics["joint_targets"]].messages
            self.assertLess(callback_elapsed, 0.01)
            self.assertGreaterEqual(len(published), 8)
            serialized = [tuple(round(v, 7) for v in msg.data) for msg in published]
            self.assertLess(len(set(serialized)), len(serialized))

            raw_target = np.asarray(
                fake_rospy.publishers[cfg.deploy.topics["joint_targets_raw"]].messages[-1].data,
                dtype=np.float32,
            )
            action = np.asarray(
                fake_rospy.publishers[cfg.deploy.topics["action_raw"]].messages[-1].data,
                dtype=np.float32,
            )
            expected_action = (
                2.0 * (raw_target - cfg.runtime.hard_lower)
                / (cfg.runtime.hard_upper - cfg.runtime.hard_lower)
                - 1.0
            )
            np.testing.assert_allclose(action, expected_action, atol=1.0e-6)
        finally:
            if node is not None:
                node._shutdown()
            rl_inference_node._import_ros = original_import_ros

    def test_hardware_tracking_watchdog_enters_fault(self):
        fake_rospy = _FakeRospy()
        original_import_ros = rl_inference_node._import_ros
        rl_inference_node._import_ros = lambda: (
            fake_rospy, _JointState, _Float64MultiArray, _Marker
        )
        node = None
        try:
            cfg = Config.load(
                PACKAGE_DIR / "config" / "reach_rl_config.yaml",
                platform="hardware",
            )
            cfg.deploy.watchdog.track_error_persist_s = 0.0
            node = rl_inference_node.RLInferenceNode(cfg)
            zeros = np.zeros(4, dtype=np.float32)
            safe_target = np.full(
                4,
                cfg.deploy.watchdog.track_error_rad + 1.0e-3,
                dtype=np.float32,
            )
            node._last_output = (
                safe_target.copy(),
                safe_target,
                np.zeros(28, dtype=np.float32),
                zeros.copy(),
            )

            node._update_watchdog(zeros, zeros)

            self.assertEqual(
                node._fault_reason,
                "joint target tracking error persisted",
            )
        finally:
            if node is not None:
                node._shutdown()
            rl_inference_node._import_ros = original_import_ros


if __name__ == "__main__":
    unittest.main()
