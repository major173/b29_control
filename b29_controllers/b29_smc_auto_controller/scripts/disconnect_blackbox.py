#!/usr/bin/env python3
"""Lightweight read-only recorder for B29 cable-disconnect validation."""

import csv
import datetime
import json
import os
import threading
import time

import rospy
from sensor_msgs.msg import JointState

from b29_smc_auto_controller.msg import AutoStateTrace


LEG_JOINTS = (
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
)
TRACE_TARGET_INDICES = (0, 1, 3, 4)


class DisconnectBlackbox(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._positions = {}
        self._velocities = {}
        self._last_sample_wall = None
        self._last_event_key = None
        self._closed = False

        output_directory = os.path.expanduser(
            rospy.get_param("~output_directory", "~/.ros/b29_disconnect_blackbox")
        )
        os.makedirs(output_directory, exist_ok=True)
        run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._sample_path = os.path.join(
            output_directory, "disconnect_{}_samples.csv".format(run_id)
        )
        self._event_path = os.path.join(
            output_directory, "disconnect_{}_events.jsonl".format(run_id)
        )
        self._sample_file = open(self._sample_path, "w", newline="", buffering=1)
        self._event_file = open(self._event_path, "w", buffering=1)
        with open(os.path.join(output_directory, "LATEST"), "w") as latest:
            latest.write(self._sample_path + "\n" + self._event_path + "\n")

        fields = [
            "wall_time", "ros_time", "current_state", "crossing_stage", "crossing_side",
            "disconnect_step", "disconnect_transition_reason", "command_reason",
            "max_abs_pose_joint_velocity", "velocity_threshold",
            "velocity_within_threshold", "velocity_stable_elapsed_sec", "settle_duration",
        ]
        for prefix in ("position", "velocity", "target"):
            fields.extend("{}_{}".format(prefix, name) for name in LEG_JOINTS)
        fields.extend([
            "gravity_compensation_mode", "lower_alive", "imu_ready", "posture_ready",
            "grip_confirmed", "temporary_start_grip_bypass", "joint_fault", "grip_fault",
            "last_failure_reason",
        ])
        self._writer = csv.DictWriter(self._sample_file, fieldnames=fields)
        self._writer.writeheader()

        sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 20.0)))
        self._sample_period = 1.0 / sample_rate
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic", "/joint_states"),
            JointState, self._joint_state_callback, queue_size=100,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~state_trace_topic",
                "/b29_controller/b29_smc_auto_controller/state_trace",
            ),
            AutoStateTrace, self._trace_callback, queue_size=100,
        )
        rospy.on_shutdown(self._close)
        self._event("recorder_started", {
            "sample_path": self._sample_path,
            "event_path": self._event_path,
            "sample_rate": sample_rate,
            "read_only": True,
        })
        rospy.logwarn("[disconnect blackbox] read-only recorder started")
        rospy.logwarn("[disconnect blackbox] samples: %s", self._sample_path)
        rospy.logwarn("[disconnect blackbox] events : %s", self._event_path)

    def _event(self, kind, data):
        record = {
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "ros_time": rospy.Time.now().to_sec(),
            "event": kind,
            "data": data,
        }
        with self._lock:
            if not self._closed:
                self._event_file.write(
                    json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                )

    def _joint_state_callback(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        with self._lock:
            for name in LEG_JOINTS:
                if name in positions:
                    self._positions[name] = float(positions[name])
                if name in velocities:
                    self._velocities[name] = float(velocities[name])

    def _trace_callback(self, message):
        event_key = (
            message.current_state,
            message.obstacle_crossing_stage,
            message.disconnect_step,
            message.disconnect_step_transition_reason,
            message.disconnect_velocity_within_threshold,
            message.last_failure_reason,
        )
        if event_key != self._last_event_key:
            self._last_event_key = event_key
            self._event("disconnect_state_changed", {
                "current_state": message.current_state,
                "crossing_stage": message.obstacle_crossing_stage,
                "crossing_side": message.obstacle_crossing_side,
                "disconnect_step": message.disconnect_step,
                "transition_reason": message.disconnect_step_transition_reason,
                "velocity_within_threshold": bool(message.disconnect_velocity_within_threshold),
                "last_failure_reason": message.last_failure_reason,
            })

        now_wall = time.monotonic()
        if (self._last_sample_wall is not None and
                now_wall - self._last_sample_wall < self._sample_period):
            return
        self._last_sample_wall = now_wall

        with self._lock:
            positions = dict(self._positions)
            velocities = dict(self._velocities)
        row = {
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "ros_time": message.header.stamp.to_sec(),
            "current_state": message.current_state,
            "crossing_stage": message.obstacle_crossing_stage,
            "crossing_side": message.obstacle_crossing_side,
            "disconnect_step": message.disconnect_step,
            "disconnect_transition_reason": message.disconnect_step_transition_reason,
            "command_reason": message.command_reason,
            "max_abs_pose_joint_velocity": message.disconnect_max_abs_pose_joint_velocity,
            "velocity_threshold": message.disconnect_settle_velocity_threshold,
            "velocity_within_threshold": int(message.disconnect_velocity_within_threshold),
            "velocity_stable_elapsed_sec": message.disconnect_velocity_stable_elapsed_sec,
            "settle_duration": message.disconnect_settle_duration,
            "gravity_compensation_mode": message.gravity_compensation_mode,
            "lower_alive": int(message.lower_alive),
            "imu_ready": int(message.imu_ready),
            "posture_ready": int(message.posture_ready),
            "grip_confirmed": int(message.grip_confirmed),
            "temporary_start_grip_bypass": int(
                message.temporary_allow_start_without_grip_confirmed
            ),
            "joint_fault": int(message.joint_fault),
            "grip_fault": int(message.grip_fault),
            "last_failure_reason": message.last_failure_reason,
        }
        for index, name in enumerate(LEG_JOINTS):
            row["position_" + name] = positions.get(name, "")
            row["velocity_" + name] = velocities.get(name, "")
            row["target_" + name] = message.joint_targets[TRACE_TARGET_INDICES[index]]
        with self._lock:
            if not self._closed:
                self._writer.writerow(row)

    def _close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._sample_file.flush()
            self._event_file.flush()
            self._sample_file.close()
            self._event_file.close()


if __name__ == "__main__":
    rospy.init_node("b29_disconnect_blackbox")
    DisconnectBlackbox()
    rospy.spin()
