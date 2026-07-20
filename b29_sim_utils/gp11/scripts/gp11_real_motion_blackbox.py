#!/usr/bin/env python3
"""Read-only black-box recorder for GP11/B29 real-robot motion debugging.

The recorder correlates MoveIt goals, adapter commands, the SMC effective
hardware targets, encoder-backed joint states and TF poses on one clock.  It
does not create publishers, action clients or service clients.
"""

import csv
import datetime
import json
import math
import os
import threading

import rospy
import tf2_ros
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from moveit_msgs.msg import DisplayTrajectory, ExecuteTrajectoryActionGoal, ExecuteTrajectoryActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback

try:
    from b29_smc_auto_controller.msg import AutoStateTrace, PlannerControlState, PlannerJointCommand
except ImportError:
    AutoStateTrace = None
    PlannerControlState = None
    PlannerJointCommand = None


JOINT_NAMES = (
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
)
TRACE_INDICES = (0, 1, 3, 4)


def _finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else ""
    except (TypeError, ValueError):
        return ""


def _stamp_sec(header):
    stamp = getattr(header, "stamp", None)
    return stamp.to_sec() if stamp is not None and stamp != rospy.Time() else None


def _trajectory_dict(trajectory):
    return {
        "joint_names": list(trajectory.joint_names),
        "points": [
            {
                "t": point.time_from_start.to_sec(),
                "positions": list(point.positions),
                "velocities": list(point.velocities),
                "accelerations": list(point.accelerations),
            }
            for point in trajectory.points
        ],
    }


def _yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MotionBlackbox(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._start = rospy.Time.now()
        self._actual = {}
        self._velocity = {}
        self._adapter_target = None
        self._effective_target = None
        self._planner_state = None
        self._trace = None
        self._last_state_key = None
        self._last_trace_key = None
        self._closing = False

        output_dir = os.path.expanduser(
            rospy.get_param("~output_directory", "~/.ros/gp11_blackbox")
        )
        os.makedirs(output_dir, exist_ok=True)
        run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._sample_path = os.path.join(output_dir, "gp11_motion_{}_samples.csv".format(run_id))
        self._event_path = os.path.join(output_dir, "gp11_motion_{}_events.jsonl".format(run_id))
        self._sample_file = open(self._sample_path, "w", newline="", buffering=1)
        self._event_file = open(self._event_path, "w", buffering=1)
        with open(os.path.join(output_dir, "LATEST"), "w") as latest:
            latest.write(self._sample_path + "\n" + self._event_path + "\n")

        self._world_frame = rospy.get_param("~world_frame", "world")
        self._tf_frames = tuple(rospy.get_param(
            "~tf_frames",
            ["base_link", "left_gripper_tool", "right_gripper_tool"],
        ))
        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        fields = ["wall_time", "ros_time", "elapsed_sec"]
        for prefix in ("actual", "velocity", "adapter_target", "effective_target",
                       "adapter_error", "effective_error"):
            fields.extend("{}_{}".format(prefix, name) for name in JOINT_NAMES)
        fields.extend([
            "session_id", "session_active", "accepting_commands", "last_accepted_sequence",
            "last_rejected_sequence", "reject_reason", "rejection_count", "exit_reason", "command_timeout",
            "smc_state", "command_reason", "output_mode", "stop_all", "freeze_joints",
            "dispatch_attempted", "dispatch_succeeded",
        ])
        fields.extend("session_reference_{}".format(name) for name in JOINT_NAMES)
        fields.append("session_reference_stamp")
        for frame in self._tf_frames:
            safe = frame.replace("/", "_")
            fields.extend("tf_{}_{}".format(safe, axis) for axis in ("x", "y", "z", "yaw"))
        self._writer = csv.DictWriter(self._sample_file, fieldnames=fields)
        self._writer.writeheader()

        self._subscribe()
        rate = max(1.0, float(rospy.get_param("~sample_rate", 20.0)))
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate), self._sample)
        rospy.on_shutdown(self._close)
        self._event("recorder_started", {
            "sample_path": self._sample_path,
            "event_path": self._event_path,
            "world_frame": self._world_frame,
            "tf_frames": list(self._tf_frames),
            "read_only": True,
        })
        rospy.logwarn("[motion blackbox] 纯监听已启动，不会发送控制命令")
        rospy.logwarn("[motion blackbox] samples: %s", self._sample_path)
        rospy.logwarn("[motion blackbox] events : %s", self._event_path)

    def _subscribe(self):
        rospy.Subscriber(rospy.get_param("~joint_states_topic", "/joint_states"),
                         JointState, self._joint_state_cb, queue_size=100)
        rospy.Subscriber(rospy.get_param(
            "~trajectory_goal_topic",
            "/gp11_moveit/reach_arm_controller/follow_joint_trajectory/goal"),
            FollowJointTrajectoryActionGoal, self._trajectory_goal_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param(
            "~trajectory_result_topic",
            "/gp11_moveit/reach_arm_controller/follow_joint_trajectory/result"),
            FollowJointTrajectoryActionResult, self._trajectory_result_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~execute_goal_topic", "/execute_trajectory/goal"),
                         ExecuteTrajectoryActionGoal, self._execute_goal_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~execute_result_topic", "/execute_trajectory/result"),
                         ExecuteTrajectoryActionResult, self._execute_result_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~display_trajectory_topic", "/move_group/display_planned_path"),
                         DisplayTrajectory, self._display_trajectory_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param(
            "~marker_feedback_topic",
            "/gp11_moveit/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback"),
            InteractiveMarkerFeedback, self._marker_feedback_cb, queue_size=100)
        rospy.Subscriber(rospy.get_param(
            "~cartesian_goal_event_topic", "/gp11_moveit/cartesian_goal/events"),
            String, self._cartesian_goal_event_cb, queue_size=100)
        if PlannerJointCommand is None:
            rospy.logerr("[motion blackbox] 找不到 B29 消息类型；仍记录 MoveIt、joint_states 和 TF")
            return
        rospy.Subscriber(rospy.get_param(
            "~planner_command_topic",
            "/b29_controller/b29_smc_auto_controller/planner_joint_command"),
            PlannerJointCommand, self._planner_command_cb, queue_size=200)
        rospy.Subscriber(rospy.get_param(
            "~planner_state_topic",
            "/b29_controller/b29_smc_auto_controller/planner_control_state"),
            PlannerControlState, self._planner_state_cb, queue_size=100)
        rospy.Subscriber(rospy.get_param(
            "~state_trace_topic", "/b29_controller/b29_smc_auto_controller/state_trace"),
            AutoStateTrace, self._trace_cb, queue_size=100)

    def _event(self, kind, data):
        record = {
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "ros_time": rospy.Time.now().to_sec(),
            "elapsed_sec": (rospy.Time.now() - self._start).to_sec(),
            "event": kind,
            "data": data,
        }
        with self._lock:
            if not self._closing and not self._event_file.closed:
                self._event_file.write(
                    json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
                )

    def _joint_state_cb(self, msg):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        with self._lock:
            for name in JOINT_NAMES:
                if name in positions:
                    self._actual[name] = float(positions[name])
                if name in velocities:
                    self._velocity[name] = float(velocities[name])

    def _cartesian_goal_event_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._event("cartesian_goal_event_invalid", {
                "error": str(exc), "raw": msg.data,
            })
            return
        kind = payload.pop("event", "cartesian_goal_event")
        self._event(kind, payload)

    def _planner_command_cb(self, msg):
        target = tuple(float(value) for value in msg.positions)
        with self._lock:
            self._adapter_target = target
        self._event("adapter_command", {
            "header_stamp": _stamp_sec(msg.header), "session_id": int(msg.session_id),
            "sequence": int(msg.sequence), "positions": list(target),
        })

    def _planner_state_cb(self, msg):
        key = (msg.session_id, msg.active, msg.accepting_commands, msg.last_accepted_sequence,
               msg.last_rejected_sequence, msg.reject_reason, msg.rejection_count,
               msg.last_completed_session_id,
               msg.exit_reason)
        with self._lock:
            self._planner_state = msg
            changed = key != self._last_state_key
            self._last_state_key = key
        if changed:
            self._event("planner_state_changed", {
                "session_id": int(msg.session_id), "active": bool(msg.active),
                "accepting_commands": bool(msg.accepting_commands),
                "has_accepted_command": bool(msg.has_accepted_command),
                "last_accepted_sequence": int(msg.last_accepted_sequence),
                "last_rejected_sequence": int(msg.last_rejected_sequence),
                "reject_reason": int(msg.reject_reason),
                "rejection_count": int(msg.rejection_count),
                "last_completed_session_id": int(msg.last_completed_session_id),
                "exit_reason": int(msg.exit_reason), "command_timeout": float(msg.command_timeout),
                "reference_positions": [float(value) for value in msg.reference_positions],
                "reference_stamp": msg.reference_stamp.to_sec(),
            })

    def _trace_cb(self, msg):
        target = tuple(float(msg.joint_targets[index]) for index in TRACE_INDICES)
        key = (msg.current_state, msg.command_reason, msg.output_mode, msg.stop_all,
               msg.freeze_joints, msg.command_dispatch_succeeded)
        with self._lock:
            self._trace = msg
            self._effective_target = target
            changed = key != self._last_trace_key
            self._last_trace_key = key
        if changed:
            self._event("smc_trace_changed", {
                "current_state": msg.current_state, "previous_state": msg.previous_state,
                "last_event": msg.last_event, "transition_reason": msg.transition_reason,
                "command_reason": msg.command_reason, "base_command_reason": msg.base_command_reason,
                "output_mode": msg.output_mode, "stop_all": bool(msg.stop_all),
                "freeze_joints": bool(msg.freeze_joints), "joint_targets": list(msg.joint_targets),
                "dispatch_attempted": bool(msg.command_dispatch_attempted),
                "dispatch_succeeded": bool(msg.command_dispatch_succeeded),
            })

    def _trajectory_goal_cb(self, msg):
        self._event("adapter_action_goal", {
            "goal_id": msg.goal_id.id, "header_stamp": _stamp_sec(msg.header),
            "trajectory": _trajectory_dict(msg.goal.trajectory),
        })

    def _trajectory_result_cb(self, msg):
        self._event("adapter_action_result", {
            "goal_id": msg.status.goal_id.id, "status": int(msg.status.status),
            "status_text": msg.status.text, "error_code": int(msg.result.error_code),
            "error_string": msg.result.error_string,
        })

    def _execute_goal_cb(self, msg):
        self._event("moveit_execute_goal", {
            "goal_id": msg.goal_id.id,
            "trajectory": _trajectory_dict(msg.goal.trajectory.joint_trajectory),
        })

    def _execute_result_cb(self, msg):
        self._event("moveit_execute_result", {
            "goal_id": msg.status.goal_id.id, "status": int(msg.status.status),
            "status_text": msg.status.text, "error_code": int(msg.result.error_code.val),
        })

    def _display_trajectory_cb(self, msg):
        self._event("display_trajectory", {
            "model_id": msg.model_id,
            "trajectories": [_trajectory_dict(item.joint_trajectory) for item in msg.trajectory],
        })

    def _marker_feedback_cb(self, msg):
        pose = msg.pose
        self._event("rviz_marker_feedback", {
            "marker_name": msg.marker_name, "control_name": msg.control_name,
            "event_type": int(msg.event_type), "frame_id": msg.header.frame_id,
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        })

    def _sample(self, _event):
        now = rospy.Time.now()
        row = {
            "wall_time": datetime.datetime.now().isoformat(timespec="milliseconds"),
            "ros_time": now.to_sec(), "elapsed_sec": (now - self._start).to_sec(),
        }
        with self._lock:
            actual = dict(self._actual)
            velocity = dict(self._velocity)
            adapter = self._adapter_target
            effective = self._effective_target
            state = self._planner_state
            trace = self._trace
        for index, name in enumerate(JOINT_NAMES):
            a = actual.get(name)
            row["actual_" + name] = _finite(a)
            row["velocity_" + name] = _finite(velocity.get(name))
            row["adapter_target_" + name] = _finite(adapter[index]) if adapter else ""
            row["effective_target_" + name] = _finite(effective[index]) if effective else ""
            row["adapter_error_" + name] = _finite(adapter[index] - a) if adapter and a is not None else ""
            row["effective_error_" + name] = _finite(effective[index] - a) if effective and a is not None else ""
        if state is not None:
            row.update({
                "session_id": state.session_id, "session_active": int(state.active),
                "accepting_commands": int(state.accepting_commands),
                "last_accepted_sequence": state.last_accepted_sequence,
                "last_rejected_sequence": state.last_rejected_sequence,
                "reject_reason": state.reject_reason, "rejection_count": state.rejection_count,
                "exit_reason": state.exit_reason,
                "command_timeout": state.command_timeout,
                "session_reference_stamp": state.reference_stamp.to_sec(),
            })
            for index, name in enumerate(JOINT_NAMES):
                row["session_reference_" + name] = _finite(state.reference_positions[index])
        if trace is not None:
            row.update({
                "smc_state": trace.current_state, "command_reason": trace.command_reason,
                "output_mode": trace.output_mode, "stop_all": int(trace.stop_all),
                "freeze_joints": int(trace.freeze_joints),
                "dispatch_attempted": int(trace.command_dispatch_attempted),
                "dispatch_succeeded": int(trace.command_dispatch_succeeded),
            })
        for frame in self._tf_frames:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._world_frame, frame, rospy.Time(0), rospy.Duration(0.0))
                p = transform.transform.translation
                q = transform.transform.rotation
                values = (p.x, p.y, p.z, _yaw(q.x, q.y, q.z, q.w))
            except Exception:
                values = ("", "", "", "")
            safe = frame.replace("/", "_")
            for axis, value in zip(("x", "y", "z", "yaw"), values):
                row["tf_{}_{}".format(safe, axis)] = value
        with self._lock:
            if not self._closing and not self._sample_file.closed:
                self._writer.writerow(row)

    def _close(self):
        self._closing = True
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.shutdown()
        with self._lock:
            if not self._sample_file.closed:
                self._sample_file.flush()
                self._event_file.flush()
                self._sample_file.close()
                self._event_file.close()


if __name__ == "__main__":
    rospy.init_node("gp11_real_motion_blackbox")
    MotionBlackbox()
    rospy.spin()
