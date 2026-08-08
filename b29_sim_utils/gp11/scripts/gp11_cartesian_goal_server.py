#!/usr/bin/env python3
"""Headless Action server for audited GP11 Cartesian position goals."""

import copy
import json
import math
import threading
import time
import uuid

import actionlib
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transforms
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from b29_smc_auto_controller.msg import AutoStateTrace, PlannerControlState
from gp11.cartesian_goal_core import (
    CartesianGoalCore,
    DEFAULT_GOAL_JOINT_DEADBAND,
    DEFAULT_JOINT_POSITION_BOUNDS,
    DEFAULT_LARGE_FLIP_THRESHOLD,
    DEFAULT_MAX_TRAJECTORY_ACCELERATION,
    DEFAULT_MAX_TRAJECTORY_DURATION,
    DEFAULT_MAX_TRAJECTORY_POINT_DELTA,
    DEFAULT_MAX_TRAJECTORY_POINTS,
    DEFAULT_MAX_TRAJECTORY_VELOCITY,
    DEFAULT_POSITIVE_DIRECTION_TOLERANCE,
    REACH_JOINTS,
    Stage,
    audit_trajectory,
    build_move_group_goal,
    commissioned_free_arm_second_goal,
    commissioned_large_flip_direction,
    commissioned_large_flip_joints,
    direction_error,
    finite_point,
    inside_workspace,
    joint_error,
    large_flip_override_expected,
    planned_goal_joint_state,
    planner_control_cancel_reason,
    large_flip_ik_seed_positions,
    reach_joint_positions_from_state,
    tip_link_for_anchor,
    trajectory_goal_within_deadband,
    unwrap_direction_goal,
)
from gp11.msg import ReachPointAction, ReachPointFeedback, ReachPointResult


def _positive_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("{} must be a positive finite number".format(name))
    return value


def _unit_interval_param(name, default):
    value = float(rospy.get_param(name, default))
    if not 0.0 < value <= 1.0:
        raise ValueError("{} must be in (0, 1]".format(name))
    return value


def _vector_param(name, default):
    value = rospy.get_param(name, list(default))
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("{} must be a three-element numeric list".format(name))
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise ValueError("{} must contain finite values".format(name))
    return result


def _joint_bounds_param(name, defaults):
    configured = rospy.get_param(
        name, {joint: list(bounds) for joint, bounds in defaults.items()}
    )
    if not isinstance(configured, dict):
        raise ValueError("{} must be a joint -> [min, max] mapping".format(name))
    bounds = {}
    for joint, fallback in defaults.items():
        value = configured.get(joint, fallback)
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError("{}[{}] must be [min, max]".format(name, joint))
        lower, upper = float(value[0]), float(value[1])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError("{}[{}] must be finite and ordered".format(name, joint))
        bounds[joint] = (lower, upper)
    return bounds


class CartesianGoalServer(object):
    """Serialize, validate, plan, audit and optionally execute one point goal."""

    def __init__(self):
        self._lock = threading.RLock()
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._allowed_input_frames = tuple(rospy.get_param(
            "~allowed_input_frames", [self._world_frame, "base_link"]
        ))
        self._group_name = rospy.get_param("~group_name", "reach_arm")
        self._left_tip_link = rospy.get_param("~left_tip_link", "right_gripper_tool")
        self._right_tip_link = rospy.get_param("~right_tip_link", "left_gripper_tool")
        self._workspace_min = _vector_param("~workspace_min", (-1.50, -1.50, -1.50))
        self._workspace_max = _vector_param("~workspace_max", (1.50, 1.50, 1.50))
        if any(low >= high for low, high in zip(self._workspace_min, self._workspace_max)):
            raise ValueError("workspace_min must be strictly smaller than workspace_max")

        self._default_tolerance = _positive_param("~position_tolerance", 0.008)
        self._min_tolerance = _positive_param("~min_position_tolerance", 0.001)
        self._max_tolerance = _positive_param("~max_position_tolerance", 0.050)
        if not self._min_tolerance <= self._default_tolerance <= self._max_tolerance:
            raise ValueError("position_tolerance must be within configured min/max")
        self._ik_timeout = _positive_param("~ik_timeout", 0.20)
        self._ik_goal_joint_tolerance = _positive_param(
            "~ik_goal_joint_tolerance", 0.005
        )
        self._planning_time = _positive_param("~planning_time", 4.0)
        self._planning_attempts = max(1, int(rospy.get_param("~planning_attempts", 2)))
        self._planning_action_timeout = max(
            self._planning_time + 2.0,
            _positive_param("~planning_action_timeout", 10.0),
        )
        self._execution_action_timeout = _positive_param(
            "~execution_action_timeout", 110.0
        )
        self._velocity_scaling = _unit_interval_param("~velocity_scaling", 0.20)
        self._acceleration_scaling = _unit_interval_param(
            "~acceleration_scaling", 0.15
        )
        self._goal_joint_deadband = _positive_param(
            "~goal_joint_deadband", DEFAULT_GOAL_JOINT_DEADBAND
        )
        self._joint_position_bounds = _joint_bounds_param(
            "~trajectory_joint_position_bounds", DEFAULT_JOINT_POSITION_BOUNDS
        )
        self._max_trajectory_velocity = _positive_param(
            "~max_trajectory_velocity", DEFAULT_MAX_TRAJECTORY_VELOCITY
        )
        self._max_trajectory_acceleration = _positive_param(
            "~max_trajectory_acceleration", DEFAULT_MAX_TRAJECTORY_ACCELERATION
        )
        self._max_trajectory_point_delta = _positive_param(
            "~max_trajectory_point_delta", DEFAULT_MAX_TRAJECTORY_POINT_DELTA
        )
        self._max_trajectory_duration = _positive_param(
            "~max_trajectory_duration", DEFAULT_MAX_TRAJECTORY_DURATION
        )
        self._max_trajectory_points = max(
            1, int(rospy.get_param(
                "~max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS
            ))
        )
        self._large_flip_threshold = _positive_param(
            "~large_flip_threshold", DEFAULT_LARGE_FLIP_THRESHOLD
        )
        legacy_require_direction = bool(rospy.get_param(
            "~require_positive_large_flip_direction", True
        ))
        self._require_commissioned_large_flip_direction = bool(rospy.get_param(
            "~require_commissioned_large_flip_direction",
            legacy_require_direction,
        ))
        self._flip_ik_seed_fractions = tuple(float(value) for value in rospy.get_param(
            "~flip_ik_seed_fractions", [0.0, 0.50, 0.90]
        ))
        if (not 1 <= len(self._flip_ik_seed_fractions) <= 3 or
                any(not math.isfinite(value) or not 0.0 <= value < 1.0
                    for value in self._flip_ik_seed_fractions)):
            raise ValueError(
                "flip_ik_seed_fractions must contain one to three values in [0, 1)"
            )
        self._left_anchor_free_second_target = float(rospy.get_param(
            "~flip_free_arm_second_target", -math.pi
        ))
        self._right_anchor_free_second_delta = float(rospy.get_param(
            "~flip_right_anchor_free_arm_second_delta", -math.pi
        ))
        if (not math.isfinite(self._left_anchor_free_second_target) or
                not math.isfinite(self._right_anchor_free_second_delta) or
                not math.isclose(
                    self._left_anchor_free_second_target, -math.pi,
                    rel_tol=0.0, abs_tol=1e-9,
                ) or
                not math.isclose(
                    self._right_anchor_free_second_delta, -math.pi,
                    rel_tol=0.0, abs_tol=1e-9,
                )):
            raise ValueError(
                "commissioned free-arm requirements are fixed: pass-1 "
                "right_second target=-pi and pass-2 left_second delta=-pi"
            )
        legacy_direction_tolerance = _positive_param(
            "~positive_direction_tolerance", DEFAULT_POSITIVE_DIRECTION_TOLERANCE
        )
        self._direction_tolerance = _positive_param(
            "~direction_tolerance", legacy_direction_tolerance
        )
        legacy_constraint_margin = _positive_param(
            "~positive_direction_constraint_margin",
            DEFAULT_POSITIVE_DIRECTION_TOLERANCE,
        )
        self._direction_constraint_margin = _positive_param(
            "~direction_constraint_margin", legacy_constraint_margin
        )
        self._joint_state_timeout = _positive_param("~joint_state_timeout", 0.5)
        self._control_state_timeout = _positive_param("~control_state_timeout", 0.5)
        self._target_max_age = _positive_param("~target_max_age", 1.0)
        self._tf_timeout = _positive_param("~tf_timeout", 0.5)
        self._session_rearm_timeout = _positive_param("~session_rearm_timeout", 3.0)
        self._cached_plan_timeout = _positive_param("~cached_plan_timeout", 120.0)
        self._cached_plan_start_tolerance = _positive_param(
            "~cached_plan_start_tolerance", math.radians(25.0)
        )
        self._allow_direct_execution = bool(rospy.get_param(
            "~allow_direct_execution", False
        ))

        self._anchor_side = ""
        self._anchor_receipt = 0.0
        self._joint_positions = {}
        self._joint_receipt = 0.0
        self._planner_state = None
        self._planner_receipt = 0.0
        self._trace = None
        self._trace_receipt = 0.0
        self._accepted_anchor = ""
        self._operation_id = ""
        self._operation_execute = False
        self._execution_session_id = 0
        self._cancel_reason = ""
        self._cached_plan = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._core = CartesianGoalCore(
            rospy.get_param("~move_group_action", "/move_group"),
            rospy.get_param("~execute_action", "/execute_trajectory"),
            rospy.get_param("~ik_service", "/compute_ik"),
        )
        self._event_pub = rospy.Publisher(
            rospy.get_param(
                "~event_topic", "/gp11_moveit/cartesian_goal/events"
            ),
            String,
            queue_size=50,
        )

        rospy.Subscriber(
            rospy.get_param("~anchor_topic", "/gp11_moveit/runtime_anchor"),
            String, self._anchor_cb, queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic", "/joint_states"),
            JointState, self._joint_state_cb, queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~planner_state_topic",
                "/b29_controller/b29_smc_auto_controller/planner_control_state",
            ),
            PlannerControlState, self._planner_state_cb, queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~state_trace_topic",
                "/b29_controller/b29_smc_auto_controller/state_trace",
            ),
            AutoStateTrace, self._trace_cb, queue_size=20,
        )

        action_name = rospy.get_param("~action_name", "/gp11_moveit/reach_point")
        self._server = actionlib.SimpleActionServer(
            action_name, ReachPointAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._server.start()
        rospy.on_shutdown(self._core.cancel_active_operation)
        rospy.loginfo(
            "GP11 headless Cartesian goal server ready: action=%s, allowed_frames=%s",
            action_name, self._allowed_input_frames,
        )

    def _event(self, event, **data):
        record = {
            "event": event,
            "operation_id": self._operation_id,
            "ros_time": rospy.Time.now().to_sec(),
        }
        record.update(data)
        self._event_pub.publish(String(
            data=json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        ))

    def _anchor_cb(self, msg):
        side = msg.data.strip().lower()
        if side not in ("left", "right"):
            side = ""
        with self._lock:
            self._anchor_side = side
            self._anchor_receipt = time.monotonic()
            if self._accepted_anchor and side != self._accepted_anchor:
                self._cancel_reason = "runtime_anchor changed during operation"
        if self._cancel_reason:
            self._core.cancel_active_operation()

    def _joint_state_cb(self, msg):
        positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in REACH_JOINTS and math.isfinite(position):
                positions[name] = float(position)
        with self._lock:
            self._joint_positions.update(positions)
            self._joint_receipt = time.monotonic()

    def _planner_state_cb(self, msg):
        with self._lock:
            self._planner_state = copy.deepcopy(msg)
            self._planner_receipt = time.monotonic()

    def _trace_cb(self, msg):
        with self._lock:
            self._trace = copy.deepcopy(msg)
            self._trace_receipt = time.monotonic()

    def _feedback(self, stage, message):
        feedback = ReachPointFeedback()
        feedback.stage = stage
        feedback.stage_name = Stage.NAMES[stage]
        feedback.message = message
        self._server.publish_feedback(feedback)
        rospy.loginfo("[cartesian goal:%s] %s: %s",
                      self._operation_id, feedback.stage_name, message)

    def _base_result(self, stage, message, success=False):
        result = ReachPointResult()
        result.success = success
        result.final_stage = stage
        result.moveit_error_code = 0
        result.message = message
        with self._lock:
            result.accepted_anchor_side = self._accepted_anchor
        result.accepted_tip_link = tip_link_for_anchor(
            result.accepted_anchor_side, self._left_tip_link, self._right_tip_link
        )
        return result

    def _finish(self, result, status="aborted"):
        self._event(
            "cartesian_goal_result",
            success=bool(result.success),
            final_stage=int(result.final_stage),
            moveit_error_code=int(result.moveit_error_code),
            message=result.message,
            large_flip_override_expected=bool(result.large_flip_override_expected),
            plan_id=result.plan_id,
        )
        if status == "succeeded":
            self._server.set_succeeded(result, result.message)
        elif status == "preempted":
            self._server.set_preempted(result, result.message)
        else:
            self._server.set_aborted(result, result.message)
        with self._lock:
            self._accepted_anchor = ""
            self._operation_execute = False
            self._execution_session_id = 0
            self._cancel_reason = ""

    def _cancel_requested(self):
        if self._server.is_preempt_requested():
            return "client preempt requested"
        with self._lock:
            if self._cancel_reason:
                return self._cancel_reason
            accepted_anchor = self._accepted_anchor
            anchor_side = self._anchor_side
            trace = copy.deepcopy(self._trace)
            trace_age = time.monotonic() - self._trace_receipt
            planner = copy.deepcopy(self._planner_state)
            planner_age = time.monotonic() - self._planner_receipt
            operation_execute = self._operation_execute
            execution_session_id = self._execution_session_id
        if accepted_anchor and anchor_side != accepted_anchor:
            return "runtime_anchor changed during operation"
        if trace is not None and trace_age <= self._control_state_timeout:
            state = trace.current_state.strip().lower()
            if state in ("commsloss", "safestop"):
                return "SMC entered {}".format(trace.current_state)
            if not trace.lower_alive:
                return "lower_alive became false"
            if trace.software_emergency_stop_latched:
                return "software emergency stop latched"
        if (operation_execute and execution_session_id and planner is not None and
                planner_age <= self._control_state_timeout):
            reason = planner_control_cancel_reason(
                planner.active,
                planner.session_id,
                execution_session_id,
                planner.last_completed_session_id,
                planner.exit_reason,
                PlannerControlState.EXIT_COMPLETED,
            )
            if reason:
                return reason
        return ""

    def _common_state_snapshot(self):
        now = time.monotonic()
        with self._lock:
            side = self._anchor_side
            joints = dict(self._joint_positions)
            joint_age = now - self._joint_receipt
            planner = copy.deepcopy(self._planner_state)
            planner_age = now - self._planner_receipt
            trace = copy.deepcopy(self._trace)
            trace_age = now - self._trace_receipt
        if side not in ("left", "right"):
            return None, "runtime_anchor is not uniquely left or right"
        if joint_age > self._joint_state_timeout:
            return None, "joint_states are missing or stale"
        missing = [name for name in REACH_JOINTS if name not in joints]
        if missing:
            return None, "joint_states missing: {}".format(", ".join(missing))
        if planner is None or planner_age > self._control_state_timeout:
            return None, "PlannerControlState is missing or stale"
        if trace is None or trace_age > self._control_state_timeout:
            return None, "AutoStateTrace is missing or stale"
        state = trace.current_state.strip().lower()
        if state in ("commsloss", "safestop"):
            return None, "SMC is in {}".format(trace.current_state)
        if not trace.lower_alive:
            return None, "lower_alive is false"
        if not trace.imu_ready:
            return None, "imu_ready is false"
        if trace.joint_fault or trace.grip_fault:
            return None, "joint_fault or grip_fault is active"
        if trace.software_emergency_stop_latched:
            return None, "software emergency stop is latched"
        expected_anchor = {
            PlannerControlState.CROSSING_SIDE_RIGHT: "left",
            PlannerControlState.CROSSING_SIDE_LEFT: "right",
        }.get(planner.crossing_side)
        if expected_anchor and side != expected_anchor:
            return None, (
                "runtime_anchor {} does not match PlannerControl crossing "
                "side {} (expected {})"
            ).format(side, planner.crossing_side, expected_anchor)
        return {
            "anchor": side,
            "joints": joints,
            "planner": planner,
            "trace": trace,
        }, ""

    @staticmethod
    def _execution_state_error(snapshot):
        planner = snapshot["planner"]
        if not planner.active:
            return "PlannerControl is not active"
        if not planner.accepting_commands:
            return "PlannerControl is not accepting commands"
        if planner.has_accepted_command:
            return "PlannerControl session has already accepted a command"
        return ""

    def _transform_target(self, point):
        frame_id = point.header.frame_id.strip()
        if not frame_id:
            raise ValueError("target frame_id is empty")
        if frame_id not in self._allowed_input_frames:
            raise ValueError("target frame {} is not allowed".format(frame_id))
        if not finite_point(point.point):
            raise ValueError("target contains NaN or infinity")
        if point.header.stamp != rospy.Time(0):
            age = (rospy.Time.now() - point.header.stamp).to_sec()
            if age < -0.1:
                raise ValueError("target timestamp is in the future")
            if age > self._target_max_age:
                raise ValueError("target is stale ({:.3f}s old)".format(age))
        if frame_id == self._world_frame:
            transformed = copy.deepcopy(point)
        else:
            transformed = self._tf_buffer.transform(
                point, self._world_frame, rospy.Duration(self._tf_timeout)
            )
        target = PoseStamped()
        target.header = transformed.header
        target.header.frame_id = self._world_frame
        target.pose.position = transformed.point
        target.pose.orientation = Quaternion(w=1.0)
        return target

    def _preempt_result(self, result, reason):
        result.success = False
        result.final_stage = Stage.PREEMPTED
        result.message = reason
        self._event("cartesian_goal_cancelled", reason=reason)
        self._finish(result, "preempted")

    def _execute_cb(self, goal):
        try:
            self._execute_goal(goal)
        except Exception as exc:
            self._core.cancel_active_operation()
            rospy.logerr("Unhandled Cartesian goal failure: %s", exc)
            if self._server.is_active():
                result = self._base_result(
                    Stage.FAILED, "internal Cartesian goal error: {}".format(exc)
                )
                self._finish(result)

    def _invalidate_cached_plan(self, reason):
        with self._lock:
            cached = self._cached_plan
            self._cached_plan = None
        if cached is not None:
            self._event(
                "cartesian_goal_plan_invalidated",
                plan_id=cached["plan_id"], reason=reason,
            )

    def _store_cached_plan(
            self, trajectory, target, result, snapshot, audit_message,
            direction_signs=None):
        plan_id = uuid.uuid4().hex
        cached = {
            "plan_id": plan_id,
            "created_monotonic": time.monotonic(),
            "trajectory": copy.deepcopy(trajectory),
            "target": copy.deepcopy(target),
            "anchor": snapshot["anchor"],
            "tip_link": result.accepted_tip_link,
            "start_joints": dict(snapshot["joints"]),
            "planned_goal_joint_state": copy.deepcopy(
                result.planned_goal_joint_state
            ),
            "large_flip_override_expected": bool(
                result.large_flip_override_expected
            ),
            "moveit_error_code": int(result.moveit_error_code),
            "audit_message": audit_message,
            "direction_signs": dict(direction_signs or {}),
        }
        with self._lock:
            self._cached_plan = cached
        self._event(
            "cartesian_goal_plan_cached",
            plan_id=plan_id,
            valid_for=self._cached_plan_timeout,
            anchor=cached["anchor"],
            start_joint_positions=[
                cached["start_joints"][name] for name in REACH_JOINTS
            ],
        )
        return plan_id

    def _commissioned_free_arm_goal(self, snapshot, target_profile):
        free_second, base_target = commissioned_free_arm_second_goal(
            snapshot["joints"],
            snapshot["anchor"],
            self._left_anchor_free_second_target,
            self._right_anchor_free_second_delta,
            left_anchor_relative_to_live=(
                target_profile == "reverse" and
                snapshot["anchor"] == "left"
            ),
        )
        adjusted = unwrap_direction_goal(
            snapshot["joints"],
            {free_second: base_target},
            {free_second: -1.0},
            self._joint_position_bounds,
            self._large_flip_threshold,
        )
        return free_second, adjusted[free_second]

    def _commissioned_branch_ik(
            self, initial_response, initial_positions, target, tip_link,
            snapshot):
        """Select the required signed IK branch without replacing the IK goal."""
        if not self._require_commissioned_large_flip_direction:
            return initial_response, initial_positions, {}, ""
        required = commissioned_large_flip_joints(
            snapshot["anchor"], initial_positions, snapshot["joints"],
            self._large_flip_threshold,
        )
        if not required:
            return None, None, {}, (
                "目标点IK没有形成要求的大翻越：当前锁定侧second相对实时起点"
                "的最短角差未超过{:.1f}度；未生成轨迹"
            ).format(math.degrees(self._large_flip_threshold))

        support_direction = commissioned_large_flip_direction(
            snapshot["anchor"]
        )
        direction_signs = dict(
            (joint_name, support_direction) for joint_name in required
        )

        try:
            positions = unwrap_direction_goal(
                snapshot["joints"], initial_positions, direction_signs,
                self._joint_position_bounds, self._large_flip_threshold,
            )
        except ValueError as exc:
            return None, None, direction_signs, (
                "无法将连续关节IK解展开到指定方向分支：{}；未生成轨迹".format(exc)
            )

        response = copy.deepcopy(initial_response)
        response_positions = list(response.solution.joint_state.position)
        response_indices = {
            name: index
            for index, name in enumerate(response.solution.joint_state.name)
        }
        for joint_name in required:
            response_positions[response_indices[joint_name]] = positions[joint_name]
        response.solution.joint_state.position = response_positions
        self._event(
            "cartesian_goal_commissioned_ik_unwrapped",
            anchor=snapshot["anchor"],
            joint_names=list(required),
            direction_signs=[direction_signs[name] for name in required],
            original_positions=[initial_positions[name] for name in required],
            unwrapped_positions=[positions[name] for name in required],
            start_positions=[snapshot["joints"][name] for name in required],
        )
        rospy.loginfo(
            "Unwrapped continuous-joint IK onto the commissioned %s branch: %s",
            "counter-clockwise" if support_direction > 0.0 else "clockwise",
            ", ".join(
                "{}={:+.6f}".format(name, positions[name])
                for name in required
            ),
        )
        return response, positions, direction_signs, ""

    def _retry_no_solution_ik(self, initial_response, initial_error,
                              target, tip_link, snapshot, target_profile):
        """Retry collision-checked IK with commissioned signed-flip seeds."""
        if (not initial_error or initial_response is None or
                int(initial_response.error_code.val) != MoveItErrorCodes.NO_IK_SOLUTION or
                snapshot["anchor"] not in ("left", "right") or
                not self._require_commissioned_large_flip_direction):
            return initial_response, initial_error

        rospy.logwarn(
            "Live-state IK returned NO_IK_SOLUTION; trying %d commissioned flip seed(s)",
            len(self._flip_ik_seed_fractions),
        )
        last_response = initial_response
        last_error = initial_error
        try:
            _free_second, free_second_target = self._commissioned_free_arm_goal(
                snapshot, target_profile
            )
        except ValueError as exc:
            return initial_response, "备用IK自由臂目标生成失败：{}".format(exc)
        for attempt, fraction in enumerate(self._flip_ik_seed_fractions, 1):
            try:
                seed = large_flip_ik_seed_positions(
                    snapshot["joints"], fraction,
                    snapshot["anchor"],
                    free_second_target,
                )
            except ValueError as exc:
                return initial_response, "备用IK种子生成失败：{}".format(exc)

            response, error = self._core.check_ik(
                target, tip_link, self._group_name, seed, self._ik_timeout,
            )
            code = 0 if response is None else int(response.error_code.val)
            solution_positions = []
            if not error:
                try:
                    positions = reach_joint_positions_from_state(response.solution)
                    solution_positions = [positions[name] for name in REACH_JOINTS]
                except ValueError as exc:
                    error = "IK result rejected: {}".format(exc)
            self._event(
                "cartesian_goal_flip_ik_seed_attempt",
                anchor=snapshot["anchor"],
                attempt=attempt,
                seed_fraction=fraction,
                seed_joint_positions=[seed[name] for name in REACH_JOINTS],
                success=not bool(error),
                moveit_error_code=code,
                message=error,
                solution_joint_positions=solution_positions,
            )
            if not error:
                rospy.loginfo(
                    "Commissioned flip IK seed %d/%d succeeded (fraction=%.2f)",
                    attempt, len(self._flip_ik_seed_fractions), fraction,
                )
                return response, ""
            last_response = response
            last_error = error

        return last_response, (
            "{}；{}个备用种子均未找到IK解，未生成或发布轨迹"
        ).format(last_error, len(self._flip_ik_seed_fractions))

    def _fix_commissioned_free_arm_second(
            self, response, positions, snapshot, direction_signs,
            target_profile):
        """Fix the free-arm second joint to the commissioned clockwise branch."""
        support_second = {
            "left": "left_second_leg_joint",
            "right": "right_second_leg_joint",
        }.get(snapshot["anchor"])
        if not support_second or support_second not in direction_signs:
            return response, positions, direction_signs, ""

        try:
            free_second, free_second_target = self._commissioned_free_arm_goal(
                snapshot, target_profile
            )
        except ValueError as exc:
            return None, None, direction_signs, (
                "自由臂second顺时针目标生成失败：{}；未生成轨迹".format(exc)
            )

        adjusted = dict(positions)
        original = adjusted[free_second]
        adjusted[free_second] = free_second_target
        direction_signs = dict(direction_signs)
        direction_signs[free_second] = -1.0
        response = copy.deepcopy(response)
        indices = {
            name: index
            for index, name in enumerate(response.solution.joint_state.name)
        }
        response_positions = list(response.solution.joint_state.position)
        response_positions[indices[free_second]] = adjusted[free_second]
        response.solution.joint_state.position = response_positions
        self._event(
            "cartesian_goal_free_arm_second_fixed",
            anchor=snapshot["anchor"],
            joint_name=free_second,
            original_position=original,
            fixed_position=adjusted[free_second],
            start_position=snapshot["joints"][free_second],
            commanded_delta=(
                adjusted[free_second] - snapshot["joints"][free_second]
            ),
        )
        rospy.loginfo(
            "Fixed commissioned %s free-arm second goal to %.6f rad",
            snapshot["anchor"], adjusted[free_second],
        )
        return response, adjusted, direction_signs, ""

    def _load_cached_plan(self, plan_id):
        with self._lock:
            cached = copy.deepcopy(self._cached_plan)
            if cached is None:
                return None, "no audited plan is cached"
            if cached["plan_id"] != plan_id:
                return None, "plan_id does not match the currently cached plan"
            age = time.monotonic() - cached["created_monotonic"]
            if age > self._cached_plan_timeout:
                self._cached_plan = None
                return None, "cached plan expired ({:.1f}s old)".format(age)
        cached["age"] = age
        return cached, ""

    def _consume_cached_plan(self, plan_id):
        with self._lock:
            if (self._cached_plan is None or
                    self._cached_plan["plan_id"] != plan_id):
                return False
            self._cached_plan = None
        self._event("cartesian_goal_plan_consumed", plan_id=plan_id)
        return True

    def _cached_start_state_error(self, cached, actual_positions):
        errors = {}
        for name in REACH_JOINTS:
            if name not in cached["start_joints"] or name not in actual_positions:
                return "cached or live start state is missing {}".format(name)
            errors[name] = abs(joint_error(
                name, cached["start_joints"][name], actual_positions[name]
            ))
        worst_name = max(errors, key=errors.get)
        worst_error = errors[worst_name]
        if worst_error > self._cached_plan_start_tolerance:
            return (
                "live start drift for {} is {:.4f} rad, above {:.4f} rad"
            ).format(
                worst_name, worst_error, self._cached_plan_start_tolerance
            )
        return ""

    def _execute_goal(self, goal):
        self._operation_id = uuid.uuid4().hex
        approved_plan_id = goal.approved_plan_id.strip()
        target_profile = goal.target_profile.strip() or "default_forward"
        with self._lock:
            self._accepted_anchor = ""
            self._operation_execute = bool(goal.execute)
            self._execution_session_id = 0
            self._cancel_reason = ""
        self._event(
            "cartesian_goal_received",
            frame_id=goal.target.header.frame_id,
            point=[goal.target.point.x, goal.target.point.y, goal.target.point.z],
            execute=bool(goal.execute),
            approved_plan_id=approved_plan_id,
            target_profile=target_profile,
            position_tolerance=float(goal.position_tolerance),
        )
        if target_profile not in ("default_forward", "reverse"):
            result = self._base_result(Stage.REJECTED, "invalid target profile")
            result.message = (
                "target_profile must be default_forward or reverse"
            )
            self._finish(result)
            return
        if approved_plan_id:
            if not goal.execute:
                result = self._base_result(Stage.REJECTED, "invalid approval request")
                result.plan_id = approved_plan_id
                result.message = "approved_plan_id requires execute=true"
                self._finish(result)
                return
            self._execute_cached_plan(approved_plan_id)
            return
        if goal.execute:
            if not self._allow_direct_execution:
                result = self._base_result(Stage.REJECTED, "direct execution disabled")
                result.message = "direct point execution is disabled by configuration"
                self._finish(result)
                return
            self._plan_goal(
                goal, execute_after_plan=True,
                target_profile=target_profile,
            )
            return
        self._plan_goal(
            goal, execute_after_plan=False,
            target_profile=target_profile,
        )

    def _plan_goal(self, goal, execute_after_plan=False,
                   target_profile="default_forward"):
        self._invalidate_cached_plan("new planning request")
        result = self._base_result(Stage.WAITING_STATE, "planning request received")
        tolerance = goal.position_tolerance or self._default_tolerance
        if not math.isfinite(tolerance) or not self._min_tolerance <= tolerance <= self._max_tolerance:
            result.final_stage = Stage.REJECTED
            result.message = "position_tolerance must be in [{:.3f}, {:.3f}]".format(
                self._min_tolerance, self._max_tolerance
            )
            self._finish(result)
            return

        self._feedback(Stage.WAITING_STATE, "checking live robot and control state")
        snapshot, error = self._common_state_snapshot()
        if not error and execute_after_plan:
            error = self._execution_state_error(snapshot)
        if error:
            result.final_stage = Stage.REJECTED
            result.message = error
            self._finish(result)
            return
        with self._lock:
            self._accepted_anchor = snapshot["anchor"]
        result.accepted_anchor_side = snapshot["anchor"]
        result.accepted_tip_link = tip_link_for_anchor(
            snapshot["anchor"], self._left_tip_link, self._right_tip_link
        )

        try:
            self._feedback(Stage.TRANSFORMING_TARGET, "transforming target into world")
            target = self._transform_target(goal.target)
        except (ValueError, tf2_ros.TransformException) as exc:
            result.final_stage = Stage.REJECTED
            result.message = "target transform rejected: {}".format(exc)
            self._finish(result)
            return
        result.target_in_world = copy.deepcopy(target)
        self._event(
            "cartesian_goal_transformed",
            frame_id=self._world_frame,
            point=[target.pose.position.x, target.pose.position.y, target.pose.position.z],
            anchor=snapshot["anchor"], tip_link=result.accepted_tip_link,
            target_profile=target_profile,
        )
        if not inside_workspace(target.pose.position, self._workspace_min, self._workspace_max):
            result.final_stage = Stage.REJECTED
            result.message = "target is outside configured workspace"
            self._finish(result)
            return
        reason = self._cancel_requested()
        if reason:
            self._preempt_result(result, reason)
            return

        self._feedback(Stage.CHECKING_IK, "checking IK and endpoint collision")
        ik_response, error = self._core.check_ik(
            target, result.accepted_tip_link, self._group_name,
            snapshot["joints"], self._ik_timeout,
        )
        ik_response, error = self._retry_no_solution_ik(
            ik_response, error, target, result.accepted_tip_link, snapshot,
            target_profile,
        )
        ik_code = 0 if ik_response is None else int(ik_response.error_code.val)
        self._event("cartesian_goal_ik_result", success=not bool(error),
                    moveit_error_code=ik_code, message=error)
        if error:
            result.final_stage = Stage.REJECTED
            result.moveit_error_code = ik_code
            result.message = error
            self._finish(result)
            return

        try:
            ik_goal_positions = reach_joint_positions_from_state(
                ik_response.solution
            )
        except ValueError as exc:
            result.final_stage = Stage.REJECTED
            result.moveit_error_code = ik_code
            result.message = "IK result rejected: {}".format(exc)
            self._finish(result)
            return
        ik_response, ik_goal_positions, direction_signs, error = (
            self._commissioned_branch_ik(
                ik_response, ik_goal_positions, target,
                result.accepted_tip_link, snapshot,
            )
        )
        if error:
            result.final_stage = Stage.REJECTED
            result.moveit_error_code = ik_code
            result.message = error
            self._finish(result)
            return
        ik_response, ik_goal_positions, direction_signs, error = (
            self._fix_commissioned_free_arm_second(
                ik_response, ik_goal_positions, snapshot, direction_signs,
                target_profile,
            )
        )
        if error:
            result.final_stage = Stage.REJECTED
            result.moveit_error_code = ik_code
            result.message = error
            self._finish(result)
            return
        self._event(
            "cartesian_goal_ik_solution",
            joint_names=list(REACH_JOINTS),
            joint_positions=[ik_goal_positions[name] for name in REACH_JOINTS],
            direction_signs=direction_signs,
        )

        self._feedback(
            Stage.PLANNING,
            "planning to the collision-checked IK joint solution",
        )
        plan_goal = build_move_group_goal(
            target, result.accepted_tip_link, self._group_name,
            self._workspace_min, self._workspace_max, tolerance,
            self._planning_attempts, self._planning_time,
            self._velocity_scaling, self._acceleration_scaling,
            goal_joint_positions=ik_goal_positions,
            goal_joint_tolerance=self._ik_goal_joint_tolerance,
            direction_signs=direction_signs,
            direction_start_positions=snapshot["joints"],
            direction_constraint_margin=(
                self._direction_constraint_margin
            ),
        )
        plan_result, error = self._core.plan_target(
            plan_goal, self._planning_action_timeout, self._cancel_requested
        )
        plan_code = 0 if plan_result is None else int(plan_result.error_code.val)
        self._event("cartesian_goal_plan_result", success=not bool(error),
                    moveit_error_code=plan_code, message=error)
        if error:
            if self._cancel_requested():
                self._preempt_result(result, error)
            else:
                result.final_stage = Stage.FAILED
                result.moveit_error_code = plan_code
                result.message = error
                self._finish(result)
            return
        result.moveit_error_code = plan_code

        self._feedback(Stage.AUDITING_TRAJECTORY, "auditing the four-joint trajectory")
        safe, audit_message = audit_trajectory(
            plan_result.planned_trajectory,
            self._joint_position_bounds,
            self._max_trajectory_velocity,
            self._max_trajectory_acceleration,
            self._max_trajectory_point_delta,
            self._max_trajectory_duration,
            self._max_trajectory_points,
        )
        if safe and direction_signs:
            audit_error = direction_error(
                plan_result.planned_trajectory,
                snapshot["joints"],
                direction_signs,
                self._direction_tolerance,
            )
            if audit_error:
                safe = False
                audit_message = "指定方向轨迹约束失败：{}".format(audit_error)
            else:
                audit_message += "; 指定方向单调检查通过：{}".format(
                    ",".join(
                        "{}:{:+.0f}".format(name, sign)
                        for name, sign in direction_signs.items()
                    )
                )
        result.planned_goal_joint_state = planned_goal_joint_state(
            plan_result.planned_trajectory
        )
        result.large_flip_override_expected = large_flip_override_expected(
            snapshot["anchor"], plan_result.planned_trajectory,
            snapshot["joints"], self._large_flip_threshold,
        )
        self._event(
            "cartesian_goal_audit_result",
            anchor=snapshot["anchor"],
            success=bool(safe), message=audit_message,
            planned_joint_names=list(result.planned_goal_joint_state.name),
            planned_joint_positions=list(result.planned_goal_joint_state.position),
            large_flip_override_expected=bool(result.large_flip_override_expected),
        )
        if not safe:
            result.final_stage = Stage.REJECTED
            result.message = audit_message
            self._finish(result)
            return

        if execute_after_plan:
            self._execute_planned_trajectory(
                result,
                plan_result.planned_trajectory,
                snapshot,
                direction_signs,
                audit_message,
            )
            return

        result.plan_id = self._store_cached_plan(
            plan_result.planned_trajectory, target, result, snapshot, audit_message,
            direction_signs,
        )
        result.success = True
        result.final_stage = Stage.SUCCEEDED
        result.message = (
            "PLAN_READY: plan_id={} valid_for={:.1f}s; {}"
        ).format(result.plan_id, self._cached_plan_timeout, audit_message)
        self._feedback(Stage.SUCCEEDED, result.message)
        self._finish(result, "succeeded")

    def _execute_planned_trajectory(
            self, result, trajectory, planning_snapshot,
            direction_signs, audit_message):
        self._feedback(
            Stage.AUDITING_TRAJECTORY,
            "revalidating live state before direct trajectory dispatch",
        )
        snapshot, error = self._common_state_snapshot()
        if not error:
            error = self._execution_state_error(snapshot)
        if error:
            result.final_stage = Stage.REJECTED
            result.message = "direct execution gate rejected: {}".format(error)
            self._finish(result)
            return
        if snapshot["anchor"] != planning_snapshot["anchor"]:
            result.final_stage = Stage.REJECTED
            result.message = "runtime_anchor changed during direct planning"
            self._finish(result)
            return
        if snapshot["planner"].session_id != planning_snapshot["planner"].session_id:
            result.final_stage = Stage.REJECTED
            result.message = "PlannerControl session changed during direct planning"
            self._finish(result)
            return

        start_reference = {"start_joints": planning_snapshot["joints"]}
        error = self._cached_start_state_error(start_reference, snapshot["joints"])
        if error:
            result.final_stage = Stage.REJECTED
            result.message = "direct plan start rejected: {}".format(error)
            self._finish(result)
            return
        if direction_signs:
            error = direction_error(
                trajectory, snapshot["joints"], direction_signs,
                self._direction_tolerance,
            )
            if error:
                result.final_stage = Stage.REJECTED
                result.message = "direct commissioned direction rejected: {}".format(error)
                self._finish(result)
                return

        result.large_flip_override_expected = large_flip_override_expected(
            snapshot["anchor"], trajectory, snapshot["joints"],
            self._large_flip_threshold,
        )
        self._event(
            "cartesian_goal_direct_plan_revalidated",
            session_id=int(snapshot["planner"].session_id),
            audit_message=audit_message,
            large_flip_override_expected=bool(
                result.large_flip_override_expected
            ),
        )
        if trajectory_goal_within_deadband(
                trajectory, snapshot["joints"], self._goal_joint_deadband):
            result.success = True
            result.final_stage = Stage.SUCCEEDED
            result.message = (
                "ALREADY_AT_GOAL: final joints are within {:.3f} rad; no trajectory sent"
            ).format(self._goal_joint_deadband)
            self._finish(result, "succeeded")
            return

        if result.large_flip_override_expected:
            support_second = {
                "left": "left_second_leg_joint",
                "right": "right_second_leg_joint",
            }[snapshot["anchor"]]
            direction_name = (
                "counter-clockwise"
                if commissioned_large_flip_direction(snapshot["anchor"]) > 0.0
                else "clockwise"
            )
            self._feedback(
                Stage.AUDITING_TRAJECTORY,
                "commissioned {} {} large-flip completion fallback is "
                "expected; the endpoint must be verified from encoder/TF "
                "evidence".format(support_second, direction_name),
            )
        self._dispatch_trajectory(result, trajectory, snapshot, "direct")

    def _execute_cached_plan(self, plan_id):
        result = self._base_result(Stage.WAITING_STATE, "cached execution requested")
        result.plan_id = plan_id
        cached, error = self._load_cached_plan(plan_id)
        if error:
            result.final_stage = Stage.REJECTED
            result.message = "cached plan rejected: {}".format(error)
            self._finish(result)
            return

        with self._lock:
            self._accepted_anchor = cached["anchor"]
        result.accepted_anchor_side = cached["anchor"]
        result.accepted_tip_link = cached["tip_link"]
        result.target_in_world = copy.deepcopy(cached["target"])
        result.planned_goal_joint_state = copy.deepcopy(
            cached["planned_goal_joint_state"]
        )
        result.large_flip_override_expected = bool(
            cached["large_flip_override_expected"]
        )
        result.moveit_error_code = int(cached["moveit_error_code"])

        self._feedback(
            Stage.WAITING_STATE,
            "revalidating cached plan {}".format(plan_id),
        )
        snapshot, error = self._common_state_snapshot()
        if not error:
            error = self._execution_state_error(snapshot)
        if error:
            result.final_stage = Stage.REJECTED
            result.message = "execution gate rejected: {}".format(error)
            self._finish(result)
            return
        if snapshot["anchor"] != cached["anchor"]:
            result.final_stage = Stage.REJECTED
            result.message = "runtime_anchor changed since planning"
            self._finish(result)
            return
        error = self._cached_start_state_error(cached, snapshot["joints"])
        if error:
            result.final_stage = Stage.REJECTED
            result.message = "cached plan start rejected: {}".format(error)
            self._finish(result)
            return

        self._feedback(Stage.AUDITING_TRAJECTORY, "re-auditing the cached trajectory")
        safe, audit_message = audit_trajectory(
            cached["trajectory"],
            self._joint_position_bounds,
            self._max_trajectory_velocity,
            self._max_trajectory_acceleration,
            self._max_trajectory_point_delta,
            self._max_trajectory_duration,
            self._max_trajectory_points,
        )
        if not safe:
            self._invalidate_cached_plan("cached trajectory failed re-audit")
            result.final_stage = Stage.REJECTED
            result.message = audit_message
            self._finish(result)
            return
        direction_signs = dict(cached.get("direction_signs", {}))
        if direction_signs:
            audit_error = direction_error(
                cached["trajectory"], snapshot["joints"],
                direction_signs,
                self._direction_tolerance,
            )
            if audit_error:
                self._invalidate_cached_plan(
                    "cached commissioned direction check failed"
                )
                result.final_stage = Stage.REJECTED
                result.message = (
                    "cached plan commissioned direction rejected: {}"
                ).format(audit_error)
                self._finish(result)
                return
        previous_flip_prediction = result.large_flip_override_expected
        result.large_flip_override_expected = large_flip_override_expected(
            snapshot["anchor"], cached["trajectory"],
            snapshot["joints"], self._large_flip_threshold,
        )
        self._event(
            "cartesian_goal_cached_plan_revalidated",
            plan_id=plan_id, age=float(cached["age"]),
            audit_message=audit_message,
            large_flip_override_expected=bool(
                result.large_flip_override_expected
            ),
        )
        if result.large_flip_override_expected != previous_flip_prediction:
            self._event(
                "cartesian_goal_large_flip_prediction_updated",
                plan_id=plan_id,
                large_flip_override_expected=bool(
                    result.large_flip_override_expected
                ),
                reason="refreshed encoder state before cached execution",
            )
        if trajectory_goal_within_deadband(
                cached["trajectory"], snapshot["joints"],
                self._goal_joint_deadband):
            self._consume_cached_plan(plan_id)
            result.success = True
            result.final_stage = Stage.SUCCEEDED
            result.message = (
                "ALREADY_AT_GOAL: final joints are within {:.3f} rad; no trajectory sent"
            ).format(self._goal_joint_deadband)
            self._finish(result, "succeeded")
            return

        if result.large_flip_override_expected:
            support_second = {
                "left": "left_second_leg_joint",
                "right": "right_second_leg_joint",
            }[snapshot["anchor"]]
            direction_name = (
                "counter-clockwise"
                if commissioned_large_flip_direction(snapshot["anchor"]) > 0.0
                else "clockwise"
            )
            self._feedback(
                Stage.AUDITING_TRAJECTORY,
                "commissioned {} {} large-flip completion fallback is "
                "expected; the endpoint must be verified from encoder/TF "
                "evidence".format(support_second, direction_name),
            )
        if not self._consume_cached_plan(plan_id):
            result.final_stage = Stage.REJECTED
            result.message = "cached plan was invalidated before dispatch"
            self._finish(result)
            return
        self._dispatch_trajectory(
            result, cached["trajectory"], snapshot, plan_id
        )

    def _dispatch_trajectory(self, result, trajectory, snapshot, plan_reference):
        initial_session_id = snapshot["planner"].session_id
        with self._lock:
            self._execution_session_id = initial_session_id
        self._feedback(Stage.EXECUTING, "sending audited trajectory")
        self._event(
            "cartesian_goal_execute_requested",
            plan_reference=plan_reference,
            session_id=int(initial_session_id),
            large_flip_override_expected=bool(result.large_flip_override_expected),
        )
        execute_result, error = self._core.execute_trajectory(
            trajectory,
            self._execution_action_timeout,
            self._cancel_requested,
        )
        execute_code = 0 if execute_result is None else int(execute_result.error_code.val)
        if error:
            result.moveit_error_code = execute_code
            if self._cancel_requested():
                self._preempt_result(result, error)
            else:
                result.final_stage = Stage.FAILED
                result.message = error
                self._finish(result)
            return

        self._feedback(
            Stage.WAITING_SESSION_REARM,
            "waiting for SMC PlannerControl completion confirmation",
        )
        deadline = time.monotonic() + self._session_rearm_timeout
        completion_confirmed = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            reason = self._cancel_requested()
            if reason:
                self._preempt_result(result, reason)
                return
            with self._lock:
                planner = copy.deepcopy(self._planner_state)
            if planner is not None:
                normally_completed = (
                    planner.last_completed_session_id == initial_session_id and
                    planner.exit_reason == PlannerControlState.EXIT_COMPLETED
                )
                legacy_rearmed = (
                    planner.session_id != initial_session_id and
                    planner.active and planner.accepting_commands and
                    not planner.has_accepted_command
                )
                if normally_completed or legacy_rearmed:
                    completion_confirmed = True
                    break
            rospy.sleep(0.05)
        if not completion_confirmed:
            result.final_stage = Stage.FAILED
            result.message = (
                "trajectory execution succeeded, but SMC completion was not confirmed within "
                "{:.1f}s; do not resend without checking live state"
            ).format(self._session_rearm_timeout)
            self._finish(result)
            return
        result.success = True
        result.final_stage = Stage.SUCCEEDED
        result.moveit_error_code = execute_code
        result.message = (
            "EXECUTION_SUCCEEDED: audited trajectory completed and SMC confirmed "
            "normal PlannerControl exit; confirm the encoder and TF evidence"
        )
        self._feedback(Stage.SUCCEEDED, result.message)
        self._finish(result, "succeeded")


def main():
    rospy.init_node("gp11_cartesian_goal_server")
    CartesianGoalServer()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
