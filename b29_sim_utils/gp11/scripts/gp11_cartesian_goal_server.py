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
    finite_point,
    inside_workspace,
    joint_error,
    large_flip_override_expected,
    planned_goal_joint_state,
    planner_control_cancel_reason,
    positive_direction_error,
    positive_direction_seed_positions,
    positive_large_flip_joints,
    reach_joint_positions_from_state,
    tip_link_for_anchor,
    trajectory_goal_within_deadband,
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
        self._require_positive_large_flip_direction = bool(rospy.get_param(
            "~require_positive_large_flip_direction", True
        ))
        self._positive_direction_tolerance = _positive_param(
            "~positive_direction_tolerance",
            DEFAULT_POSITIVE_DIRECTION_TOLERANCE,
        )
        self._positive_direction_constraint_margin = _positive_param(
            "~positive_direction_constraint_margin",
            DEFAULT_POSITIVE_DIRECTION_TOLERANCE,
        )
        self._positive_ik_seed_fractions = tuple(float(value) for value in rospy.get_param(
            "~positive_ik_seed_fractions", [0.50, 0.80, 0.95]
        ))
        if (not self._positive_ik_seed_fractions or
                any(not math.isfinite(value) or not 0.0 < value < 1.0
                    for value in self._positive_ik_seed_fractions)):
            raise ValueError("positive_ik_seed_fractions must contain values in (0, 1)")
        self._joint_state_timeout = _positive_param("~joint_state_timeout", 0.5)
        self._control_state_timeout = _positive_param("~control_state_timeout", 0.5)
        self._target_max_age = _positive_param("~target_max_age", 1.0)
        self._tf_timeout = _positive_param("~tf_timeout", 0.5)
        self._session_rearm_timeout = _positive_param("~session_rearm_timeout", 3.0)
        self._cached_plan_timeout = _positive_param("~cached_plan_timeout", 120.0)
        self._cached_plan_start_tolerance = _positive_param(
            "~cached_plan_start_tolerance", math.radians(25.0)
        )

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
            positive_direction_joints=()):
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
            "positive_direction_joints": tuple(positive_direction_joints),
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

    def _positive_branch_ik(
            self, initial_response, initial_positions, target, tip_link,
            snapshot):
        """Retry collision-checked IK with seeds on the required positive arc."""
        required = positive_large_flip_joints(
            snapshot["anchor"], initial_positions, snapshot["joints"],
            self._large_flip_threshold,
        ) if self._require_positive_large_flip_direction else ()
        if not required:
            return initial_response, initial_positions, required, ""

        def candidate_error(positions):
            for joint_name in required:
                delta = positions[joint_name] - snapshot["joints"][joint_name]
                if delta <= self._large_flip_threshold:
                    return (
                        "{} IK delta is {:+.6f} rad; a positive large-flip "
                        "branch above {:+.6f} rad is required"
                    ).format(joint_name, delta, self._large_flip_threshold)
            return ""

        error = candidate_error(initial_positions)
        if not error:
            return initial_response, initial_positions, required, ""

        rospy.logwarn(
            "Initial IK selected a negative large-flip branch (%s); "
            "retrying with positive-arc seeds", error,
        )
        for attempt, fraction in enumerate(self._positive_ik_seed_fractions, 1):
            seed = positive_direction_seed_positions(
                snapshot["joints"], initial_positions, required, fraction
            )
            response, ik_error = self._core.check_ik(
                target, tip_link, self._group_name, seed, self._ik_timeout,
            )
            positions = None
            if not ik_error:
                try:
                    positions = reach_joint_positions_from_state(response.solution)
                except ValueError as exc:
                    ik_error = "IK result rejected: {}".format(exc)
            direction_error = "" if positions is None else candidate_error(positions)
            self._event(
                "cartesian_goal_positive_ik_attempt",
                attempt=attempt,
                seed_fraction=fraction,
                success=not bool(ik_error or direction_error),
                message=ik_error or direction_error,
                joint_positions=(
                    [] if positions is None else
                    [positions[name] for name in REACH_JOINTS]
                ),
            )
            if not ik_error and not direction_error:
                rospy.loginfo(
                    "Positive large-flip IK branch selected on attempt %d", attempt
                )
                return response, positions, required, ""
        return None, None, required, (
            "无法找到满足正向约束的IK解：{}；未生成或缓存轨迹".format(error)
        )

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
        with self._lock:
            self._accepted_anchor = ""
            self._operation_execute = bool(goal.execute and approved_plan_id)
            self._execution_session_id = 0
            self._cancel_reason = ""
        self._event(
            "cartesian_goal_received",
            frame_id=goal.target.header.frame_id,
            point=[goal.target.point.x, goal.target.point.y, goal.target.point.z],
            execute=bool(goal.execute),
            approved_plan_id=approved_plan_id,
            position_tolerance=float(goal.position_tolerance),
        )
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
            result = self._base_result(Stage.REJECTED, "direct execution disabled")
            result.message = (
                "direct point execution is disabled; plan first, then submit its plan_id"
            )
            self._finish(result)
            return
        self._plan_goal(goal)

    def _plan_goal(self, goal):
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
        ik_response, ik_goal_positions, positive_direction_joints, error = (
            self._positive_branch_ik(
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
        self._event(
            "cartesian_goal_ik_solution",
            joint_names=list(REACH_JOINTS),
            joint_positions=[ik_goal_positions[name] for name in REACH_JOINTS],
            positive_direction_joints=list(positive_direction_joints),
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
            positive_direction_joints=positive_direction_joints,
            positive_direction_start_positions=snapshot["joints"],
            positive_direction_constraint_margin=(
                self._positive_direction_constraint_margin
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
        if safe and positive_direction_joints:
            direction_error = positive_direction_error(
                plan_result.planned_trajectory,
                snapshot["joints"],
                positive_direction_joints,
                self._positive_direction_tolerance,
            )
            if direction_error:
                safe = False
                audit_message = "正向轨迹约束失败：{}".format(direction_error)
            else:
                audit_message += "; left_second正向单调检查通过"
        result.planned_goal_joint_state = planned_goal_joint_state(
            plan_result.planned_trajectory
        )
        result.large_flip_override_expected = large_flip_override_expected(
            snapshot["anchor"], plan_result.planned_trajectory,
            snapshot["joints"], self._large_flip_threshold,
        )
        self._event(
            "cartesian_goal_audit_result",
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

        result.plan_id = self._store_cached_plan(
            plan_result.planned_trajectory, target, result, snapshot, audit_message,
            positive_direction_joints,
        )
        result.success = True
        result.final_stage = Stage.SUCCEEDED
        result.message = (
            "PLAN_READY: plan_id={} valid_for={:.1f}s; {}"
        ).format(result.plan_id, self._cached_plan_timeout, audit_message)
        self._feedback(Stage.SUCCEEDED, result.message)
        self._finish(result, "succeeded")

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
        positive_direction_joints = tuple(
            cached.get("positive_direction_joints", ())
        )
        if positive_direction_joints:
            direction_error = positive_direction_error(
                cached["trajectory"], snapshot["joints"],
                positive_direction_joints,
                self._positive_direction_tolerance,
            )
            if direction_error:
                self._invalidate_cached_plan(
                    "cached positive direction check failed"
                )
                result.final_stage = Stage.REJECTED
                result.message = (
                    "cached plan positive direction rejected: {}"
                ).format(direction_error)
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
            self._feedback(
                Stage.AUDITING_TRAJECTORY,
                "large right_second adapter override is expected; spatial endpoint "
                "must be verified from encoder/TF evidence",
            )
        if not self._consume_cached_plan(plan_id):
            result.final_stage = Stage.REJECTED
            result.message = "cached plan was invalidated before dispatch"
            self._finish(result)
            return
        self._dispatch_cached_trajectory(
            result, cached["trajectory"], snapshot, plan_id
        )

    def _dispatch_cached_trajectory(self, result, trajectory, snapshot, plan_id):
        initial_session_id = snapshot["planner"].session_id
        with self._lock:
            self._execution_session_id = initial_session_id
        self._feedback(Stage.EXECUTING, "sending approved cached trajectory")
        self._event(
            "cartesian_goal_execute_requested",
            plan_id=plan_id,
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

        self._feedback(Stage.WAITING_SESSION_REARM, "waiting for a fresh PlannerControl session")
        deadline = time.monotonic() + self._session_rearm_timeout
        rearmed = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            reason = self._cancel_requested()
            if reason:
                self._preempt_result(result, reason)
                return
            with self._lock:
                planner = copy.deepcopy(self._planner_state)
            if (planner is not None and planner.session_id != initial_session_id and
                    planner.active and planner.accepting_commands and
                    not planner.has_accepted_command):
                rearmed = True
                break
            rospy.sleep(0.05)
        if not rearmed:
            result.final_stage = Stage.FAILED
            result.message = (
                "trajectory execution succeeded, but PlannerControl did not rearm within "
                "{:.1f}s; do not resend without checking live state"
            ).format(self._session_rearm_timeout)
            self._finish(result)
            return
        result.success = True
        result.final_stage = Stage.SUCCEEDED
        result.moveit_error_code = execute_code
        result.message = (
            "EXECUTION_SUCCEEDED: approved cached plan completed and PlannerControl "
            "rearmed; confirm the encoder and TF evidence"
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
