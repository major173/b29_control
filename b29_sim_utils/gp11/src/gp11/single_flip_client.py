"""Shared client for one commissioned pass of the two-anchor MoveIt flip."""

import math
import threading
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from b29_smc_auto_controller.msg import AutoStateTrace, PlannerControlState
from gp11.msg import ReachPointAction, ReachPointGoal


DEFAULT_TARGET_FRAME = "world"
DEFAULT_TARGET_X = 0.0060528
DEFAULT_TARGET_Y = -0.00000272
DEFAULT_TARGET_Z = -0.7399961
DEFAULT_REVERSE_TARGET_X = 0.1700000
DEFAULT_REVERSE_TARGET_Z = 0.7399961
REACH_JOINTS = (
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
)
POSE_JOINTS_BY_CROSSING_SIDE = {
    "Right": (
        "left_first_leg_joint",
        "left_second_leg_joint",
        "right_first_leg_joint",
    ),
    "Left": (
        "right_first_leg_joint",
        "right_second_leg_joint",
        "left_first_leg_joint",
    ),
}
ANCHOR_BY_CROSSING_SIDE = {
    "Right": "left",
    "Left": "right",
}
TARGET_PROFILE_BY_FIRST_CROSSING_SIDE = {
    "Right": "default_forward",
    "Left": "reverse",
}


def target_profile_for_first_crossing_side(first_crossing_side):
    """Return the commissioned travel profile selected by the SMC direction code."""
    try:
        return TARGET_PROFILE_BY_FIRST_CROSSING_SIDE[first_crossing_side]
    except KeyError:
        raise ValueError(
            "Unsupported first crossing side: {}".format(
                first_crossing_side
            )
        )


def obstacle_cycle_complete_wait_matches(trace, first_crossing_side):
    """Return whether both passes reached the normal obstacle-clear wait."""
    return (
        trace is not None and
        trace.obstacle_crossing_stage == "CompleteWaitObstacleClear" and
        trace.obstacle_crossing_side == "None" and
        trace.first_crossing_side == first_crossing_side and
        trace.obstacle_crossing_transition_reason == "both_sides_completed"
    )


def obstacle_cycle_idle_reset_matches(trace):
    """Return whether SMC consumed the obstacle falling edge and reset."""
    return (
        trace is not None and
        not trace.obstacle_crossing_active and
        trace.obstacle_crossing_stage == "Idle" and
        trace.obstacle_crossing_side == "None" and
        trace.obstacle_crossing_transition_reason ==
        "obstacle_trigger_falling_edge"
    )


class SingleFlipClient(object):
    def __init__(self):
        self._condition = threading.Condition()
        self._trace = None
        self._planner = None
        self._runtime_anchor = ""
        self._runtime_status = ""
        self._joint_samples = []
        self._disconnect_done_monotonic = None
        self._adapter_failure_detail = ""
        self._last_failure = ""

        controller_namespace = str(rospy.get_param(
            "~smc_controller_namespace",
            "/b29_controller/b29_smc_auto_controller",
        )).rstrip("/")
        self._action_name = rospy.get_param(
            "~action_name", "/gp11_moveit/reach_point"
        )
        self._target_frame = rospy.get_param(
            "~target_frame", DEFAULT_TARGET_FRAME
        )
        self._target_x = float(rospy.get_param("~target_x", DEFAULT_TARGET_X))
        self._target_y = float(rospy.get_param("~target_y", DEFAULT_TARGET_Y))
        self._target_z = float(rospy.get_param("~target_z", DEFAULT_TARGET_Z))
        self._reverse_target_frame = rospy.get_param(
            "~reverse_target_frame", self._target_frame
        )
        self._reverse_target_x = float(rospy.get_param(
            "~reverse_target_x", DEFAULT_REVERSE_TARGET_X
        ))
        self._reverse_target_y = float(rospy.get_param(
            "~reverse_target_y", self._target_y
        ))
        self._reverse_target_z = float(rospy.get_param(
            "~reverse_target_z", DEFAULT_REVERSE_TARGET_Z
        ))
        self._targets_by_first_crossing_side = {
            "Right": (
                self._target_frame,
                self._target_x,
                self._target_y,
                self._target_z,
            ),
            "Left": (
                self._reverse_target_frame,
                self._reverse_target_x,
                self._reverse_target_y,
                self._reverse_target_z,
            ),
        }
        self._position_tolerance = float(rospy.get_param(
            "~position_tolerance", 0.008
        ))
        self._stability_timeout = max(0.1, float(rospy.get_param(
            "~post_disconnect_stability_timeout", 20.0
        )))
        self._stability_duration = max(0.1, float(rospy.get_param(
            "~post_disconnect_stability_duration", 1.0
        )))
        self._stability_position_span = max(0.0, float(rospy.get_param(
            "~post_disconnect_position_span_threshold", 0.01
        )))
        self._stability_velocity = max(0.0, float(rospy.get_param(
            "~post_disconnect_velocity_threshold", 0.02
        )))
        self._joint_state_max_age = max(0.05, float(rospy.get_param(
            "~post_disconnect_joint_state_max_age", 0.5
        )))
        if any(not math.isfinite(value) for value in (
                self._target_x, self._target_y, self._target_z,
                self._reverse_target_x, self._reverse_target_y,
                self._reverse_target_z,
                self._stability_timeout, self._stability_duration,
                self._stability_position_span, self._stability_velocity,
                self._joint_state_max_age)):
            raise ValueError("flip target and stability parameters must be finite")

        self._trace_sub = rospy.Subscriber(
            controller_namespace + "/state_trace",
            AutoStateTrace, self._trace_cb, queue_size=10,
        )
        self._planner_sub = rospy.Subscriber(
            controller_namespace + "/planner_control_state",
            PlannerControlState, self._planner_cb, queue_size=10,
        )
        self._anchor_sub = rospy.Subscriber(
            "/gp11_moveit/runtime_anchor",
            String, self._anchor_cb, queue_size=5,
        )
        self._runtime_status_sub = rospy.Subscriber(
            "/gp11_moveit/runtime_status",
            String, self._runtime_status_cb, queue_size=5,
        )
        self._joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joint_state_cb, queue_size=20,
        )
        self._adapter_result_sub = rospy.Subscriber(
            rospy.get_param(
                "~adapter_result_topic",
                "/gp11_moveit/reach_arm_controller/follow_joint_trajectory/result",
            ),
            FollowJointTrajectoryActionResult,
            self._adapter_result_cb,
            queue_size=5,
        )
        self._start_flip_pub = rospy.Publisher(
            controller_namespace + "/start_flip", Empty, queue_size=1,
        )
        self._action = actionlib.SimpleActionClient(
            self._action_name, ReachPointAction
        )

    def _trace_cb(self, message):
        with self._condition:
            previous_stage = (
                "" if self._trace is None else
                self._trace.obstacle_crossing_stage
            )
            self._trace = message
            if (message.obstacle_crossing_stage == "DisconnectDoneWaitFlip" and
                    previous_stage != "DisconnectDoneWaitFlip"):
                self._disconnect_done_monotonic = time.monotonic()
            self._condition.notify_all()

    def _planner_cb(self, message):
        with self._condition:
            self._planner = message
            self._condition.notify_all()

    def _anchor_cb(self, message):
        with self._condition:
            self._runtime_anchor = message.data.strip().lower()
            self._condition.notify_all()

    def _runtime_status_cb(self, message):
        with self._condition:
            self._runtime_status = message.data.strip()
            self._condition.notify_all()

    def _joint_state_cb(self, message):
        indices = {name: index for index, name in enumerate(message.name)}
        if any(name not in indices for name in REACH_JOINTS):
            return
        positions = {}
        velocities = {}
        for name in REACH_JOINTS:
            index = indices[name]
            if index >= len(message.position):
                return
            position = float(message.position[index])
            if not math.isfinite(position):
                return
            if index >= len(message.velocity):
                return
            velocity = float(message.velocity[index])
            if not math.isfinite(velocity):
                return
            positions[name] = position
            velocities[name] = velocity
        now = time.monotonic()
        with self._condition:
            self._joint_samples.append((now, positions, velocities))
            cutoff = now - max(2.0, self._stability_duration + 1.0)
            self._joint_samples = [
                sample for sample in self._joint_samples if sample[0] >= cutoff
            ]
            self._condition.notify_all()

    def _adapter_result_cb(self, message):
        if message.result.error_code == 0:
            return
        detail = message.result.error_string.strip()
        if not detail:
            detail = "adapter status={} code={}".format(
                message.status.status, message.result.error_code
            )
        with self._condition:
            self._adapter_failure_detail = detail
            self._condition.notify_all()

    @property
    def last_failure(self):
        with self._condition:
            return self._last_failure

    def _fail(self, code, message, *args):
        detail = message % args if args else message
        with self._condition:
            if self._last_failure and self._last_failure != detail:
                detail = "{}; root_cause={}".format(detail, self._last_failure)
            self._last_failure = detail
        rospy.logerr("%s", detail)
        return code

    def _controller_failure_locked(self):
        """Return an already-latched lower-level failure, if one is visible."""
        if self._trace is None:
            return ""
        trace = self._trace
        if trace.software_emergency_stop_latched:
            return "SMC software emergency stop is latched"
        if not trace.obstacle_crossing_active:
            return ""
        current_state = trace.current_state.strip().lower()
        if current_state in ("commsloss", "safestop"):
            return "SMC entered {}".format(trace.current_state)
        if not trace.lower_alive:
            return "lower_alive became false during obstacle crossing"
        if trace.joint_fault or trace.grip_fault:
            return "joint_fault or grip_fault became active during obstacle crossing"
        if trace.obstacle_crossing_stage == "ManualIntervention":
            detail = trace.last_failure_reason.strip()
            if not detail:
                detail = trace.manual_intervention_reason.strip()
            return "SMC entered ManualIntervention: {}".format(
                detail or "failure reason unavailable"
            )
        return ""

    def _cycle_reset_failure_locked(self):
        """Return safety failures while finishing the current obstacle cycle."""
        if self._trace is None:
            return ""
        trace = self._trace
        if trace.software_emergency_stop_latched:
            return "SMC software emergency stop is latched"
        current_state = trace.current_state.strip().lower()
        if current_state in ("commsloss", "safestop"):
            return "SMC entered {}".format(trace.current_state)
        if not trace.lower_alive:
            return "lower_alive became false while resetting obstacle cycle"
        if trace.joint_fault or trace.grip_fault:
            return "joint_fault or grip_fault became active while resetting obstacle cycle"
        if trace.obstacle_crossing_stage == "ManualIntervention":
            detail = trace.last_failure_reason.strip()
            if not detail:
                detail = trace.manual_intervention_reason.strip()
            return "SMC entered ManualIntervention: {}".format(
                detail or "failure reason unavailable"
            )
        return ""

    def _post_disconnect_stable_locked(self, now, pose_joints):
        if self._disconnect_done_monotonic is None:
            return False
        window_start = now - self._stability_duration
        if window_start < self._disconnect_done_monotonic:
            return False
        samples = [
            sample for sample in self._joint_samples
            if sample[0] >= window_start
        ]
        if (not samples or now - samples[-1][0] > self._joint_state_max_age or
                samples[0][0] > window_start + 0.1):
            return False
        for joint_name in pose_joints:
            positions = [sample[1][joint_name] for sample in samples]
            if max(positions) - min(positions) > self._stability_position_span:
                return False
            if any(abs(sample[2][joint_name]) > self._stability_velocity
                   for sample in samples):
                return False
        return True

    def _wait_for_post_disconnect_stability(self, crossing_side):
        pose_joints = POSE_JOINTS_BY_CROSSING_SIDE[crossing_side]
        deadline = time.monotonic() + self._stability_timeout
        with self._condition:
            while not rospy.is_shutdown():
                failure = self._controller_failure_locked()
                if failure:
                    self._last_failure = failure
                    return False
                if (self._trace is None or
                        self._trace.obstacle_crossing_side != crossing_side or
                        self._trace.obstacle_crossing_stage != "DisconnectDoneWaitFlip"):
                    return False
                now = time.monotonic()
                if self._post_disconnect_stable_locked(now, pose_joints):
                    latest = self._joint_samples[-1]
                    rospy.loginfo(
                        "Post-disconnect %s-side pose encoders stable for %.2f s: positions=%s velocities=%s",
                        crossing_side,
                        self._stability_duration,
                        [round(latest[1][name], 6) for name in pose_joints],
                        [round(latest[2][name], 6) for name in pose_joints],
                    )
                    return True
                remaining = deadline - now
                if remaining <= 0.0:
                    return False
                self._condition.wait(min(0.1, remaining))
        return False

    def _wait_for(self, predicate, timeout, abort_predicate=None):
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while not rospy.is_shutdown():
                if abort_predicate is not None:
                    failure = abort_predicate()
                    if failure:
                        self._last_failure = failure
                        return False
                if predicate():
                    return True
                if deadline is None:
                    self._condition.wait(0.1)
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(min(0.1, remaining))
        return False

    def _wait_for_action_server(self, timeout):
        if timeout is not None:
            return self._action.wait_for_server(rospy.Duration(timeout))
        while not rospy.is_shutdown():
            if self._action.wait_for_server(rospy.Duration(1.0)):
                return True
        return False

    def _stage(self):
        return "unknown" if self._trace is None else self._trace.obstacle_crossing_stage

    def wait_for_disconnect_side(self, timeout=None, expected_side=None):
        """Return the side currently latched by SMC once disconnect is complete."""
        if expected_side is not None and expected_side not in ANCHOR_BY_CROSSING_SIDE:
            raise ValueError("Unsupported expected crossing side: {}".format(expected_side))

        ready = self._wait_for(
            lambda: (
                self._trace is not None and
                self._trace.obstacle_crossing_stage == "DisconnectDoneWaitFlip" and
                self._trace.obstacle_crossing_side in ANCHOR_BY_CROSSING_SIDE and
                (
                    expected_side is None or
                    self._trace.obstacle_crossing_side == expected_side
                )
            ),
            timeout,
            abort_predicate=self._controller_failure_locked,
        )
        if not ready:
            return None
        with self._condition:
            return self._trace.obstacle_crossing_side

    def wait_for_obstacle_cycle_reset(self, first_crossing_side,
                                      final_crossing_side, timeout=None):
        """Wait for normal regrip, obstacle clear, and SMC reset after pass two.

        This is deliberately a two-phase observation.  An Idle trace already
        present when the node starts must never be mistaken for completion of
        the obstacle whose two MoveIt passes just handed off.
        """
        if first_crossing_side not in ANCHOR_BY_CROSSING_SIDE:
            raise ValueError(
                "Unsupported first crossing side: {}".format(
                    first_crossing_side
                )
            )
        expected_final_side = {
            "Left": "Right",
            "Right": "Left",
        }[first_crossing_side]
        if final_crossing_side != expected_final_side:
            raise ValueError(
                "Final crossing side {} is not opposite first side {}".format(
                    final_crossing_side, first_crossing_side
                )
            )

        deadline = None if timeout is None else time.monotonic() + timeout
        complete_wait_seen = False
        with self._condition:
            self._last_failure = ""
            while not rospy.is_shutdown():
                failure = self._cycle_reset_failure_locked()
                if failure:
                    self._last_failure = failure
                    rospy.logerr("%s", failure)
                    return False

                if not complete_wait_seen:
                    if obstacle_cycle_complete_wait_matches(
                            self._trace, first_crossing_side):
                        complete_wait_seen = True
                        rospy.loginfo(
                            "Obstacle cycle reached CompleteWaitObstacleClear; "
                            "waiting for the obstacle falling edge before "
                            "arming the next cycle"
                        )
                    elif (self._trace is not None and
                          self._trace.obstacle_crossing_stage in (
                              "CompleteWaitObstacleClear", "Idle"
                          )):
                        self._last_failure = (
                            "SMC reached {} before the normal two-pass cycle "
                            "completion marker; transition_reason={}"
                        ).format(
                            self._trace.obstacle_crossing_stage,
                            self._trace.obstacle_crossing_transition_reason or
                            "none",
                        )
                        rospy.logerr("%s", self._last_failure)
                        return False
                elif obstacle_cycle_idle_reset_matches(self._trace):
                    rospy.loginfo(
                        "Obstacle falling edge consumed: SMC is Idle with no "
                        "latched crossing side"
                    )
                    return True
                elif (self._trace is not None and
                      self._trace.obstacle_crossing_stage == "Idle"):
                    self._last_failure = (
                        "SMC returned to Idle without the commissioned "
                        "obstacle falling-edge reset; transition_reason={}"
                    ).format(
                        self._trace.obstacle_crossing_transition_reason or
                        "none"
                    )
                    rospy.logerr("%s", self._last_failure)
                    return False

                if deadline is None:
                    self._condition.wait(0.1)
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    phase = (
                        "Idle reset after obstacle falling edge"
                        if complete_wait_seen else
                        "CompleteWaitObstacleClear after the second handoff"
                    )
                    self._last_failure = (
                        "Timed out waiting for {}".format(phase)
                    )
                    rospy.logerr("%s", self._last_failure)
                    return False
                self._condition.wait(min(0.1, remaining))
        return False

    def run(self, startup_timeout=30.0, state_timeout=5.0,
            execution_timeout=150.0, handoff_timeout=5.0,
            crossing_side="Right", first_crossing_side=None,
            pass_index=1, pass_count=1):
        if crossing_side not in ANCHOR_BY_CROSSING_SIDE:
            return self._fail(
                1, "Unsupported commissioned crossing side: %s", crossing_side
            )
        if first_crossing_side is None:
            first_crossing_side = crossing_side
        try:
            target_profile = target_profile_for_first_crossing_side(
                first_crossing_side
            )
            target_frame, target_x, target_y, target_z = (
                self._targets_by_first_crossing_side[first_crossing_side]
            )
        except ValueError as error:
            return self._fail(1, "%s", error)
        anchor_side = ANCHOR_BY_CROSSING_SIDE[crossing_side]
        with self._condition:
            self._last_failure = ""
            self._adapter_failure_detail = ""

        rospy.loginfo("Waiting for the GP11 Cartesian action server...")
        if not self._wait_for_action_server(startup_timeout):
            return self._fail(2, "%s is unavailable", self._action_name)

        runtime_ready = self._wait_for(
            lambda: (
                self._runtime_anchor == anchor_side and
                self._runtime_status.startswith(
                    "MoveIt runtime synced to {} anchor".format(anchor_side)
                )
            ),
            startup_timeout,
            abort_predicate=self._controller_failure_locked,
        )
        if not runtime_ready:
            return self._fail(
                3,
                "MoveIt %s-anchor runtime is not ready: anchor=%s status=%s",
                anchor_side,
                self._runtime_anchor or "none",
                self._runtime_status or "no runtime status",
            )

        state_ready = self._wait_for(
            lambda: (
                self._trace is not None and
                self._trace.obstacle_crossing_side == crossing_side and
                self._trace.first_crossing_side == first_crossing_side and
                (
                    self._trace.obstacle_crossing_stage == "DisconnectDoneWaitFlip" or
                    (
                        self._trace.obstacle_crossing_stage == "PlannerControl" and
                        self._planner is not None
                    )
                )
            ),
            state_timeout,
            abort_predicate=self._controller_failure_locked,
        )
        if not state_ready:
            return self._fail(
                4,
                "Single flip pass %d/%d rejected: expected %s disconnect wait or empty PlannerControl, current stage=%s",
                pass_index, pass_count, crossing_side, self._stage(),
            )

        resume_empty_session = (
            self._trace.obstacle_crossing_stage == "PlannerControl" and
            self._planner is not None and
            self._planner.active and
            self._planner.accepting_commands and
            not self._planner.has_accepted_command
        )
        if (self._trace.obstacle_crossing_stage == "PlannerControl" and
                not resume_empty_session):
            return self._fail(
                5,
                "Existing PlannerControl session is not an unused session; refusing to resend"
            )

        if resume_empty_session:
            rospy.logwarn(
                "Resuming existing unused PlannerControl session %d",
                self._planner.session_id,
            )
        else:
            rospy.loginfo(
                "Pass %d/%d %s-side disconnect complete; waiting for %.2f s of stable live encoder feedback",
                pass_index, pass_count, crossing_side, self._stability_duration,
            )
            if not self._wait_for_post_disconnect_stability(crossing_side):
                return self._fail(
                    6,
                    "Post-disconnect encoders did not stabilize within %.1f s or crossing state changed; stage=%s",
                    self._stability_timeout, self._stage(),
                )
            connection_deadline = time.monotonic() + 3.0
            while (self._start_flip_pub.get_num_connections() == 0 and
                   time.monotonic() < connection_deadline and
                   not rospy.is_shutdown()):
                rospy.sleep(0.05)
            if self._start_flip_pub.get_num_connections() == 0:
                return self._fail(7, "SMC start_flip subscriber is unavailable")
            rospy.logwarn(
                "Requesting PlannerControl for audited MoveIt flip pass %d/%d: free_side=%s anchor=%s",
                pass_index, pass_count, crossing_side, anchor_side,
            )
            self._start_flip_pub.publish(Empty())

        planner_ready = self._wait_for(
            lambda: (
                self._planner is not None and
                self._planner.active and
                self._planner.accepting_commands and
                not self._planner.has_accepted_command
            ),
            5.0,
            abort_predicate=self._controller_failure_locked,
        )
        if not planner_ready:
            return self._fail(8, "PlannerControl did not become ready after start_flip")

        goal = ReachPointGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = target_frame
        goal.target.point.x = target_x
        goal.target.point.y = target_y
        goal.target.point.z = target_z
        goal.execute = True
        goal.position_tolerance = self._position_tolerance
        goal.approved_plan_id = ""
        goal.target_profile = target_profile

        def feedback_cb(feedback):
            rospy.loginfo("[%s] %s", feedback.stage_name, feedback.message)

        rospy.logwarn(
            "Planning and executing pass %d/%d %s-anchor flip target: "
            "profile=%s first_side=%s %s (%.7f, %.7f, %.7f)",
            pass_index, pass_count, anchor_side,
            target_profile, first_crossing_side,
            target_frame, target_x, target_y, target_z,
        )
        self._action.send_goal(goal, feedback_cb=feedback_cb)
        if not self._action.wait_for_result(rospy.Duration(execution_timeout)):
            self._action.cancel_goal()
            return self._fail(9, "Single flip timed out; cancellation requested")
        result = self._action.get_result()
        if result is None:
            return self._fail(10, "Single flip returned no result")
        if not result.success:
            with self._condition:
                adapter_detail = self._adapter_failure_detail
            detail_suffix = (
                "; adapter_detail={}".format(adapter_detail)
                if adapter_detail else ""
            )
            return self._fail(
                11,
                "Single flip failed: stage=%d moveit_code=%d message=%s%s",
                result.final_stage, result.moveit_error_code, result.message,
                detail_suffix,
            )

        handoff_ready = self._wait_for(
            lambda: (
                self._trace is not None and
                self._trace.obstacle_crossing_side == crossing_side and
                self._trace.obstacle_crossing_stage in (
                    "RemoteControl", "Regrip", "ReopenBeforeRemoteControl"
                )
            ),
            handoff_timeout,
            abort_predicate=self._controller_failure_locked,
        )
        if not handoff_ready:
            return self._fail(
                12,
                "Flip succeeded but lower-level RemoteControl handoff was not observed; current stage=%s",
                self._stage(),
            )

        rospy.loginfo(
            "Single flip pass %d/%d succeeded on %s side: %s",
            pass_index, pass_count, crossing_side, result.message,
        )
        rospy.loginfo(
            "Lower-level handoff confirmed: stage=%s; waiting for remote motion/completion input",
            self._stage(),
        )
        return 0
