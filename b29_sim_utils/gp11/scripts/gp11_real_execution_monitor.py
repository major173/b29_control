#!/usr/bin/env python3
"""Read-only execution evidence monitor for the GP11 real MoveIt path.

The monitor deliberately separates the things that are often conflated in a
real-robot UI:

* MoveIt requested execution;
* the B29 adapter emitted a PlannerJointCommand;
* SMC acknowledged that command and the controller wrote its command handle;
* encoder-backed /joint_states changed and converged.

It publishes a concise latched status string and an RViz text marker.  It never
publishes to a B29 controller or action server.
"""

import math
import threading

import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
)
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, ExecuteTrajectoryActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from b29_smc_auto_controller.msg import (
        AutoStateTrace,
        PlannerControlState,
        PlannerJointCommand,
    )
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


def _angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


class RealExecutionMonitor(object):
    def __init__(self):
        self._lock = threading.RLock()
        self._commanded = None
        self._command_baseline = None
        self._command_session = None
        self._command_sequence = None
        self._actual = {}
        self._last_phase = "WAITING"
        self._last_detail = "等待 MoveIt / adapter / SMC 数据"
        self._last_color = (0.25, 0.70, 1.00, 0.95)
        self._last_publish_key = None
        self._terminal_failure = False
        self._motion_threshold = max(1e-4, float(rospy.get_param("~motion_threshold", 0.005)))
        self._goal_tolerance = max(1e-4, float(rospy.get_param("~goal_tolerance", 0.025)))
        self._status_frame = rospy.get_param("~status_frame", "base_link")
        self._status_height = float(rospy.get_param("~status_height", 0.75))

        self._status_pub = rospy.Publisher(
            "/gp11_moveit/execution_monitor/status", String, queue_size=1, latch=True
        )
        self._marker_pub = rospy.Publisher(
            "/gp11_moveit/execution_monitor/markers", MarkerArray, queue_size=1, latch=True
        )
        self._command_pub = rospy.Publisher(
            "/gp11_moveit/execution_monitor/commanded_joint_targets",
            JointState,
            queue_size=1,
            latch=True,
        )
        self._actual_pub = rospy.Publisher(
            "/gp11_moveit/execution_monitor/actual_leg_joint_states",
            JointState,
            queue_size=1,
        )

        rospy.Subscriber(
            rospy.get_param(
                "~trajectory_goal_topic",
                "/gp11_moveit/reach_arm_controller/follow_joint_trajectory/goal",
            ),
            FollowJointTrajectoryActionGoal,
            self._trajectory_goal_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~trajectory_result_topic",
                "/gp11_moveit/reach_arm_controller/follow_joint_trajectory/result",
            ),
            FollowJointTrajectoryActionResult,
            self._trajectory_result_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~execute_goal_topic", "/gp11_moveit/execute_trajectory/goal"
            ),
            ExecuteTrajectoryActionGoal,
            self._execute_goal_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~execute_result_topic", "/gp11_moveit/execute_trajectory/result"
            ),
            ExecuteTrajectoryActionResult,
            self._execute_result_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic", "/joint_states"),
            JointState,
            self._joint_state_cb,
            queue_size=20,
        )

        if PlannerJointCommand is None:
            self._set_status(
                "DEGRADED",
                "未找到 b29_smc_auto_controller 消息；请先 source B29_ws/devel/setup.bash。",
                (1.0, 0.58, 0.10, 0.96),
            )
        else:
            rospy.Subscriber(
                rospy.get_param(
                    "~planner_command_topic",
                    "/b29_controller/b29_smc_auto_controller/planner_joint_command",
                ),
                PlannerJointCommand,
                self._planner_command_cb,
                queue_size=20,
            )
            rospy.Subscriber(
                rospy.get_param(
                    "~planner_state_topic",
                    "/b29_controller/b29_smc_auto_controller/planner_control_state",
                ),
                PlannerControlState,
                self._planner_state_cb,
                queue_size=20,
            )
            rospy.Subscriber(
                rospy.get_param(
                    "~state_trace_topic",
                    "/b29_controller/b29_smc_auto_controller/state_trace",
                ),
                AutoStateTrace,
                self._state_trace_cb,
                queue_size=20,
            )
            self._set_status(
                "WAITING",
                "监听 MoveIt → adapter → SMC → 编码器反馈。",
                (0.25, 0.70, 1.00, 0.95),
            )
        rospy.loginfo("GP11 real execution monitor started (read-only)")

    def _set_status(self, phase, detail, color, console_level="info", force=False):
        message = "{}: {}".format(phase, detail)
        key = (phase, detail)
        with self._lock:
            if self._terminal_failure and not force:
                return
            self._last_phase = phase
            self._last_detail = detail
            self._last_color = color
            if key == self._last_publish_key:
                return
            self._last_publish_key = key
        if console_level == "fatal":
            rospy.logfatal("[execution monitor] %s", message)
        elif console_level == "error":
            rospy.logerr("[execution monitor] %s", message)
        elif console_level == "warn":
            rospy.logwarn("[execution monitor] %s", message)
        elif console_level == "info":
            rospy.loginfo("[execution monitor] %s", message)
        self._status_pub.publish(String(data=message))
        self._publish_marker(message, color)

    def _publish_marker(self, text, color):
        background = Marker()
        background.header.frame_id = self._status_frame
        background.header.stamp = rospy.Time.now()
        background.ns = "gp11_execution_monitor"
        background.id = 0
        background.type = Marker.CUBE
        background.action = Marker.ADD
        background.pose.position.z = self._status_height
        background.pose.orientation.w = 1.0
        background.scale.x = 1.45
        background.scale.y = 0.07
        background.scale.z = 0.25
        background.color.r, background.color.g, background.color.b = color[:3]
        background.color.a = 0.18

        label = Marker()
        label.header.frame_id = self._status_frame
        label.header.stamp = background.header.stamp
        label.ns = "gp11_execution_monitor"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.z = self._status_height
        label.pose.orientation.w = 1.0
        label.scale.z = 0.075
        label.color.r, label.color.g, label.color.b = color[:3]
        label.color.a = 1.0
        label.text = text
        self._marker_pub.publish(MarkerArray(markers=[background, label]))

    def _execute_goal_cb(self, msg):
        with self._lock:
            self._terminal_failure = False
        points = len(msg.goal.trajectory.joint_trajectory.points)
        self._set_status(
            "MOVEIT_EXECUTE_REQUESTED",
            "MoveIt 收到 {} 点轨迹，等待 controller action。".format(points),
            (1.0, 0.72, 0.12, 0.96),
        )

    def _execute_result_cb(self, msg):
        if msg.result.error_code.val == 1:
            self._set_status(
                "MOVEIT_EXECUTE_SUCCEEDED",
                "MoveIt 执行 Action 成功；adapter/编码器证据见此前阶段。",
                (0.18, 0.95, 0.35, 0.96),
            )
        else:
            # The controller/adapter result is normally published just before
            # the ExecuteTrajectory result.  Keep that more useful root cause
            # visible in RViz instead of replacing it with MoveIt's generic
            # CONTROL_FAILED (-4), for example when PlannerControl is safely
            # refusing commands in lower-controller mode.
            with self._lock:
                previous_phase = self._last_phase
                previous_detail = self._last_detail
                self._terminal_failure = True
            if previous_phase in (
                "ADAPTER_FAILED",
                "SMC_REJECTED",
                "COMMAND_DISPATCH_FAILED",
            ):
                self._set_status(
                    previous_phase,
                    "{}；MoveIt Execute code {}。".format(
                        previous_detail, msg.result.error_code.val
                    ),
                    (1.0, 0.22, 0.20, 0.98),
                    console_level="error",
                    force=True,
                )
                return
            self._set_status(
                "MOVEIT_EXECUTE_FAILED",
                "MoveIt code {}。".format(msg.result.error_code.val),
                (1.0, 0.22, 0.20, 0.98),
                console_level="error",
                force=True,
            )

    def _trajectory_goal_cb(self, msg):
        trajectory = msg.goal.trajectory
        self._set_status(
            "ADAPTER_GOAL_RECEIVED",
            "adapter 收到 {} 关节 / {} 点轨迹，等待 SMC 指令。".format(
                len(trajectory.joint_names), len(trajectory.points)
            ),
            (1.0, 0.72, 0.12, 0.96),
        )

    def _trajectory_result_cb(self, msg):
        if msg.status.status == GoalStatus.SUCCEEDED and msg.result.error_code == 0:
            self._set_status(
                "ADAPTER_SUCCEEDED",
                "adapter 已确认 SMC 正常完成和终点编码器收敛。",
                (0.18, 0.95, 0.35, 0.96),
            )
        else:
            detail = msg.result.error_string or "no adapter detail"
            with self._lock:
                self._terminal_failure = True
            self._set_status(
                "ADAPTER_FAILED",
                "status={} code={} {}".format(
                    msg.status.status, msg.result.error_code, detail
                ),
                (1.0, 0.22, 0.20, 0.98),
                console_level="error",
                force=True,
            )

    def _planner_command_cb(self, msg):
        positions = tuple(float(value) for value in msg.positions)
        with self._lock:
            self._commanded = positions
            self._command_baseline = tuple(
                self._actual.get(name, float("nan")) for name in JOINT_NAMES
            )
            self._command_session = int(msg.session_id)
            self._command_sequence = int(msg.sequence)
        command = JointState()
        command.header = msg.header
        command.name = list(JOINT_NAMES)
        command.position = list(positions)
        self._command_pub.publish(command)
        self._set_status(
            "SMC_COMMAND_PUBLISHED",
            "adapter 已下发 session={} sequence={} targets=[{}]。".format(
                msg.session_id,
                msg.sequence,
                ", ".join("{:.3f}".format(value) for value in positions),
            ),
            (1.0, 0.72, 0.12, 0.96),
            console_level=None,
        )

    def _planner_state_cb(self, msg):
        with self._lock:
            session = self._command_session
            sequence = self._command_sequence
        if sequence is None or session is None or msg.session_id != session:
            return
        if msg.last_rejected_sequence == sequence and msg.reject_reason != PlannerControlState.REJECT_NONE:
            self._set_status(
                "SMC_REJECTED",
                "session={} sequence={} reject_reason={}。".format(
                    session, sequence, msg.reject_reason
                ),
                (1.0, 0.22, 0.20, 0.98),
                console_level="error",
            )
        elif msg.has_accepted_command and msg.last_accepted_sequence >= sequence:
            self._set_status(
                "SMC_ACKNOWLEDGED",
                "SMC 已接受 session={} sequence={}。".format(session, sequence),
                (0.30, 0.85, 0.36, 0.96),
                console_level=None,
            )

    def _state_trace_cb(self, msg):
        with self._lock:
            active_command = self._command_sequence is not None
        if not active_command or not msg.command_dispatch_attempted:
            return
        if msg.command_dispatch_succeeded:
            self._set_status(
                "COMMAND_DISPATCH_OK",
                "SMC 已成功写入电机命令句柄；等待编码器反馈变化。",
                (0.30, 0.85, 0.36, 0.96),
                console_level=None,
            )
        else:
            self._set_status(
                "COMMAND_DISPATCH_FAILED",
                "SMC 尝试写入电机命令句柄但失败。",
                (1.0, 0.22, 0.20, 0.98),
                console_level="error",
            )

    def _joint_state_cb(self, msg):
        actual = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
            if name in JOINT_NAMES
        }
        if len(actual) != len(JOINT_NAMES):
            return
        with self._lock:
            self._actual = actual
            commanded = self._commanded
            baseline = self._command_baseline
            sequence = self._command_sequence
        filtered = JointState()
        filtered.header = msg.header
        filtered.name = list(JOINT_NAMES)
        filtered.position = [actual[name] for name in JOINT_NAMES]
        self._actual_pub.publish(filtered)
        if commanded is None or baseline is None or sequence is None:
            return

        errors = [
            abs(_angle_error(target, actual[name]))
            for name, target in zip(JOINT_NAMES, commanded)
        ]
        movement = [
            abs(_angle_error(actual[name], start))
            for name, start in zip(JOINT_NAMES, baseline)
            if math.isfinite(start)
        ]
        max_error = max(errors)
        max_movement = max(movement) if movement else 0.0
        if max_error <= self._goal_tolerance:
            self._set_status(
                "ENCODER_AT_TARGET",
                "sequence={} 最大关节误差 {:.4f} rad。".format(sequence, max_error),
                (0.18, 0.95, 0.35, 0.96),
                console_level=None,
            )
        elif max_movement >= self._motion_threshold:
            self._set_status(
                "ENCODER_MOVING",
                "sequence={} 编码器已变化 {:.4f} rad，当前最大误差 {:.4f} rad。".format(
                    sequence, max_movement, max_error
                ),
                (0.30, 0.85, 0.36, 0.96),
                console_level=None,
            )


def main():
    rospy.init_node("gp11_real_execution_monitor")
    RealExecutionMonitor()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
