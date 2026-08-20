#!/usr/bin/env python3
"""Direct-drag, constrained Cartesian target for the GP11 real MoveIt stack.

The stock MoveIt RViz end-effector marker uses MOVE_ROTATE_3D.  In RViz that
requires Shift+Ctrl while dragging, which makes the visible centre sphere look
like an inert decoration.  This node owns a separate interactive marker with
ordinary screen-plane and axis controls, validates the dropped target through
the namespaced MoveIt IK service, and exposes ``Plan`` / ``Plan & Execute`` in
its right-click menu.

It never writes B29 hardware topics directly.  A successful execution follows
the normal real-robot path:

    /gp11_moveit/move_group -> /gp11_moveit/execute_trajectory
      -> reach_arm_controller/follow_joint_trajectory -> B29 adapter

The action is deliberately split into plan-only then execute-trajectory so the
four-joint trajectory can be checked before anything is handed to the adapter.
"""

import copy
import math
import threading
import xml.etree.ElementTree as ET

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from moveit_msgs.msg import (
    DisplayRobotState,
    DisplayTrajectory,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryGoal,
    MoveGroupAction,
    MoveItErrorCodes,
    ObjectColor,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

try:
    from b29_smc_auto_controller.msg import PlannerControlState
except ImportError:
    PlannerControlState = None

from gp11.cartesian_goal_core import (
    DEFAULT_GOAL_JOINT_DEADBAND,
    DEFAULT_JOINT_POSITION_BOUNDS,
    DEFAULT_MAX_TRAJECTORY_ACCELERATION,
    DEFAULT_MAX_TRAJECTORY_DURATION,
    DEFAULT_MAX_TRAJECTORY_POINT_DELTA,
    DEFAULT_MAX_TRAJECTORY_POINTS,
    DEFAULT_MAX_TRAJECTORY_VELOCITY,
    DEFAULT_SESSION_REARM_TIMEOUT,
    REACH_JOINTS,
    audit_trajectory,
    build_move_group_goal,
    inside_workspace,
    joint_error,
    robot_state_from_positions,
    trajectory_goal_within_deadband,
)


def _copy_pose_stamped(value):
    return copy.deepcopy(value)


def _identity_quaternion():
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


class RealReachTarget(object):
    """Own the direct-drag target and safely forward valid requests to MoveIt."""

    def __init__(self):
        self._lock = threading.RLock()
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._group_name = rospy.get_param("~group_name", "reach_arm")
        self._marker_name = rospy.get_param("~marker_name", "gp11_reach_target")
        # Large enough to remain usable at the default RViz orbit distance.
        self._marker_scale = max(0.08, float(rospy.get_param("~marker_scale", 0.38)))
        self._ik_timeout = max(0.02, float(rospy.get_param("~ik_timeout", 0.20)))
        self._planning_time = max(0.1, float(rospy.get_param("~planning_time", 4.0)))
        self._planning_attempts = max(1, int(rospy.get_param("~planning_attempts", 2)))
        self._velocity_scaling = self._unit_interval_param("~velocity_scaling", 0.20)
        self._acceleration_scaling = self._unit_interval_param("~acceleration_scaling", 0.15)
        self._goal_tolerance = max(0.001, float(rospy.get_param("~goal_tolerance", 0.008)))
        self._joint_position_bounds = self._joint_bounds_param(
            "~trajectory_joint_position_bounds", DEFAULT_JOINT_POSITION_BOUNDS
        )
        self._max_trajectory_velocity = self._positive_param(
            "~max_trajectory_velocity", DEFAULT_MAX_TRAJECTORY_VELOCITY
        )
        self._max_trajectory_acceleration = self._positive_param(
            "~max_trajectory_acceleration", DEFAULT_MAX_TRAJECTORY_ACCELERATION
        )
        self._max_trajectory_point_delta = self._positive_param(
            "~max_trajectory_point_delta", DEFAULT_MAX_TRAJECTORY_POINT_DELTA
        )
        self._max_trajectory_duration = self._positive_param(
            "~max_trajectory_duration", DEFAULT_MAX_TRAJECTORY_DURATION
        )
        self._max_trajectory_points = max(
            1, int(rospy.get_param("~max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS))
        )
        self._planning_action_timeout = max(
            self._planning_time + 2.0,
            float(rospy.get_param("~planning_action_timeout", self._planning_time + 6.0)),
        )
        self._execution_action_timeout = max(
            1.0, float(rospy.get_param("~execution_action_timeout", 110.0))
        )
        self._goal_joint_deadband = self._positive_param(
            "~goal_joint_deadband", DEFAULT_GOAL_JOINT_DEADBAND
        )
        self._session_rearm_timeout = self._positive_param(
            "~session_rearm_timeout", DEFAULT_SESSION_REARM_TIMEOUT
        )
        self._workspace_min = self._vector_param(
            "~workspace_min", (-1.50, -1.50, -1.50)
        )
        self._workspace_max = self._vector_param(
            "~workspace_max", (1.50, 1.50, 1.50)
        )
        if any(low >= high for low, high in zip(self._workspace_min, self._workspace_max)):
            raise ValueError("workspace_min must be strictly smaller than workspace_max")

        self._move_group_action = rospy.get_param(
            "~move_group_action", "/gp11_moveit/move_group"
        )
        self._execute_action = rospy.get_param(
            "~execute_action", "/gp11_moveit/execute_trajectory"
        )
        self._ik_service_name = rospy.get_param(
            "~ik_service", "/gp11_moveit/compute_ik"
        )
        self._display_trajectory_topic = rospy.get_param(
            "~display_trajectory_topic",
            "/gp11_moveit/move_group/display_planned_path",
        )
        self._joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self._anchor_topic = rospy.get_param(
            "~anchor_topic", "/gp11_moveit/runtime_anchor"
        )
        self._left_tip_link = rospy.get_param("~left_tip_link", "right_gripper_tool")
        self._right_tip_link = rospy.get_param("~right_tip_link", "left_gripper_tool")

        self._anchor_side = ""
        self._tip_link = ""
        self._target_pose = None
        self._last_valid_pose = None
        # A target is a command-space goal, not a live end-effector display.
        # It is initialized from the free tool only once.  If the anchor ever
        # changes afterwards, planning is disabled until the operator makes an
        # explicit reset; silently snapping a dragged goal to live feedback is
        # unsafe and can reverse a later plan.
        self._target_anchor_side = ""
        self._target_requires_explicit_reset = False
        self._joint_positions = {}
        self._all_joint_positions = {}
        self._planning_joint_names = self._planning_joint_names_from_param()
        self._planning_link_names = self._planning_link_names_from_param()
        self._goal_state = None
        self._validation_generation = 0
        self._operation_active = False
        self._planner_state_received = False
        self._planner_session_id = 0
        self._planner_ready = False
        self._awaiting_session_rearm = False
        self._rearm_from_session_id = 0
        self._base_target_color = (0.20, 0.65, 1.00, 0.95)
        self._color = self._base_target_color
        self._target_state_visible = False
        # InteractiveMarkerServer protects individual calls, but an insert /
        # setPose / applyChanges sequence must remain atomic across feedback,
        # timer and validation-worker threads.
        self._interactive_marker_lock = threading.Lock()
        self._interactive_marker_installed = False

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._server = InteractiveMarkerServer(
            rospy.get_param("~interactive_marker_server", "/gp11_moveit/reach_target")
        )
        self._menu = MenuHandler()
        self._menu.insert("Plan (仅规划)", callback=self._plan_menu_cb)
        self._menu.insert("Plan & Execute (发送轨迹)", callback=self._plan_execute_menu_cb)
        self._menu.insert("Reset to live tool pose", callback=self._reset_menu_cb)

        self._status_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/status", String, queue_size=1, latch=True
        )
        self._target_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/pose", PoseStamped, queue_size=1, latch=True
        )
        self._workspace_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/markers", MarkerArray, queue_size=1, latch=True
        )
        # The draggable marker itself stays structurally static.  A separate,
        # ordinary Marker supplies only the final red/green validity colour.
        # This avoids rebuilding all InteractiveMarker controls and menus after
        # every IK result, which previously triggered an RViz GL crash path.
        self._target_state_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/state_marker", Marker, queue_size=1, latch=True
        )
        self._live_robot_state_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/live_robot_state", DisplayRobotState, queue_size=1
        )
        self._goal_robot_state_pub = rospy.Publisher(
            "/gp11_moveit/reach_target/goal_robot_state", DisplayRobotState, queue_size=1,
            latch=True
        )
        self._display_pub = rospy.Publisher(
            self._display_trajectory_topic, DisplayTrajectory, queue_size=1
        )

        self._move_group_client = actionlib.SimpleActionClient(
            self._move_group_action, MoveGroupAction
        )
        self._execute_client = actionlib.SimpleActionClient(
            self._execute_action, ExecuteTrajectoryAction
        )
        self._ik_service = rospy.ServiceProxy(self._ik_service_name, GetPositionIK)

        rospy.Subscriber(self._joint_states_topic, JointState, self._joint_state_cb, queue_size=5)
        rospy.Subscriber(self._anchor_topic, String, self._anchor_cb, queue_size=1)
        if PlannerControlState is not None:
            rospy.Subscriber(
                rospy.get_param(
                    "~planner_state_topic",
                    "/b29_controller/b29_smc_auto_controller/planner_control_state",
                ),
                PlannerControlState,
                self._planner_state_cb,
                queue_size=10,
            )
        # A Marker display retains the last message across a target-node
        # restart.  Explicitly clear any old validity visual before a new
        # anchor and target are available.
        self._hide_target_state_marker(force=True)
        self._publish_workspace()
        self._publish_status("等待 runtime_anchor；目标球将在当前自由端初始化。")
        rospy.loginfo(
            "GP11 direct reach target ready: server=%s, group=%s, workspace=%s..%s",
            rospy.get_param("~interactive_marker_server", "/gp11_moveit/reach_target"),
            self._group_name,
            self._workspace_min,
            self._workspace_max,
        )

    @staticmethod
    def _unit_interval_param(name, default):
        value = float(rospy.get_param(name, default))
        if not 0.0 < value <= 1.0:
            raise ValueError("{} must be in (0, 1]".format(name))
        return value

    @staticmethod
    def _positive_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be a positive finite number".format(name))
        return value

    @staticmethod
    def _vector_param(name, default):
        value = rospy.get_param(name, list(default))
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError("{} must be a three-element numeric list".format(name))
        return tuple(float(item) for item in value)

    @staticmethod
    def _joint_bounds_param(name, defaults):
        configured = rospy.get_param(
            name,
            {joint: list(bounds) for joint, bounds in defaults.items()},
        )
        if not isinstance(configured, dict):
            raise ValueError("{} must be a mapping of joint names to [min, max]".format(name))
        bounds = {}
        for joint, fallback in defaults.items():
            value = configured.get(joint, fallback)
            if not isinstance(value, (list, tuple)) or len(value) != 2:
                raise ValueError("{}[{}] must be [min, max]".format(name, joint))
            low, high = float(value[0]), float(value[1])
            if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                raise ValueError("{}[{}] must be finite and strictly ordered".format(name, joint))
            bounds[joint] = (low, high)
        return bounds

    @staticmethod
    def _planning_urdf_names(tag, movable_only=False):
        xml_text = rospy.get_param("/gp11_moveit/robot_description", "")
        if not xml_text:
            return tuple()
        try:
            root = ET.fromstring(xml_text)
        except ET.ParseError:
            rospy.logwarn("Cannot parse /gp11_moveit/robot_description for RViz state display.")
            return tuple()
        names = []
        for element in root.findall(tag):
            if movable_only and element.get("type", "fixed") == "fixed":
                continue
            name = element.get("name", "")
            if name:
                names.append(name)
        return tuple(names)

    def _planning_joint_names_from_param(self):
        return self._planning_urdf_names("joint", movable_only=True)

    def _planning_link_names_from_param(self):
        return self._planning_urdf_names("link")

    def _display_robot_state(self, state, color):
        message = DisplayRobotState()
        message.state = copy.deepcopy(state)
        message.state.is_diff = True
        for link_name in self._planning_link_names:
            highlight = ObjectColor()
            highlight.id = link_name
            highlight.color = ColorRGBA(*color)
            message.highlight_links.append(highlight)
        return message

    def _live_robot_state(self):
        with self._lock:
            positions = dict(self._all_joint_positions)
            joint_names = self._planning_joint_names or tuple(sorted(positions))
        state = RobotState()
        state.joint_state.name = [name for name in joint_names if name in positions]
        state.joint_state.position = [positions[name] for name in state.joint_state.name]
        state.is_diff = True
        return state

    def _publish_live_robot_state(self):
        self._live_robot_state_pub.publish(
            self._display_robot_state(self._live_robot_state(), (0.92, 0.08, 0.08, 1.0))
        )

    def _publish_goal_robot_state(self, state):
        with self._lock:
            self._goal_state = copy.deepcopy(state)
        self._goal_robot_state_pub.publish(
            self._display_robot_state(state, (0.55, 0.55, 0.58, 0.35))
        )

    def _hide_goal_robot_state(self):
        hidden = DisplayRobotState()
        hidden.hide = True
        self._goal_robot_state_pub.publish(hidden)

    def _planner_state_cb(self, msg):
        ready = msg.active and msg.accepting_commands and not msg.has_accepted_command
        rearmed = False
        with self._lock:
            self._planner_state_received = True
            self._planner_session_id = msg.session_id
            self._planner_ready = ready
            if (self._awaiting_session_rearm and ready and
                    msg.session_id != self._rearm_from_session_id):
                self._awaiting_session_rearm = False
                rearmed = True
        if rearmed:
            self._publish_status(
                "PLANNER_READY: 已获得新的 PlannerControl session，可再次 Plan & Execute。"
            )

    def _session_rearm_timeout_cb(self, _event):
        with self._lock:
            if not self._awaiting_session_rearm:
                return
            self._planner_ready = False
        self._publish_status(
            "PLANNER_REARM_TIMEOUT: 未在 {:.1f}s 内获得新的 PlannerControl session；"
            "保持目标不变，禁止 Execute，请检查 planner_control_state。".format(
                self._session_rearm_timeout
            )
        )

    def _begin_session_rearm_wait(self):
        if PlannerControlState is None:
            return
        with self._lock:
            self._awaiting_session_rearm = True
            self._rearm_from_session_id = self._planner_session_id
            self._planner_ready = False
        rospy.Timer(
            rospy.Duration(self._session_rearm_timeout), self._session_rearm_timeout_cb,
            oneshot=True
        )
        self._publish_status(
            "PLANNER_REARMING: 等待 B29 保持当前位置并打开新的 PlannerControl session。"
        )

    def _publish_status(self, message):
        rospy.loginfo("[reach target] %s", message)
        self._status_pub.publish(String(data=message))

    def _joint_state_cb(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if math.isfinite(position):
                    self._all_joint_positions[name] = float(position)
                if name in REACH_JOINTS:
                    self._joint_positions[name] = float(position)
        self._publish_live_robot_state()

    def _anchor_cb(self, msg):
        side = msg.data.strip().lower()
        if side not in ("left", "right"):
            with self._lock:
                had_target = self._target_pose is not None
                self._anchor_side = ""
                self._tip_link = ""
                if had_target:
                    self._target_requires_explicit_reset = True
                    self._validation_generation += 1
                self._color = self._base_target_color
            self._hide_target_state_marker(force=True)
            self._hide_goal_robot_state()
            self._publish_status(
                "active anchor 无效；已冻结当前目标且禁止规划。锚点稳定后右键目标球，"
                "选择 Reset to live tool pose 才能继续。"
                if had_target else "等待唯一 active anchor；目标球将在当前自由端初始化。"
            )
            return

        tip_link = self._left_tip_link if side == "left" else self._right_tip_link
        with self._lock:
            changed = side != self._anchor_side or tip_link != self._tip_link
            has_target = self._target_pose is not None
            self._anchor_side = side
            self._tip_link = tip_link
            if changed and has_target:
                self._target_requires_explicit_reset = True
                self._validation_generation += 1
        # Only startup is allowed to initialize from live feedback.  A later
        # anchor transition must never overwrite an operator-selected target.
        if not has_target and not self._target_requires_explicit_reset:
            rospy.Timer(rospy.Duration(0.15), self._reset_from_live_tool, oneshot=True)
        elif changed and has_target:
            self._hide_target_state_marker()
            self._hide_goal_robot_state()
            self._publish_status(
                "active anchor 已变化；保留原目标但已禁止规划。右键目标球选择 "
                "Reset to live tool pose 后再拖动/规划。"
            )

    def _reset_menu_cb(self, _feedback):
        self._reset_from_live_tool(None, explicit=True)

    def _reset_from_live_tool(self, _event, explicit=False):
        with self._lock:
            tip_link = self._tip_link
            side = self._anchor_side
            operation_active = self._operation_active
        if operation_active:
            self._publish_status("执行/规划进行中，拒绝重置目标球以免覆盖当前命令。")
            return
        if not tip_link:
            self._publish_status("尚未收到 left/right runtime_anchor，无法初始化目标球。")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, tip_link, rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception as exc:
            self._publish_status("等待 {} -> {} TF：{}".format(self._world_frame, tip_link, exc))
            rospy.Timer(rospy.Duration(0.5), self._reset_from_live_tool, oneshot=True)
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self._world_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        # Only Cartesian position is intentionally controlled.  World-aligned
        # axes make the drag handles predictable regardless of the tool yaw.
        pose.pose.orientation = _identity_quaternion()
        with self._lock:
            self._target_pose = _copy_pose_stamped(pose)
            self._last_valid_pose = _copy_pose_stamped(pose)
            self._target_anchor_side = side
            self._target_requires_explicit_reset = False
            self._color = self._base_target_color
        self._install_or_set_interactive_marker(pose)
        # The blue draggable sphere is the only target at its centre.  Red or
        # green appears later as a separate, offset status indicator.
        self._hide_target_state_marker(force=True)
        self._hide_goal_robot_state()
        self._target_pub.publish(pose)
        self._publish_status(
            "目标球已{}到 {}（{} anchor）。左键拖球/箭头，松开后做 IK+碰撞预检。".format(
                "按用户请求重置" if explicit else "初始化",
                tip_link, side
            )
        )

    def _build_marker(self, pose):
        if pose is None:
            return None

        marker = InteractiveMarker()
        marker.header = self._latest_world_header()
        # The target itself is expressed in the anchored planning `world`,
        # while RViz may deliberately use the live `base_link` as its Fixed
        # Frame for a level view.  Do not pin an interactive marker to the
        # one-time reset stamp: after the TF cache ages out, RViz could no
        # longer resolve world -> base_link and the draggable ball vanished.
        # A zero stamp explicitly requests the latest transform and leaves the
        # goal coordinates/feedback semantics unchanged.
        marker.header.stamp = rospy.Time(0)
        marker.name = self._marker_name
        marker.description = "GP11 reach target — 左键拖球/彩色轴；右键 Plan / Plan & Execute"
        marker.scale = self._marker_scale
        marker.pose = pose.pose

        # The centre ball is a view-facing MOVE_PLANE.  Unlike MoveIt\'s
        # MOVE_ROTATE_3D control it works with an ordinary left drag.  The
        # three always-visible arrows provide deterministic depth/X/Y/Z edits.
        plane = InteractiveMarkerControl()
        plane.name = "screen_plane"
        plane.orientation = _identity_quaternion()
        plane.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        plane.independent_marker_orientation = True
        plane.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        plane.always_visible = True
        plane.description = "左键拖动：当前视图平面"
        plane.markers.append(self._sphere_marker(self._base_target_color))
        marker.controls.append(plane)

        axis_specs = (
            ("move_x", Quaternion(x=0.0, y=0.0, z=0.0, w=1.0), (1.0, 0.20, 0.20), "X"),
            ("move_y", Quaternion(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5)), (0.20, 1.0, 0.20), "Y"),
            ("move_z", Quaternion(x=0.0, y=-math.sqrt(0.5), z=0.0, w=math.sqrt(0.5)), (0.20, 0.45, 1.0), "Z"),
        )
        for name, orientation, axis_color, axis_name in axis_specs:
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation = orientation
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.always_visible = True
            control.description = "拖动 {} 轴".format(axis_name)
            control.markers.append(self._arrow_marker(axis_color))
            marker.controls.append(control)

        return marker

    def _latest_world_header(self):
        header = Header()
        header.frame_id = self._world_frame
        # RViz must resolve world -> base_link at its newest TF, not at the
        # one-time target-reset stamp.
        header.stamp = rospy.Time(0)
        return header

    def _sphere_marker(self, color):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.pose.orientation = _identity_quaternion()
        marker.scale.x = self._marker_scale * 0.62
        marker.scale.y = self._marker_scale * 0.62
        marker.scale.z = self._marker_scale * 0.62
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _arrow_marker(self, color):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.pose.position.x = self._marker_scale * 0.48
        marker.pose.orientation = _identity_quaternion()
        marker.scale.x = self._marker_scale * 0.72
        marker.scale.y = self._marker_scale * 0.11
        marker.scale.z = self._marker_scale * 0.11
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.92
        return marker

    def _install_or_set_interactive_marker(self, pose):
        """Create controls once; later resets are pose-only updates."""
        with self._interactive_marker_lock:
            if not self._interactive_marker_installed:
                marker = self._build_marker(pose)
                if marker is None:
                    return
                self._server.insert(marker, self._feedback_cb)
                self._menu.apply(self._server, marker.name)
                self._server.applyChanges()
                self._interactive_marker_installed = True
                return

            if not self._server.setPose(
                self._marker_name, pose.pose, self._latest_world_header()
            ):
                rospy.logwarn_throttle(
                    1.0, "Could not apply pose-only update to the GP11 reach target."
                )
                return
            self._server.applyChanges()

    def _hide_target_state_marker(self, force=False):
        with self._lock:
            if not self._target_state_visible and not force:
                return
            self._target_state_visible = False

        marker = Marker()
        marker.header = self._latest_world_header()
        marker.ns = "gp11_reach_target_state"
        marker.id = 0
        marker.action = Marker.DELETE
        self._target_state_pub.publish(marker)

    def _show_target_state_marker(self):
        with self._lock:
            if self._target_pose is None:
                return
            target = _copy_pose_stamped(self._target_pose)
            color = tuple(self._color)
            self._target_state_visible = True

        marker = Marker()
        marker.header = self._latest_world_header()
        marker.ns = "gp11_reach_target_state"
        marker.id = 0
        marker.action = Marker.ADD
        marker.pose = target.pose
        # A same-position overlay is a second selectable RViz object: it can
        # look like a second ball and win the mouse-picking pass over the
        # InteractiveMarker.  Use a small status cube on the negative X side
        # instead.  The target has only positive-axis drag arrows, so this
        # indicator is spatially separate from both the centre ball and its
        # handles.
        marker.type = Marker.CUBE
        marker.pose.position.x -= self._marker_scale * 0.72
        marker.scale.x = self._marker_scale * 0.22
        marker.scale.y = self._marker_scale * 0.22
        marker.scale.z = self._marker_scale * 0.22
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        self._target_state_pub.publish(marker)

    def _feedback_pose(self, feedback):
        frame_id = feedback.header.frame_id or self._world_frame
        if frame_id != self._world_frame:
            raise ValueError(
                "interactive target must remain in {}, got {}".format(
                    self._world_frame, frame_id
                )
            )
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self._world_frame
        pose.pose.position = copy.deepcopy(feedback.pose.position)
        pose.pose.orientation = _identity_quaternion()
        return pose

    def _inside_workspace(self, pose):
        return inside_workspace(
            pose.pose.position, self._workspace_min, self._workspace_max
        )

    def _feedback_cb(self, feedback):
        if feedback.marker_name != self._marker_name:
            return
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # MenuHandler dispatches the selected action callback.
            return
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        try:
            candidate = self._feedback_pose(feedback)
        except ValueError as exc:
            self._reject_target(str(exc))
            return

        if not self._inside_workspace(candidate):
            self._reject_target("目标超出工作空间边界，已回退到上一个可达点。")
            return

        with self._lock:
            self._target_pose = _copy_pose_stamped(candidate)
            self._validation_generation += 1
            generation = self._validation_generation
        # While dragging, hide the last final-state overlay so it never lags
        # behind the interactive sphere.  It is restored once validation ends.
        self._hide_target_state_marker()
        self._hide_goal_robot_state()
        self._target_pub.publish(candidate)
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._publish_status("正在验证目标可达性与终点碰撞…")
            worker = threading.Thread(
                target=self._validate_target_worker, args=(generation, candidate)
            )
            worker.daemon = True
            worker.start()

    def _reject_target(self, reason):
        with self._lock:
            if self._last_valid_pose is not None:
                self._target_pose = _copy_pose_stamped(self._last_valid_pose)
            self._color = (1.00, 0.24, 0.22, 0.96)
            target = _copy_pose_stamped(self._target_pose) if self._target_pose else None
        if target is not None:
            self._install_or_set_interactive_marker(target)
        self._show_target_state_marker()
        if target is not None:
            self._target_pub.publish(target)
        self._publish_status("TARGET_REJECTED: {}".format(reason))

    def _validate_target_worker(self, generation, candidate):
        valid, reason, solution = self._check_reachable(candidate)
        with self._lock:
            if generation != self._validation_generation:
                return
            if valid:
                self._last_valid_pose = _copy_pose_stamped(candidate)
                self._target_pose = _copy_pose_stamped(candidate)
                self._color = (0.20, 0.92, 0.36, 0.96)
            else:
                if self._last_valid_pose is not None:
                    self._target_pose = _copy_pose_stamped(self._last_valid_pose)
                self._color = (1.00, 0.24, 0.22, 0.96)
            target = _copy_pose_stamped(self._target_pose) if self._target_pose else None
        # A successful drag has already updated the InteractiveMarker pose in
        # RViz.  Only a rejected target needs a pose-only rollback.  Neither
        # path rebuilds controls, arrows or the menu.
        if not valid and target is not None:
            self._install_or_set_interactive_marker(target)
        if valid and solution is not None:
            self._publish_goal_robot_state(solution)
        elif not valid:
            with self._lock:
                previous_goal = copy.deepcopy(self._goal_state)
            if previous_goal is not None:
                self._publish_goal_robot_state(previous_goal)
        self._show_target_state_marker()
        if target is not None:
            self._target_pub.publish(target)
        self._publish_status(
            "TARGET_READY: {}".format(reason)
            if valid else "TARGET_REJECTED: {}；已回退。".format(reason)
        )

    def _robot_state(self):
        with self._lock:
            values = dict(self._joint_positions)
        return robot_state_from_positions(values)

    def _check_reachable(self, pose):
        with self._lock:
            tip_link = self._tip_link
        if not tip_link:
            return False, "没有 active anchor / tip link", None
        if not self._inside_workspace(pose):
            return False, "目标超出工作空间", None
        try:
            rospy.wait_for_service(self._ik_service_name, timeout=0.5)
        except rospy.ROSException:
            return False, "MoveIt IK 服务尚未就绪", None

        request = GetPositionIKRequest()
        request.ik_request.group_name = self._group_name
        request.ik_request.ik_link_name = tip_link
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = self._robot_state()
        request.ik_request.timeout = rospy.Duration(self._ik_timeout)
        request.ik_request.avoid_collisions = True
        try:
            response = self._ik_service(request)
        except rospy.ServiceException as exc:
            return False, "IK 调用失败：{}".format(exc), None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return False, "IK/终点碰撞检查失败（MoveIt code {}）".format(
                response.error_code.val
            ), None
        return True, "IK 与终点碰撞检查通过", response.solution

    def _plan_menu_cb(self, _feedback):
        self._start_operation(execute=False)

    def _plan_execute_menu_cb(self, _feedback):
        self._start_operation(execute=True)

    def _start_operation(self, execute):
        with self._lock:
            if self._operation_active:
                self._publish_status("已有 Plan / Execute 正在进行，请等待结果。")
                return
            if self._target_pose is None:
                self._publish_status("目标球尚未初始化，不能规划。")
                return
            if (self._target_requires_explicit_reset or
                    self._target_anchor_side != self._anchor_side):
                self._publish_status(
                    "目标球不再对应当前 active anchor，拒绝规划。右键目标球选择 "
                    "Reset to live tool pose 后重新拖动。"
                )
                return
            if execute and PlannerControlState is not None:
                if self._awaiting_session_rearm:
                    self._publish_status("PLANNER_REARMING: 正在等待新 session，暂不发送 Execute。")
                    return
                if not self._planner_state_received or not self._planner_ready:
                    self._publish_status(
                        "PlannerControl 尚未就绪；需要 active=true、accepting_commands=true、"
                        "has_accepted_command=false 才能 Execute。"
                    )
                    return
            self._operation_active = True
            target = _copy_pose_stamped(self._target_pose)
        worker = threading.Thread(target=self._run_operation, args=(target, execute))
        worker.daemon = True
        worker.start()

    def _build_plan_goal(self, target):
        with self._lock:
            tip_link = self._tip_link
        return build_move_group_goal(
            target, tip_link, self._group_name,
            self._workspace_min, self._workspace_max,
            self._goal_tolerance, self._planning_attempts,
            self._planning_time, self._velocity_scaling,
            self._acceleration_scaling,
        )

    def _trajectory_is_safe(self, trajectory):
        return audit_trajectory(
            trajectory,
            self._joint_position_bounds,
            self._max_trajectory_velocity,
            self._max_trajectory_acceleration,
            self._max_trajectory_point_delta,
            self._max_trajectory_duration,
            self._max_trajectory_points,
        )

    def _publish_plan_preview(self, result):
        display = DisplayTrajectory()
        display.model_id = "gp11"
        display.trajectory_start = result.trajectory_start
        display.trajectory = [result.planned_trajectory]
        self._display_pub.publish(display)

    @staticmethod
    def _joint_error(joint_name, desired, actual):
        return joint_error(joint_name, desired, actual)

    def _goal_state_from_trajectory(self, result):
        state = copy.deepcopy(result.trajectory_start)
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        trajectory = result.planned_trajectory.joint_trajectory
        final_point = trajectory.points[-1]
        positions.update(dict(zip(trajectory.joint_names, final_point.positions)))
        state.joint_state.name = list(positions)
        state.joint_state.position = [positions[name] for name in state.joint_state.name]
        state.is_diff = True
        return state

    def _trajectory_goal_within_deadband(self, trajectory):
        with self._lock:
            actual_positions = dict(self._joint_positions)
        return trajectory_goal_within_deadband(
            trajectory, actual_positions, self._goal_joint_deadband
        )

    def _run_operation(self, target, execute):
        try:
            valid, reason, _solution = self._check_reachable(target)
            if not valid:
                self._finish_operation(False, "拒绝规划：{}".format(reason))
                return
            if not self._move_group_client.wait_for_server(rospy.Duration(2.0)):
                self._finish_operation(False, "MoveGroup Action 未就绪：{}".format(self._move_group_action))
                return

            self._publish_status(
                "PLANNING: group={}, tip={}, velocity={:.2f}, acceleration={:.2f}".format(
                    self._group_name,
                    self._tip_link,
                    self._velocity_scaling,
                    self._acceleration_scaling,
                )
            )
            self._move_group_client.send_goal(self._build_plan_goal(target))
            if not self._move_group_client.wait_for_result(
                    rospy.Duration(self._planning_action_timeout)):
                self._move_group_client.cancel_goal()
                self._finish_operation(False, "MoveIt 规划超时，已取消。"); return
            result = self._move_group_client.get_result()
            if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
                code = "<no result>" if result is None else str(result.error_code.val)
                self._finish_operation(False, "MoveIt 规划失败（code {}）。".format(code))
                return
            safe, reason = self._trajectory_is_safe(result.planned_trajectory)
            if not safe:
                self._finish_operation(False, "拒绝执行：{}".format(reason))
                return
            self._publish_plan_preview(result)
            self._publish_goal_robot_state(self._goal_state_from_trajectory(result))
            if not execute:
                self._finish_operation(True, "PLAN_READY: {}；已发布轨迹预览。".format(reason))
                return

            if self._trajectory_goal_within_deadband(result.planned_trajectory):
                self._finish_operation(
                    True,
                    "ALREADY_AT_GOAL: 终点位于 {:.3f} rad 关节死区内，未发送电机轨迹。".format(
                        self._goal_joint_deadband
                    ),
                )
                return

            if not self._execute_client.wait_for_server(rospy.Duration(2.0)):
                self._finish_operation(False, "执行 Action 未就绪：{}".format(self._execute_action))
                return
            self._publish_status(
                "EXECUTION_REQUESTED: 已审查四关节轨迹，发送给 MoveIt trajectory execution。"
            )
            execute_goal = ExecuteTrajectoryGoal()
            execute_goal.trajectory = result.planned_trajectory
            self._execute_client.send_goal(execute_goal)
            if not self._execute_client.wait_for_result(
                    rospy.Duration(self._execution_action_timeout)):
                self._execute_client.cancel_goal()
                self._finish_operation(False, "执行超时，已向 MoveIt 请求取消。")
                self._begin_session_rearm_wait()
                return
            execute_result = self._execute_client.get_result()
            if execute_result is None or execute_result.error_code.val != MoveItErrorCodes.SUCCESS:
                code = "<no result>" if execute_result is None else str(execute_result.error_code.val)
                self._finish_operation(False, "MoveIt 执行失败（code {}）；检查 execution monitor。".format(code))
                self._begin_session_rearm_wait()
                return
            self._finish_operation(True, "EXECUTION_SUCCEEDED: MoveIt 已报告轨迹执行成功；请以编码器反馈为准。")
            self._begin_session_rearm_wait()
        except Exception as exc:
            rospy.logerr("Reach target operation failed: %s", exc)
            self._finish_operation(False, "内部异常：{}".format(exc))

    def _finish_operation(self, success, message):
        with self._lock:
            self._operation_active = False
            self._color = (0.20, 0.92, 0.36, 0.96) if success else (1.00, 0.24, 0.22, 0.96)
        self._show_target_state_marker()
        self._publish_status(message)

    def _publish_workspace(self):
        x0, y0, z0 = self._workspace_min
        x1, y1, z1 = self._workspace_max
        corners = (
            Point(x0, y0, z0), Point(x1, y0, z0), Point(x1, y1, z0), Point(x0, y1, z0),
            Point(x0, y0, z1), Point(x1, y0, z1), Point(x1, y1, z1), Point(x0, y1, z1),
        )
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        line = Marker()
        line.header.frame_id = self._world_frame
        line.header.stamp = rospy.Time.now()
        line.ns = "gp11_reach_workspace"
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation = _identity_quaternion()
        line.scale.x = 0.008
        line.color.r, line.color.g, line.color.b, line.color.a = (0.10, 0.70, 1.00, 0.32)
        for start, end in edges:
            line.points.append(corners[start])
            line.points.append(corners[end])

        # Remove the legacy workspace label as well when this node is restarted
        # while RViz remains open: MarkerArray does not remove omitted IDs by
        # itself.  The boundary is now just a static line box, while target
        # validation is communicated solely through the red/green overlay.
        delete_legacy_label = Marker()
        delete_legacy_label.header = self._latest_world_header()
        delete_legacy_label.ns = "gp11_reach_workspace"
        delete_legacy_label.id = 1
        delete_legacy_label.action = Marker.DELETE
        self._workspace_pub.publish(MarkerArray(markers=[line, delete_legacy_label]))


def main():
    rospy.init_node("gp11_real_reach_target")
    RealReachTarget()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
