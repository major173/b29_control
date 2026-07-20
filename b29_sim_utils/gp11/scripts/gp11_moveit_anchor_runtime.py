#!/usr/bin/env python3
"""Start a real-robot MoveIt runtime stack aligned to the anchor pose."""

import os
import threading
import xml.etree.ElementTree as ET
import importlib.util
import shlex
import subprocess
import time

import numpy
import rospy
import tf2_ros
import yaml
import actionlib
from rospkg import RosPack
from moveit_msgs.msg import MoveGroupAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_matrix, translation_matrix

# Anchor pose provider factory  (bundled with the gp11 package).
_ANCHOR_POSE_PROVIDER_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "gp11", "moveit", "anchor_pose_provider.py",
)


def _load_anchor_pose_factory():
    spec = importlib.util.spec_from_file_location(
        "gp11_anchor_pose_provider_src", _ANCHOR_POSE_PROVIDER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(
            "Failed to load anchor_pose_provider from {}".format(_ANCHOR_POSE_PROVIDER_PATH)
        )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.make_anchor_pose_provider


_make_anchor_pose_provider = None

class JointStateCache(object):
    def __init__(self, topic_name):
        self._lock = threading.Lock()
        self._positions = {}
        self._last_update_wall_time = None
        rospy.Subscriber(topic_name, JointState, self._callback, queue_size=10)

    def _callback(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._positions[name] = float(position)
            self._last_update_wall_time = time.time()

    def get_position(self, joint_name):
        with self._lock:
            return self._positions.get(joint_name)

    def missing_or_stale(self, joint_names, timeout):
        with self._lock:
            if self._last_update_wall_time is None:
                return list(joint_names), "no messages received"
            age = time.time() - self._last_update_wall_time
            if age > timeout:
                return list(joint_names), "last message age {:.2f}s".format(age)
            return [name for name in joint_names if name not in self._positions], ""


class MoveItAnchorRuntime(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._sync_lock = threading.Lock()
        self._anchor_mode = rospy.get_param("~anchor_mode", "right").strip().lower()
        self._joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self._joint_state_timeout = max(
            0.0, float(rospy.get_param("~joint_state_timeout", 0.5))
        )
        self._required_joint_names = tuple(rospy.get_param(
            "~required_joint_names",
            [
                "left_first_leg_joint",
                "left_second_leg_joint",
                "right_first_leg_joint",
                "right_second_leg_joint",
            ],
        ))
        raw_fixed_joint_positions = rospy.get_param(
            "~planning_fixed_joint_positions", {}
        )
        if not isinstance(raw_fixed_joint_positions, dict):
            raise ValueError(
                "~planning_fixed_joint_positions must be a mapping of joint names to positions"
            )
        self._planning_fixed_joint_positions = {
            str(name): float(position)
            for name, position in raw_fixed_joint_positions.items()
        }
        raw_inverse_axis_signs = rospy.get_param("~inverse_joint_axis_signs", {})
        if not isinstance(raw_inverse_axis_signs, dict):
            raise ValueError("~inverse_joint_axis_signs must map joint names to +1 or -1")
        self._inverse_joint_axis_signs = {}
        for name, sign in raw_inverse_axis_signs.items():
            value = float(sign)
            if value not in (-1.0, 1.0):
                raise ValueError(
                    "~inverse_joint_axis_signs[{}] must be exactly +1 or -1".format(name)
                )
            self._inverse_joint_axis_signs[str(name)] = value
        self._joint_state_cache = JointStateCache(self._joint_states_topic)
        self._real_base_frame = rospy.get_param("~real_base_frame", "base_link")
        self._real_anchor_links = {
            "left": rospy.get_param("~real_left_anchor_link", "l_gripper_left_drive"),
            "right": rospy.get_param("~real_right_anchor_link", "r_gripper_left_drive"),
        }
        self._real_tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._real_tf_listener = tf2_ros.TransformListener(self._real_tf_buffer)
        self._rviz_enabled = bool(rospy.get_param("~rviz", True))
        self._rviz_config = rospy.get_param("~rviz_config", "")
        self._launch_point_target = bool(rospy.get_param("~launch_point_target", False))
        self._start_trajectory_bridge = bool(rospy.get_param("~start_trajectory_bridge", True))
        self._allow_trajectory_execution = bool(
            rospy.get_param("~allow_trajectory_execution", True)
        )
        self._start_robot_state_publisher = bool(rospy.get_param("~start_robot_state_publisher", True))
        self._start_filtered_joint_states = bool(rospy.get_param("~start_filtered_joint_states", True))
        self._moveit_param_namespace = rospy.get_param(
            "~moveit_param_namespace", ""
        ).rstrip("/")
        # Keep the planning model parameters private to GP11, but allow the
        # MoveGroup/RViz interaction API to use the root namespace exactly as
        # the proven Gazebo stack does.  Separating these two namespaces also
        # prevents RViz from hot-switching '/' -> '/gp11_moveit', which races
        # RobotInteraction's marker feedback map in MoveIt 1.1.16.
        self._move_group_namespace = rospy.get_param(
            "~move_group_namespace", self._moveit_param_namespace
        ).rstrip("/")
        self._settle_time = max(0.0, float(rospy.get_param("~settle_time", 1.0)))
        self._move_group_ready_timeout = max(
            0.0, float(rospy.get_param("~move_group_ready_timeout", 15.0))
        )
        # get_planning_scene becomes available before move_group has finished
        # constructing its action server and planning-scene monitor.  RViz
        # started in that window can leave its native marker feedback map stale.
        self._rviz_start_delay = max(
            0.0, float(rospy.get_param("~rviz_start_delay", 0.5))
        )
        self._failed_retry_delay = max(
            0.0, float(rospy.get_param("~failed_retry_delay", 5.0))
        )
        self._next_retry_wall_time = 0.0
        self._last_failure_reason = ""
        self._load_support_bar = bool(rospy.get_param("~load_support_bar", True))

        # --- Anchor pose provider  (gazebo | real_robot) ---
        global _make_anchor_pose_provider
        if _make_anchor_pose_provider is None:
            _make_anchor_pose_provider = _load_anchor_pose_factory()
        self._anchor_pose_provider_name = rospy.get_param("~anchor_pose_provider", "real_robot")
        anchor_pose_kwargs = {}
        if self._anchor_pose_provider_name in ("real_robot", "real"):
            anchor_pose_kwargs["fixed_xyz"] = (
                float(rospy.get_param("~anchor_fixed_x", 0.0)),
                float(rospy.get_param("~anchor_fixed_y", 0.0)),
                float(rospy.get_param("~anchor_fixed_z", 0.0)),
            )
            anchor_pose_kwargs["fixed_rpy"] = (
                float(rospy.get_param("~anchor_fixed_roll", 0.0)),
                float(rospy.get_param("~anchor_fixed_pitch", 0.0)),
                float(rospy.get_param("~anchor_fixed_yaw", 0.0)),
            )
        self._anchor_pose_provider = _make_anchor_pose_provider(
            self._anchor_pose_provider_name, **anchor_pose_kwargs
        )
        rospy.loginfo(
            "Anchor pose provider: %s", self._anchor_pose_provider_name
        )
        self._status_pub = rospy.Publisher("/gp11_moveit/runtime_status", String, queue_size=1, latch=True)
        self._active_anchor_pub = rospy.Publisher("/gp11_moveit/runtime_anchor", String, queue_size=1, latch=True)
        self._left_attached = False
        self._right_attached = False
        self._last_change_wall_time = time.time()
        self._synced_side = None
        self._pending_sync = True
        self._processes = {}

        self._package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        try:
            self._package_dir = RosPack().get_path("gp11")
        except Exception:
            pass
        self._anchored_urdf = self._load_anchored_urdf_module()
        self._source_urdf_path = rospy.get_param(
            "~source_urdf_path",
            os.path.join(self._package_dir, "urdf", "gp11_double_slider.urdf"),
        )
        self._kinematics_path = rospy.get_param(
            "~kinematics_path",
            os.path.join(self._package_dir, "config", "gp11_moveit_kinematics.yaml"),
        )
        self._joint_limits_path = rospy.get_param(
            "~joint_limits_path",
            os.path.join(self._package_dir, "config", "gp11_moveit_joint_limits.yaml"),
        )
        self._ompl_path = rospy.get_param(
            "~ompl_path",
            os.path.join(self._package_dir, "config", "gp11_moveit_ompl_planning.yaml"),
        )
        self._controllers_path = rospy.get_param(
            "~controllers_path",
            os.path.join(self._package_dir, "config", "gp11_moveit_controllers.yaml"),
        )
        self._srdf_paths = {
            "left": rospy.get_param(
                "~left_srdf_path",
                os.path.join(self._package_dir, "config", "gp11_moveit_left_anchor.srdf"),
            ),
            "right": rospy.get_param(
                "~right_srdf_path",
                os.path.join(self._package_dir, "config", "gp11_moveit_right_anchor.srdf"),
            ),
        }
        self._anchor_links = {
            "left": rospy.get_param("~left_anchor_link", "l_gripper_left_drive"),
            "right": rospy.get_param("~right_anchor_link", "r_gripper_left_drive"),
        }
        self._anchor_joint_names = {
            "left": rospy.get_param("~left_anchor_joint", "l_gripper_left_drive_joint"),
            "right": rospy.get_param("~right_anchor_joint", "r_gripper_left_drive_joint"),
        }
        self._anchor_joint_defaults = {
            "left": float(rospy.get_param("~anchor_slider_position", 0.005)),
            "right": float(rospy.get_param("~anchor_slider_position", 0.005)),
        }
        self._robot_model_name = rospy.get_param("~robot_model_name", "gp11")
        self._left_state_service = rospy.get_param("~left_state_service", "/gp11/support_plugin/left/get_state")
        self._right_state_service = rospy.get_param("~right_state_service", "/gp11/support_plugin/right/get_state")
        self._get_link_state_service = rospy.get_param("~get_link_state_service", "/gazebo/get_link_state")
        self._filtered_joint_states_topic = rospy.get_param("~filtered_joint_states_topic", "/gp11_moveit/joint_states")
        self._support_bar_params = {
            "object_id": rospy.get_param("~support_bar_object_id", "support_bar"),
            "frame_id": rospy.get_param("~support_bar_frame_id", "world"),
            "length": float(rospy.get_param("~support_bar_length", 3.0)),
            "radius": float(rospy.get_param("~support_bar_radius", 0.007)),
            "x": float(rospy.get_param("~support_bar_x", 0.0)),
            "y": float(rospy.get_param("~support_bar_y", 1.002)),
            "z": float(rospy.get_param("~support_bar_z", 2.0)),
            "roll": float(rospy.get_param("~support_bar_roll", 1.5707963267948966)),
            "pitch": float(rospy.get_param("~support_bar_pitch", 0.0)),
            "yaw": float(rospy.get_param("~support_bar_yaw", 0.0)),
        }

        rospy.Service("/gp11_moveit/sync_anchor", Trigger, self._sync_service_cb)

        # Subscribe to attach-state Bool topics  (push updates from either
        # the Gazebo support plugin or the real B29 controller).
        rospy.Subscriber(
            "/gp11/support_plugin/left/attached", Bool,
            self._left_attached_cb, queue_size=1,
        )
        rospy.Subscriber(
            "/gp11/support_plugin/right/attached", Bool,
            self._right_attached_cb, queue_size=1,
        )

        self._worker = threading.Thread(target=self._worker_loop)
        self._worker.daemon = True
        self._worker.start()
        rospy.on_shutdown(self._shutdown)
        self._publish_status("Waiting for support attach before starting MoveIt runtime.")

    def _load_anchored_urdf_module(self):
        module_path = os.path.join(self._package_dir, "scripts", "gp11_generate_anchored_urdf.py")
        spec = importlib.util.spec_from_file_location("gp11_generate_anchored_urdf_src", module_path)
        if spec is None or spec.loader is None:
            raise RuntimeError("Failed to load anchored URDF generator from {}".format(module_path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    @staticmethod
    def _parse_attach_message(message):
        text = str(message).strip().lower()
        return text in ("attached", "already attached", "true", "attached=true")

    def _query_state_service(self, service_name):
        try:
            rospy.wait_for_service(service_name, timeout=0.2)
        except rospy.ROSException:
            return False, False
        try:
            response = rospy.ServiceProxy(service_name, Trigger)()
        except rospy.ServiceException:
            return False, False
        return True, self._parse_attach_message(response.message)

    def _publish_status(self, message):
        rospy.loginfo(message)
        self._status_pub.publish(String(data=message))

    def _set_attached_state(self, side, attached):
        changed = False
        with self._lock:
            key = "_{}_attached".format(side)
            if getattr(self, key) != attached:
                setattr(self, key, attached)
                self._last_change_wall_time = time.time()
                self._pending_sync = True
                changed = True
        if changed:
            self._active_anchor_pub.publish(String(data=self._resolved_anchor_mode()))

    def _left_attached_cb(self, msg):
        self._set_attached_state("left", bool(msg.data))

    def _right_attached_cb(self, msg):
        self._set_attached_state("right", bool(msg.data))

    def _refresh_attach_state(self):
        # Real-robot mode relies on push updates from Bool topics.
        if self._anchor_pose_provider_name in ("real_robot", "real"):
            return
        # Gazebo mode: poll the support plugin services.
        left_ok, left_attached = self._query_state_service(self._left_state_service)
        right_ok, right_attached = self._query_state_service(self._right_state_service)
        if left_ok:
            self._set_attached_state("left", left_attached)
        if right_ok:
            self._set_attached_state("right", right_attached)

    def _resolved_anchor_mode(self):
        if self._anchor_mode in ("left", "right"):
            return self._anchor_mode
        if self._left_attached and self._right_attached:
            return "both"
        if self._left_attached:
            return "left"
        if self._right_attached:
            return "right"
        return "none"

    def _selected_side(self):
        if self._left_attached and self._right_attached:
            return None, "Both supports are attached; detach one side first."
        if self._anchor_mode in ("left", "right"):
            expected = self._anchor_mode
            attached = self._left_attached if expected == "left" else self._right_attached
            if attached:
                return expected, ""
            other_attached = self._right_attached if expected == "left" else self._left_attached
            if other_attached:
                return None, "Attached side does not match anchor_mode={}".format(expected)
            return None, "Waiting for {} support to be attached.".format(expected)
        if self._left_attached:
            return "left", ""
        if self._right_attached:
            return "right", ""
        return None, "Waiting for support attach."

    @staticmethod
    def _load_yaml(path):
        with open(path, "r") as stream:
            return yaml.safe_load(stream) or {}

    @staticmethod
    def _set_param_tree(prefix, value):
        if isinstance(value, dict):
            for key, child in value.items():
                child_prefix = "{}/{}".format(prefix.rstrip("/"), key)
                MoveItAnchorRuntime._set_param_tree(child_prefix, child)
            return
        rospy.set_param(prefix, value)

    def _apply_moveit_parameters(self, side, robot_description_xml):
        ns = self._moveit_param_namespace
        rospy.set_param(ns + "/robot_description", robot_description_xml)
        with open(self._srdf_paths[side], "r") as stream:
            rospy.set_param(ns + "/robot_description_semantic", stream.read())
        rospy.set_param(ns + "/robot_description_kinematics", self._load_yaml(self._kinematics_path))
        rospy.set_param(ns + "/robot_description_planning", self._load_yaml(self._joint_limits_path))

        move_group_params = {
            "allow_trajectory_execution": self._allow_trajectory_execution,
            "capabilities": "",
            "disable_capabilities": "",
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
            "octomap_resolution": 0.01,
            "planning_scene_monitor": {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
            "planning_scene_monitor_options": {
                # Absolute name: the B29 /robot_description remains untouched,
                # while MoveGroup loads GP11's re-rooted planning model.
                "robot_description": ns + "/robot_description",
                "joint_state_topic": self._joint_states_topic,
            },
            "trajectory_execution": {
                "allowed_execution_duration_scaling": float(
                    rospy.get_param("~allowed_execution_duration_scaling", 6.0)
                ),
                "allowed_goal_duration_margin": float(
                    rospy.get_param("~allowed_goal_duration_margin", 2.0)
                ),
                "allowed_start_tolerance": float(rospy.get_param("~allowed_start_tolerance", 0.01)),
            },
            "moveit_manage_controllers": False,
        }
        ompl = self._load_yaml(self._ompl_path)
        controllers = self._load_yaml(self._controllers_path)
        move_group_params.update(ompl)
        move_group_params.update(controllers)
        self._set_param_tree(self._move_group_namespace + "/move_group", move_group_params)

    def _query_anchor_pose(self, side):
        return self._anchor_pose_provider.get_anchor_pose(
            side, self._robot_model_name, self._anchor_links[side]
        )

    def _load_source_robot(self):
        """Return the authoritative source URDF as an XML root element."""
        source_urdf_param = rospy.get_param("~source_urdf_param", "")
        if source_urdf_param:
            if not rospy.has_param(source_urdf_param):
                raise RuntimeError(
                    "Missing source URDF parameter {}.".format(source_urdf_param)
                )
            return ET.fromstring(rospy.get_param(source_urdf_param))
        return ET.parse(self._source_urdf_path).getroot()

    def _validate_planning_fixed_joints(self, source_robot):
        """Reject a typo before it becomes an incomplete MoveIt state.

        These joints are fixed only in the private planning URDF.  They are
        deliberately *not* published to the real robot's /joint_states.
        """
        if not self._planning_fixed_joint_positions:
            return True, ""
        source_joint_names = {
            joint.get("name") for joint in source_robot.findall("joint")
        }
        missing = sorted(
            set(self._planning_fixed_joint_positions) - source_joint_names
        )
        if missing:
            return False, (
                "Planning-only fixed joints are absent from the source URDF: {}."
                .format(", ".join(missing))
            )
        return True, ""

    def _build_robot_description(self, side):
        source_robot = self._load_source_robot()
        joint_name = self._anchor_joint_names[side]
        joint_value = self._joint_state_cache.get_position(joint_name)
        if joint_value is None:
            joint_value = self._anchor_joint_defaults[side]
        anchor_matrix, pose, rpy = self._query_anchor_pose(side)
        fixed_joint_positions = dict(self._planning_fixed_joint_positions)
        fixed_joint_positions[joint_name] = joint_value
        target_robot = self._anchored_urdf.build_rerooted_robot(
            source_robot,
            self._anchor_links[side],
            fixed_joint_positions,
            anchor_matrix=anchor_matrix,
            inverse_joint_axis_signs=self._inverse_joint_axis_signs,
        )
        target_joint_types = {
            joint.get("name"): joint.get("type", "fixed")
            for joint in target_robot.findall("joint")
        }
        not_fixed = sorted(
            name for name in fixed_joint_positions
            if name != joint_name
            if target_joint_types.get(name) != "fixed"
        )
        if not_fixed:
            raise RuntimeError(
                "Failed to freeze planning-only joints in generated URDF: {}."
                .format(", ".join(not_fixed))
            )
        xml_text = ET.tostring(target_robot, encoding="unicode")
        return xml_text, joint_value, pose, rpy

    def _launch_node(self, package, node_type, name, args="", output="screen"):
        cmd = ["rosrun", package, node_type, "__name:={}".format(name)]
        if args:
            cmd.extend(shlex.split(args))
        stdout = None if output == "screen" else subprocess.DEVNULL
        stderr = None if output == "screen" else subprocess.DEVNULL
        process = subprocess.Popen(cmd, stdout=stdout, stderr=stderr)
        self._processes[name] = process
        return process

    def _stop_runtime_processes(self):
        had_synced_side = self._synced_side
        for name in list(self._processes.keys())[::-1]:
            process = self._processes.pop(name)
            try:
                process.terminate()
                process.wait(timeout=3.0)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
        self._synced_side = None
        # Do not leave the TF bridge latched to a support that this runtime no
        # longer considers valid.  An empty value makes it fall back to the
        # current attach-state topics (or wait for a new one in auto mode).
        if had_synced_side is not None:
            self._active_anchor_pub.publish(String(data=""))

    def _start_runtime_processes(self, side):
        if self._start_filtered_joint_states:
            self._launch_node(
                "gp11",
                "gp11_filtered_joint_state_bridge.py",
                "filtered_joint_states",
                args=(
                    "_robot_description_param:=robot_description "
                    "_source_topic:=/joint_states "
                    "_output_topic:={}".format(self._filtered_joint_states_topic)
                ),
            )
        else:
            rospy.loginfo("Skipping filtered_joint_state_bridge (using B29 /joint_states directly)")
        if self._start_robot_state_publisher:
            self._launch_node(
                "robot_state_publisher",
                "robot_state_publisher",
                "robot_state_publisher",
                args="joint_states:={}".format(self._filtered_joint_states_topic),
            )
        else:
            rospy.loginfo("Skipping robot_state_publisher (using B29 TF tree)")
        if self._start_trajectory_bridge:
            self._launch_node("gp11", "gp11_moveit_trajectory_bridge.py", "moveit_trajectory_bridge")
        else:
            rospy.loginfo("Skipping trajectory_bridge (start_trajectory_bridge=false)")
        if self._load_support_bar:
            self._launch_node(
                "gp11",
                "gp11_moveit_scene_loader.py",
                "moveit_support_bar_scene",
                args=(
                    "_object_id:={object_id} _frame_id:={frame_id} _length:={length} "
                    "_radius:={radius} _x:={x} _y:={y} _z:={z} _roll:={roll} "
                    "_pitch:={pitch} _yaw:={yaw}"
                ).format(**self._support_bar_params),
            )
        moveit_joint_states_topic = (
            self._filtered_joint_states_topic
            if self._start_filtered_joint_states
            else self._joint_states_topic
        )
        move_group_args = "joint_states:={}".format(moveit_joint_states_topic)
        if self._move_group_namespace:
            move_group_args += " __ns:={}".format(self._move_group_namespace.lstrip("/"))
        self._launch_node(
            "moveit_ros_move_group",
            "move_group",
            "move_group",
            args=move_group_args,
        )
        return moveit_joint_states_topic

    def _start_post_ready_processes(self, side, moveit_joint_states_topic):
        if self._launch_point_target:
            tip_link = "left_gripper_tool" if side == "right" else "right_gripper_tool"
            self._launch_node(
                "gp11",
                "gp11_moveit_point_target.py",
                "moveit_point_target",
                args=(
                    "_group_name:=reach_arm "
                    "_robot_description_param:={robot_description_param} "
                    "_joint_states_topic:={joint_states_topic} "
                    "_ik_service:={ik_service} "
                    "_target_topic:=/gp11_moveit/reach_arm/target_point "
                    "_target_pose_topic:=/gp11_moveit/reach_arm/target_pose "
                    "_status_topic:=/gp11_moveit/reach_arm/status "
                    "_tip_link:={tip_link}"
                ).format(
                    robot_description_param="{}/robot_description".format(
                        self._moveit_param_namespace
                    ) if self._moveit_param_namespace else "robot_description",
                    ik_service="{}/compute_ik".format(
                        self._move_group_namespace
                    ) if self._move_group_namespace else "/compute_ik",
                    joint_states_topic=moveit_joint_states_topic,
                    tip_link=tip_link,
                ),
            )
        if self._rviz_enabled:
            if self._rviz_start_delay:
                rospy.loginfo(
                    "MoveIt action server is ready; waiting %.2fs before RViz marker initialization.",
                    self._rviz_start_delay,
                )
                rospy.sleep(self._rviz_start_delay)
            rviz_args = "-d {}".format(self._rviz_config) if self._rviz_config else ""
            # Keep RViz with the private planning model so RobotInteraction is
            # initialized once and its draggable goal marker remains stable.
            # MoveIt 1.1.16 resolves several MotionPlanning endpoints relative
            # to the RViz node even when its UI namespace says root, so remap
            # those endpoints explicitly back to the Gazebo-style global API.
            if self._moveit_param_namespace:
                rviz_ns = self._moveit_param_namespace.rstrip("/")
                rviz_args += " __ns:={}".format(rviz_ns.lstrip("/"))
                endpoint_remaps = {
                    rviz_ns + "/get_planning_scene": "/get_planning_scene",
                    rviz_ns + "/query_planner_interface": "/query_planner_interface",
                    rviz_ns + "/compute_cartesian_path": "/compute_cartesian_path",
                    rviz_ns + "/plan_kinematic_path": "/plan_kinematic_path",
                    rviz_ns + "/compute_ik": "/compute_ik",
                }
                # MoveGroupInterface constructs all of these action clients
                # when the panel is initialized, even when the operator only
                # intends to use Plan.  Remap the complete action protocol for
                # each one; otherwise initialization advances past move_group
                # and then waits 30 s on namespaced pickup/place.
                for action_name in (
                    "move_group",
                    "pickup",
                    "place",
                    "execute_trajectory",
                ):
                    private_base = rviz_ns + "/" + action_name
                    global_base = "/" + action_name
                    endpoint_remaps[private_base] = global_base
                    for suffix in ("goal", "cancel", "status", "feedback", "result"):
                        endpoint_remaps[private_base + "/" + suffix] = (
                            global_base + "/" + suffix
                        )
                for source, target in endpoint_remaps.items():
                    rviz_args += " {}:={}".format(source, target)
            self._launch_node(
                "rviz",
                "rviz",
                "gp11_moveit_rviz",
                args=rviz_args,
            )

    def _validate_joint_states(self):
        missing, reason = self._joint_state_cache.missing_or_stale(
            self._required_joint_names, self._joint_state_timeout
        )
        if reason:
            return False, (
                "Required joint states unavailable on {}: {}."
                .format(self._joint_states_topic, reason)
            )
        if missing:
            return False, "Missing required joint states on {}: {}.".format(
                self._joint_states_topic, ", ".join(missing)
            )
        return True, ""

    def _validate_source_urdf(self):
        try:
            source_robot = self._load_source_robot()
        except (ET.ParseError, OSError, RuntimeError, TypeError) as exc:
            return False, "Cannot load source URDF: {}".format(exc)
        return self._validate_planning_fixed_joints(source_robot)

    def _wait_for_move_group(self):
        service_name = "{}/get_planning_scene".format(
            self._move_group_namespace
        ) if self._move_group_namespace else "/get_planning_scene"
        try:
            rospy.wait_for_service(
                service_name, timeout=self._move_group_ready_timeout
            )
        except rospy.ROSException:
            return False, "MoveIt service {} unavailable after {:.1f}s.".format(
                service_name, self._move_group_ready_timeout
            )
        # The service only proves that MoveIt was spawned.  The action server
        # comes up after the robot model, planning scene monitor and pipeline
        # are stable; start RViz only after this point.
        action_name = "{}/move_group".format(
            self._move_group_namespace
        ) if self._move_group_namespace else "/move_group"
        client = actionlib.SimpleActionClient(action_name, MoveGroupAction)
        if not client.wait_for_server(rospy.Duration(self._move_group_ready_timeout)):
            return False, (
                "MoveIt action server {} unavailable after {:.1f}s."
                .format(action_name, self._move_group_ready_timeout)
            )
        return True, ""

    def _run_sync(self, manual=False):
        if not self._sync_lock.acquire(False):
            return False, "MoveIt anchor synchronization is already in progress."
        try:
            if manual:
                with self._lock:
                    self._pending_sync = True
                    self._next_retry_wall_time = 0.0
                    self._last_failure_reason = ""
            ok, message = self._sync_once()
            with self._lock:
                if ok:
                    self._pending_sync = False
                    self._next_retry_wall_time = 0.0
                    self._last_failure_reason = ""
                elif not message.startswith((
                    "Waiting for support pose to settle",
                    "Waiting for B29 TF",
                )):
                    self._pending_sync = False
                    self._last_failure_reason = message
                    self._next_retry_wall_time = time.time() + self._failed_retry_delay
            return ok, message
        finally:
            self._sync_lock.release()

    def _sync_once(self):
        valid, reason = self._validate_source_urdf()
        if not valid:
            self._stop_runtime_processes()
            self._publish_status(reason)
            return False, reason

        valid, reason = self._validate_joint_states()
        if not valid:
            self._stop_runtime_processes()
            self._publish_status(reason)
            return False, reason

        side, reason = self._selected_side()
        if side is None:
            self._stop_runtime_processes()
            self._publish_status(reason)
            return False, reason

        if self._anchor_pose_provider_name in ("real_robot", "real"):
            anchor_link = self._real_anchor_links[side]
            try:
                self._real_tf_buffer.lookup_transform(
                    self._real_base_frame,
                    anchor_link,
                    rospy.Time(0),
                    rospy.Duration(self._joint_state_timeout),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                rospy.ROSException,
            ) as exc:
                reason = "Waiting for B29 TF {} -> {}: {}".format(
                    self._real_base_frame, anchor_link, exc
                )
                self._stop_runtime_processes()
                self._publish_status(reason)
                return False, reason

        if time.time() - self._last_change_wall_time < self._settle_time:
            return False, "Waiting for support pose to settle."

        xml_text, joint_value, pose, rpy = self._build_robot_description(side)
        # Stop consumers before replacing their robot-description parameters.
        # This prevents a brief mixed old-process/new-model RViz frame during
        # an anchor re-sync.
        self._stop_runtime_processes()
        self._apply_moveit_parameters(side, xml_text)
        moveit_joint_states_topic = self._start_runtime_processes(side)
        ready, ready_reason = self._wait_for_move_group()
        if not ready:
            self._stop_runtime_processes()
            self._publish_status(ready_reason)
            return False, ready_reason
        self._start_post_ready_processes(side, moveit_joint_states_topic)
        self._synced_side = side
        self._active_anchor_pub.publish(String(data=side))
        message = (
            "MoveIt runtime synced to {side} anchor: "
            "link_pose=({x:.4f}, {y:.4f}, {z:.4f}; {r:.4f}, {p:.4f}, {yy:.4f}), "
            "{joint}={value:.6f}"
        ).format(
            side=side,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            r=rpy[0],
            p=rpy[1],
            yy=rpy[2],
            joint=self._anchor_joint_names[side],
            value=joint_value,
        )
        self._publish_status(message)
        return True, message

    def _sync_service_cb(self, _req):
        try:
            ok, message = self._run_sync(manual=True)
            return TriggerResponse(success=ok, message=message)
        except Exception as exc:
            message = "Sync failed: {}".format(exc)
            rospy.logwarn(message)
            return TriggerResponse(success=False, message=message)

    def _worker_loop(self):
        while not rospy.is_shutdown():
            self._iteration_once()
            time.sleep(0.2)

    def _iteration_once(self):
        self._refresh_attach_state()
        with self._lock:
            pending_sync = self._pending_sync
            synced_side = self._synced_side
        side, reason = self._selected_side()
        if side is None:
            if synced_side is not None:
                self._stop_runtime_processes()
                self._publish_status(reason)
            return
        if not pending_sync and synced_side == side:
            return
        if time.time() < self._next_retry_wall_time:
            return
        try:
            ok, message = self._run_sync()
        except Exception as exc:
            ok = False
            message = "Sync failed: {}".format(exc)
            rospy.logwarn(message)
            self._status_pub.publish(String(data=message))
        if ok:
            with self._lock:
                self._pending_sync = False

    def _shutdown(self):
        self._stop_runtime_processes()


def main():
    rospy.init_node("gp11_moveit_anchor_runtime")
    MoveItAnchorRuntime()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
