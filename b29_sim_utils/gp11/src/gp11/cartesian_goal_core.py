"""Shared planning and safety checks for GP11 Cartesian position goals."""

import copy
import math
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    Constraints,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryGoal,
    JointConstraint,
    MoveGroupAction,
    MoveGroupGoal,
    MoveItErrorCodes,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


REACH_JOINTS = (
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
)

DEFAULT_JOINT_POSITION_BOUNDS = {
    "left_first_leg_joint": (-1.10, 1.10),
    "left_second_leg_joint": (-4.0 * math.pi, 4.0 * math.pi),
    "right_first_leg_joint": (-1.10, 1.10),
    "right_second_leg_joint": (-4.0 * math.pi, 4.0 * math.pi),
}
DEFAULT_MAX_TRAJECTORY_VELOCITY = 1.20
DEFAULT_MAX_TRAJECTORY_ACCELERATION = 1.50
DEFAULT_MAX_TRAJECTORY_POINT_DELTA = 0.10
DEFAULT_MAX_TRAJECTORY_DURATION = 30.0
DEFAULT_MAX_TRAJECTORY_POINTS = 1000
DEFAULT_GOAL_JOINT_DEADBAND = 0.03
DEFAULT_SESSION_REARM_TIMEOUT = 3.0
DEFAULT_LARGE_FLIP_THRESHOLD = math.radians(150.0)
DEFAULT_POSITIVE_DIRECTION_TOLERANCE = math.radians(0.5)


class Stage(object):
    WAITING_STATE = 0
    TRANSFORMING_TARGET = 1
    CHECKING_IK = 2
    PLANNING = 3
    AUDITING_TRAJECTORY = 4
    WAITING_PLANNER_SESSION = 5
    EXECUTING = 6
    WAITING_SESSION_REARM = 7
    SUCCEEDED = 8
    REJECTED = 9
    PREEMPTED = 10
    FAILED = 11

    NAMES = {
        WAITING_STATE: "WAITING_STATE",
        TRANSFORMING_TARGET: "TRANSFORMING_TARGET",
        CHECKING_IK: "CHECKING_IK",
        PLANNING: "PLANNING",
        AUDITING_TRAJECTORY: "AUDITING_TRAJECTORY",
        WAITING_PLANNER_SESSION: "WAITING_PLANNER_SESSION",
        EXECUTING: "EXECUTING",
        WAITING_SESSION_REARM: "WAITING_SESSION_REARM",
        SUCCEEDED: "SUCCEEDED",
        REJECTED: "REJECTED",
        PREEMPTED: "PREEMPTED",
        FAILED: "FAILED",
    }


def finite_point(point):
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def inside_workspace(point, workspace_min, workspace_max):
    if not finite_point(point):
        return False
    return all(
        low <= value <= high
        for value, low, high in zip(
            (point.x, point.y, point.z), workspace_min, workspace_max
        )
    )


def tip_link_for_anchor(anchor_side, left_tip_link, right_tip_link):
    if anchor_side == "left":
        return left_tip_link
    if anchor_side == "right":
        return right_tip_link
    return ""


def joint_error(joint_name, desired, actual):
    difference = desired - actual
    if joint_name.endswith("second_leg_joint"):
        return math.atan2(math.sin(difference), math.cos(difference))
    return difference


def positive_large_flip_joints(
        anchor_side, goal_positions, actual_positions,
        threshold=DEFAULT_LARGE_FLIP_THRESHOLD):
    """Return support-side second joints that must use the positive branch.

    The positive-direction commissioning rule currently applies only while
    the left side is the support anchor.  The free-arm right-second joint does
    not affect the commissioned Cartesian endpoint and is deliberately left
    free to take either approximately-180-degree direction.  Small Cartesian
    motions also remain free to move either way.
    """
    left_second = "left_second_leg_joint"
    if anchor_side != "left":
        return ()
    if (left_second not in goal_positions or
            left_second not in actual_positions or
            abs(joint_error(
                left_second,
                goal_positions[left_second],
                actual_positions[left_second],
            )) <= threshold):
        return ()
    return (left_second,)


def positive_direction_seed_positions(
        actual_positions, goal_positions, joint_names, fraction):
    """Build an IK seed part-way along each joint's positive circular arc."""
    if not math.isfinite(fraction) or not 0.0 < fraction < 1.0:
        raise ValueError("positive direction seed fraction must be in (0, 1)")
    seed = dict(actual_positions)
    for joint_name in joint_names:
        if joint_name not in actual_positions or joint_name not in goal_positions:
            raise ValueError(
                "positive direction seed is missing {}".format(joint_name)
            )
        start = float(actual_positions[joint_name])
        desired = float(goal_positions[joint_name])
        positive_span = (desired - start) % (2.0 * math.pi)
        if positive_span <= 0.0:
            positive_span = 2.0 * math.pi
        seed[joint_name] = start + fraction * positive_span
    return seed


def positive_direction_error(
        trajectory, actual_positions, joint_names,
        tolerance=DEFAULT_POSITIVE_DIRECTION_TOLERANCE):
    """Reject a trajectory that ever decreases a required positive joint."""
    if not math.isfinite(tolerance) or tolerance < 0.0:
        return "positive direction tolerance must be finite and non-negative"
    required = tuple(joint_names)
    if not required:
        return ""
    joint_trajectory = trajectory.joint_trajectory
    if not joint_trajectory.points:
        return "positive direction audit received an empty trajectory"
    indices = {name: index for index, name in enumerate(joint_trajectory.joint_names)}
    for joint_name in required:
        if joint_name not in indices or joint_name not in actual_positions:
            return "positive direction audit is missing {}".format(joint_name)
        highest = float(actual_positions[joint_name])
        for point_index, point in enumerate(joint_trajectory.points):
            if len(point.positions) <= indices[joint_name]:
                return "positive direction audit found a malformed trajectory point"
            current = float(point.positions[indices[joint_name]])
            backtrack = current - highest
            if backtrack < -tolerance:
                return (
                    "{} attempted negative motion at point {}: {:+.6f} rad"
                ).format(joint_name, point_index, backtrack)
            highest = max(highest, current)
        finish = float(
            joint_trajectory.points[-1].positions[indices[joint_name]]
        )
        total_delta = finish - float(actual_positions[joint_name])
        if total_delta <= tolerance:
            return (
                "{} did not finish on the positive branch: total delta {:+.6f} rad"
            ).format(joint_name, total_delta)
    return ""


def planner_control_cancel_reason(
        active, session_id, expected_session_id, last_completed_session_id,
        exit_reason, completed_exit_reason):
    """Classify PlannerControl loss without treating normal completion as a fault."""
    if not expected_session_id:
        return ""
    if active:
        if session_id == expected_session_id:
            return ""
        # Starting the next session resets exit_reason to EXIT_NONE, while
        # last_completed_session_id deliberately retains the completed one.
        if last_completed_session_id == expected_session_id:
            return ""
        return (
            "PlannerControl session changed from {} to {} without confirming completion"
        ).format(expected_session_id, session_id)
    if (last_completed_session_id == expected_session_id
            and exit_reason == completed_exit_reason):
        return ""
    return "PlannerControl authority revoked (exit_reason={})".format(exit_reason)


def robot_state_from_positions(joint_positions):
    state = RobotState()
    state.joint_state.name = [
        name for name in REACH_JOINTS if name in joint_positions
    ]
    state.joint_state.position = [
        joint_positions[name] for name in state.joint_state.name
    ]
    state.is_diff = True
    return state


def reach_joint_positions_from_state(state):
    """Extract one complete, finite reach-arm solution from a RobotState."""
    positions = dict(zip(state.joint_state.name, state.joint_state.position))
    missing = [name for name in REACH_JOINTS if name not in positions]
    if missing:
        raise ValueError(
            "IK solution is missing reach-arm joints: {}".format(
                ", ".join(missing)
            )
        )
    result = {name: float(positions[name]) for name in REACH_JOINTS}
    non_finite = [name for name, value in result.items() if not math.isfinite(value)]
    if non_finite:
        raise ValueError(
            "IK solution contains non-finite joints: {}".format(
                ", ".join(non_finite)
            )
        )
    return result


def build_ik_request(group_name, tip_link, target, joint_positions, timeout):
    request = GetPositionIKRequest()
    request.ik_request.group_name = group_name
    request.ik_request.ik_link_name = tip_link
    request.ik_request.pose_stamped = copy.deepcopy(target)
    request.ik_request.robot_state = robot_state_from_positions(joint_positions)
    request.ik_request.timeout = rospy.Duration(timeout)
    request.ik_request.avoid_collisions = True
    return request


def build_move_group_goal(
        target, tip_link, group_name, workspace_min, workspace_max,
        position_tolerance, planning_attempts, planning_time,
        velocity_scaling, acceleration_scaling, goal_joint_positions=None,
        goal_joint_tolerance=0.005, positive_direction_joints=(),
        positive_direction_start_positions=None,
        positive_direction_constraint_margin=DEFAULT_POSITIVE_DIRECTION_TOLERANCE):
    goal = MoveGroupGoal()
    request = goal.request
    request.group_name = group_name
    request.num_planning_attempts = planning_attempts
    request.allowed_planning_time = planning_time
    request.max_velocity_scaling_factor = velocity_scaling
    request.max_acceleration_scaling_factor = acceleration_scaling
    request.start_state.is_diff = True
    request.workspace_parameters.header.frame_id = target.header.frame_id
    request.workspace_parameters.min_corner.x = workspace_min[0]
    request.workspace_parameters.min_corner.y = workspace_min[1]
    request.workspace_parameters.min_corner.z = workspace_min[2]
    request.workspace_parameters.max_corner.x = workspace_max[0]
    request.workspace_parameters.max_corner.y = workspace_max[1]
    request.workspace_parameters.max_corner.z = workspace_max[2]

    constraint = Constraints()
    if goal_joint_positions is None:
        position = PositionConstraint()
        position.header.frame_id = target.header.frame_id
        position.link_name = tip_link
        position.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [position_tolerance]
        position.constraint_region.primitives.append(region)
        position.constraint_region.primitive_poses.append(copy.deepcopy(target.pose))
        constraint.position_constraints.append(position)
    else:
        if not math.isfinite(goal_joint_tolerance) or goal_joint_tolerance <= 0.0:
            raise ValueError("goal_joint_tolerance must be positive and finite")
        for joint_name in REACH_JOINTS:
            if joint_name not in goal_joint_positions:
                raise ValueError("goal joint positions are missing {}".format(joint_name))
            joint_position = float(goal_joint_positions[joint_name])
            if not math.isfinite(joint_position):
                raise ValueError("goal joint position for {} is not finite".format(joint_name))
            joint = JointConstraint()
            joint.joint_name = joint_name
            joint.position = joint_position
            joint.tolerance_above = goal_joint_tolerance
            joint.tolerance_below = goal_joint_tolerance
            joint.weight = 1.0
            constraint.joint_constraints.append(joint)
    request.goal_constraints.append(constraint)

    positive_direction_joints = tuple(positive_direction_joints)
    if positive_direction_joints:
        if goal_joint_positions is None or positive_direction_start_positions is None:
            raise ValueError(
                "positive direction constraints require joint start and goal positions"
            )
        if (not math.isfinite(positive_direction_constraint_margin) or
                positive_direction_constraint_margin < 0.0):
            raise ValueError(
                "positive direction constraint margin must be finite and non-negative"
            )
        path_constraint = Constraints()
        path_constraint.name = "positive_large_flip_direction"
        for joint_name in positive_direction_joints:
            if (joint_name not in positive_direction_start_positions or
                    joint_name not in goal_joint_positions):
                raise ValueError(
                    "positive direction constraint is missing {}".format(joint_name)
                )
            start = float(positive_direction_start_positions[joint_name])
            finish = float(goal_joint_positions[joint_name])
            span = finish - start
            if not math.isfinite(start) or not math.isfinite(finish):
                raise ValueError(
                    "positive direction constraint for {} is not finite".format(
                        joint_name
                    )
                )
            if span <= 0.0 or span >= 2.0 * math.pi:
                raise ValueError(
                    "positive direction constraint for {} has invalid span {:+.6f}"
                    .format(joint_name, span)
                )
            joint = JointConstraint()
            joint.joint_name = joint_name
            joint.position = start + 0.5 * span
            joint.tolerance_below = 0.5 * span + positive_direction_constraint_margin
            joint.tolerance_above = 0.5 * span + positive_direction_constraint_margin
            joint.weight = 1.0
            path_constraint.joint_constraints.append(joint)
        request.path_constraints = path_constraint

    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal


def audit_trajectory(
        trajectory, joint_position_bounds=None,
        max_velocity=DEFAULT_MAX_TRAJECTORY_VELOCITY,
        max_acceleration=DEFAULT_MAX_TRAJECTORY_ACCELERATION,
        max_point_delta=DEFAULT_MAX_TRAJECTORY_POINT_DELTA,
        max_duration=DEFAULT_MAX_TRAJECTORY_DURATION,
        max_points=DEFAULT_MAX_TRAJECTORY_POINTS):
    bounds = joint_position_bounds or DEFAULT_JOINT_POSITION_BOUNDS
    joint_trajectory = trajectory.joint_trajectory
    joint_names = tuple(joint_trajectory.joint_names)
    if set(joint_names) != set(REACH_JOINTS) or len(joint_names) != len(REACH_JOINTS):
        return False, "规划结果关节不是唯一的四个 B29 腿关节：{}".format(
            ", ".join(joint_names) or "<empty>"
        )
    multi_dof = trajectory.multi_dof_joint_trajectory
    if multi_dof.joint_names or multi_dof.points:
        return False, "规划结果包含不允许的 multi-DOF 轨迹"
    if not joint_trajectory.points:
        return False, "规划结果没有轨迹点"
    if len(joint_trajectory.points) > max_points:
        return False, "轨迹点数超过安全上限 {}".format(max_points)

    previous_time = None
    previous_positions = None
    for point in joint_trajectory.points:
        if len(point.positions) != len(joint_names):
            return False, "轨迹点维度与关节名称不一致"
        if any(not math.isfinite(value) for value in point.positions):
            return False, "轨迹包含非有限关节目标"
        point_time = point.time_from_start.to_sec()
        if not math.isfinite(point_time) or point_time < 0.0:
            return False, "轨迹时间必须是有限且非负的"
        if previous_time is not None and point_time <= previous_time:
            return False, "轨迹时间必须严格递增"
        if point_time > max_duration:
            return False, "轨迹时长超过安全上限 {:.1f} s".format(max_duration)

        if point.velocities:
            if len(point.velocities) != len(joint_names):
                return False, "轨迹速度维度与关节名称不一致"
            if any(not math.isfinite(value) for value in point.velocities):
                return False, "轨迹包含非有限关节速度"
            if any(abs(value) > max_velocity for value in point.velocities):
                return False, "轨迹速度超过 {:.3f} rad/s 安全上限".format(max_velocity)
        if point.accelerations:
            if not point.velocities:
                return False, "轨迹加速度存在但速度缺失"
            if len(point.accelerations) != len(joint_names):
                return False, "轨迹加速度维度与关节名称不一致"
            if any(not math.isfinite(value) for value in point.accelerations):
                return False, "轨迹包含非有限关节加速度"
            if any(abs(value) > max_acceleration for value in point.accelerations):
                return False, "轨迹加速度超过 {:.3f} rad/s^2 安全上限".format(
                    max_acceleration
                )

        positions_by_joint = dict(zip(joint_names, point.positions))
        for joint_name, position in positions_by_joint.items():
            lower, upper = bounds[joint_name]
            if position < lower or position > upper:
                return False, "{} 超出本地位置范围 [{:.3f}, {:.3f}]".format(
                    joint_name, lower, upper
                )
        if previous_positions is not None:
            for joint_name, position in positions_by_joint.items():
                if abs(position - previous_positions[joint_name]) > max_point_delta:
                    return False, "{} 相邻轨迹点跨度超过 {:.3f} rad".format(
                        joint_name, max_point_delta
                    )
        previous_time = point_time
        previous_positions = positions_by_joint
    return True, "四关节轨迹时间/位置/速度/加速度/步长检查通过"


def planned_goal_joint_state(trajectory, stamp=None):
    joint_trajectory = trajectory.joint_trajectory
    state = JointState()
    state.header.stamp = stamp or rospy.Time.now()
    if not joint_trajectory.points:
        return state
    state.name = list(joint_trajectory.joint_names)
    state.position = list(joint_trajectory.points[-1].positions)
    return state


def trajectory_goal_within_deadband(
        trajectory, actual_positions, deadband=DEFAULT_GOAL_JOINT_DEADBAND):
    if not trajectory.joint_trajectory.points:
        return False
    final_point = trajectory.joint_trajectory.points[-1]
    final_positions = dict(zip(trajectory.joint_trajectory.joint_names, final_point.positions))
    if any(name not in actual_positions or name not in final_positions for name in REACH_JOINTS):
        return False
    return all(
        abs(joint_error(name, final_positions[name], actual_positions[name])) <= deadband
        for name in REACH_JOINTS
    )


def large_flip_override_expected(
        anchor_side, trajectory, actual_positions,
        threshold=DEFAULT_LARGE_FLIP_THRESHOLD):
    if anchor_side != "left" or not trajectory.joint_trajectory.points:
        return False
    names = trajectory.joint_trajectory.joint_names
    final_positions = dict(zip(names, trajectory.joint_trajectory.points[-1].positions))
    joint_name = "left_second_leg_joint"
    if joint_name not in actual_positions or joint_name not in final_positions:
        return False
    return abs(joint_error(
        joint_name, final_positions[joint_name], actual_positions[joint_name]
    )) > threshold


class CartesianGoalCore(object):
    """MoveIt clients used by both headless and interactive Cartesian frontends."""

    def __init__(self, move_group_action, execute_action, ik_service):
        self.move_group_action = move_group_action
        self.execute_action = execute_action
        self.ik_service_name = ik_service
        self.move_group_client = actionlib.SimpleActionClient(
            move_group_action, MoveGroupAction
        )
        self.execute_client = actionlib.SimpleActionClient(
            execute_action, ExecuteTrajectoryAction
        )
        self.ik_service = rospy.ServiceProxy(ik_service, GetPositionIK)

    @staticmethod
    def _wait_for_result(client, timeout, cancel_requested):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown():
            reason = cancel_requested() if cancel_requested else ""
            if reason:
                client.cancel_goal()
                return False, reason
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                client.cancel_goal()
                return False, "timeout"
            if client.wait_for_result(rospy.Duration(min(0.1, remaining))):
                return True, ""
        client.cancel_goal()
        return False, "ROS shutdown"

    def cancel_active_operation(self):
        self.move_group_client.cancel_goal()
        self.execute_client.cancel_goal()

    def check_ik(self, target, tip_link, group_name, joint_positions, timeout):
        try:
            rospy.wait_for_service(self.ik_service_name, timeout=0.5)
        except rospy.ROSException:
            return None, "MoveIt IK 服务尚未就绪"
        try:
            response = self.ik_service(build_ik_request(
                group_name, tip_link, target, joint_positions, timeout
            ))
        except rospy.ServiceException as exc:
            return None, "IK 调用失败：{}".format(exc)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return response, "IK/终点碰撞检查失败（MoveIt code {}）".format(
                response.error_code.val
            )
        return response, ""

    def plan_target(self, goal, timeout, cancel_requested):
        if not self.move_group_client.wait_for_server(rospy.Duration(2.0)):
            return None, "MoveGroup Action 未就绪：{}".format(self.move_group_action)
        self.move_group_client.send_goal(goal)
        completed, reason = self._wait_for_result(
            self.move_group_client, timeout, cancel_requested
        )
        if not completed:
            return None, "MoveIt 规划已取消：{}".format(reason)
        result = self.move_group_client.get_result()
        if result is None:
            return None, "MoveIt 规划未返回结果"
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return result, "MoveIt 规划失败（code {}）".format(result.error_code.val)
        return result, ""

    def execute_trajectory(self, trajectory, timeout, cancel_requested):
        if not self.execute_client.wait_for_server(rospy.Duration(2.0)):
            return None, "执行 Action 未就绪：{}".format(self.execute_action)
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = copy.deepcopy(trajectory)
        self.execute_client.send_goal(goal)
        completed, reason = self._wait_for_result(
            self.execute_client, timeout, cancel_requested
        )
        if not completed:
            return None, "MoveIt 执行已取消：{}".format(reason)
        result = self.execute_client.get_result()
        if result is None:
            return None, "MoveIt 执行未返回结果"
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return result, "MoveIt 执行失败（code {}）".format(result.error_code.val)
        return result, ""
