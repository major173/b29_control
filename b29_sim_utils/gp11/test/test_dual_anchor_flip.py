#!/usr/bin/env python3
"""Offline checks for the mirrored, two-anchor commissioned flip rules."""

import math
import os
import sys
import threading
import time
import types
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(PACKAGE_ROOT, "src"))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from gp11.cartesian_goal_core import (  # noqa: E402
    DEFAULT_JOINT_POSITION_BOUNDS,
    REACH_JOINTS,
    commissioned_free_arm_second_goal,
    commissioned_large_flip_direction,
    commissioned_large_flip_joints,
    build_move_group_goal,
    direction_error,
    large_flip_ik_seed_positions,
    large_flip_override_expected,
    unwrap_direction_goal,
)
from gp11.single_flip_client import (  # noqa: E402
    DEFAULT_REVERSE_TARGET_Z,
    DEFAULT_TARGET_Z,
    SingleFlipClient,
    obstacle_cycle_complete_wait_matches,
    obstacle_cycle_idle_reset_matches,
    target_profile_for_first_crossing_side,
)
from gp11_automatic_flip import run_obstacle_cycle  # noqa: E402


def cycle_trace(stage, side="None", first_side="Right", reason=""):
    return types.SimpleNamespace(
        current_state="Traversing",
        first_crossing_side=first_side,
        grip_fault=False,
        joint_fault=False,
        last_failure_reason="",
        lower_alive=True,
        manual_intervention_reason="",
        obstacle_crossing_active=(stage != "Idle"),
        obstacle_crossing_side=side,
        obstacle_crossing_stage=stage,
        obstacle_crossing_transition_reason=reason,
        software_emergency_stop_latched=False,
    )


class FakeFlipClient(object):
    def __init__(self, sides, results=None):
        self._sides = list(sides)
        self._results = list(results or [0] * len(sides))
        self.wait_expected_sides = []
        self.run_sides = []
        self.run_first_sides = []
        self.last_failure = ""

    def wait_for_disconnect_side(self, timeout=None, expected_side=None):
        self.wait_expected_sides.append(expected_side)
        if not self._sides:
            return None
        side = self._sides.pop(0)
        if expected_side is not None and side != expected_side:
            return None
        return side

    def run(self, **kwargs):
        self.run_sides.append(kwargs["crossing_side"])
        self.run_first_sides.append(kwargs["first_crossing_side"])
        result = self._results.pop(0)
        if result != 0:
            self.last_failure = "commissioned test failure"
        return result


def trajectory(*positions):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(REACH_JOINTS)
    for index, values in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = list(values)
        point.time_from_start = rospy.Duration(0.1 * (index + 1))
        result.joint_trajectory.points.append(point)
    return result


class DualAnchorFlipTest(unittest.TestCase):
    def setUp(self):
        self.actual = dict((name, 0.0) for name in REACH_JOINTS)

    def test_support_second_and_direction_are_mirrored(self):
        left_goal = dict(self.actual)
        left_goal["left_second_leg_joint"] = math.radians(151.0)
        right_goal = dict(self.actual)
        right_goal["right_second_leg_joint"] = math.radians(151.0)
        self.assertEqual(
            commissioned_large_flip_joints("left", left_goal, self.actual),
            ("left_second_leg_joint",),
        )
        self.assertEqual(
            commissioned_large_flip_joints("right", right_goal, self.actual),
            ("right_second_leg_joint",),
        )
        self.assertEqual(commissioned_large_flip_direction("left"), 1.0)
        self.assertEqual(commissioned_large_flip_direction("right"), 1.0)

    def test_both_locked_arm_ik_branches_are_positive(self):
        actual = dict(self.actual)
        actual["right_second_leg_joint"] = -math.pi
        goals = dict(self.actual)
        goals["left_second_leg_joint"] = -math.pi
        goals["right_second_leg_joint"] = 0.0
        left = unwrap_direction_goal(
            actual, goals, {"left_second_leg_joint": 1.0},
            DEFAULT_JOINT_POSITION_BOUNDS,
        )
        right = unwrap_direction_goal(
            actual, goals, {"right_second_leg_joint": 1.0},
            DEFAULT_JOINT_POSITION_BOUNDS,
        )
        self.assertAlmostEqual(left["left_second_leg_joint"], math.pi)
        self.assertAlmostEqual(right["right_second_leg_joint"], 0.0)

    def test_second_free_arm_goal_is_live_minus_pi(self):
        actual = dict(self.actual)
        actual["left_second_leg_joint"] = 3.05
        name, target = commissioned_free_arm_second_goal(
            actual, "right", right_anchor_delta=-math.pi
        )
        self.assertEqual(name, "left_second_leg_joint")
        self.assertAlmostEqual(target, 3.05 - math.pi)

    def test_reverse_second_pass_right_free_arm_uses_live_minus_pi(self):
        actual = dict(self.actual)
        actual["right_second_leg_joint"] = 3.363018035888672
        name, target = commissioned_free_arm_second_goal(
            actual,
            "left",
            left_anchor_target=-math.pi,
            left_anchor_relative_to_live=True,
        )
        self.assertEqual(name, "right_second_leg_joint")
        self.assertAlmostEqual(
            target,
            actual["right_second_leg_joint"] - math.pi,
        )
        adjusted = unwrap_direction_goal(
            actual,
            {name: target},
            {name: -1.0},
            DEFAULT_JOINT_POSITION_BOUNDS,
        )
        self.assertAlmostEqual(adjusted[name], target)

    def test_ik_seed_keeps_live_first_joints_and_mirrors_second_roles(self):
        left_seed = large_flip_ik_seed_positions(
            self.actual, 0.5, "left", -math.pi
        )
        self.assertAlmostEqual(left_seed["left_second_leg_joint"], 0.5 * math.pi)
        self.assertAlmostEqual(left_seed["right_second_leg_joint"], -math.pi)
        second_actual = dict(self.actual)
        second_actual["left_second_leg_joint"] = 3.05
        second_actual["right_second_leg_joint"] = -math.pi
        right_free_target = 3.05 - math.pi
        right_seed = large_flip_ik_seed_positions(
            second_actual, 0.5, "right", right_free_target
        )
        self.assertAlmostEqual(
            right_seed["right_second_leg_joint"], -0.5 * math.pi
        )
        self.assertAlmostEqual(
            right_seed["left_second_leg_joint"], right_free_target
        )
        for name in ("left_first_leg_joint", "right_first_leg_joint"):
            self.assertAlmostEqual(left_seed[name], self.actual[name])
            self.assertAlmostEqual(right_seed[name], second_actual[name])

    def test_second_pass_requires_locked_positive_and_free_clockwise(self):
        actual = dict(self.actual)
        actual["left_second_leg_joint"] = math.pi
        actual["right_second_leg_joint"] = -math.pi
        commissioned = trajectory(
            (0.0, math.pi, 0.0, -math.pi),
            (0.0, 0.5 * math.pi, 0.0, -0.5 * math.pi),
            (0.0, 0.0, 0.0, 0.0),
        )
        signs = {
            "left_second_leg_joint": -1.0,
            "right_second_leg_joint": 1.0,
        }
        self.assertEqual(direction_error(commissioned, actual, signs), "")
        wrong = trajectory(
            (0.0, math.pi, 0.0, -math.pi),
            (0.0, 0.5 * math.pi, 0.0, -1.5 * math.pi),
        )
        self.assertIn("opposite", direction_error(wrong, actual, signs))

    def test_first_pass_keeps_support_counter_clockwise_and_free_clockwise(self):
        first = trajectory(
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.5 * math.pi, 0.0, -0.5 * math.pi),
            (0.0, math.pi, 0.0, -math.pi),
        )
        signs = {
            "left_second_leg_joint": 1.0,
            "right_second_leg_joint": -1.0,
        }
        self.assertEqual(direction_error(first, self.actual, signs), "")

    def test_move_group_goal_accepts_mixed_second_pass_directions(self):
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.orientation.w = 1.0
        starts = dict(self.actual)
        starts["left_second_leg_joint"] = math.pi
        starts["right_second_leg_joint"] = -math.pi
        goals = dict(starts)
        goals["left_second_leg_joint"] = 0.0
        goals["right_second_leg_joint"] = 0.0
        signs = {
            "left_second_leg_joint": -1.0,
            "right_second_leg_joint": 1.0,
        }
        goal = build_move_group_goal(
            target, "left_gripper_tool", "reach_arm",
            (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0),
            0.008, 2, 4.0, 0.2, 0.15,
            goal_joint_positions=goals,
            direction_signs=signs,
            direction_start_positions=starts,
        )
        constraints = goal.request.path_constraints.joint_constraints
        self.assertEqual(
            [constraint.joint_name for constraint in constraints],
            ["left_second_leg_joint", "right_second_leg_joint"],
        )
        self.assertTrue(all(
            constraint.tolerance_above > 1.5 and
            constraint.tolerance_below > 1.5
            for constraint in constraints
        ))

    def test_large_flip_prediction_uses_active_anchor_side(self):
        left = trajectory((0.0, math.radians(151.0), 0.0, 0.0))
        right_actual = dict(self.actual)
        right_actual["right_second_leg_joint"] = -math.pi
        right = trajectory((0.0, 0.0, 0.0, 0.0))
        self.assertTrue(large_flip_override_expected("left", left, self.actual))
        self.assertFalse(large_flip_override_expected("right", left, self.actual))
        self.assertTrue(
            large_flip_override_expected("right", right, right_actual)
        )
        self.assertFalse(
            large_flip_override_expected("left", right, right_actual)
        )

    def test_each_obstacle_selects_its_first_side_independently(self):
        client = FakeFlipClient(("Right", "Left", "Left", "Right"))
        first = run_obstacle_cycle(client, 1, 2, 150.0, 5.0)
        second = run_obstacle_cycle(client, 2, 2, 150.0, 5.0)
        self.assertEqual(first, (0, "", "Right", "Left"))
        self.assertEqual(second, (0, "", "Left", "Right"))
        self.assertEqual(
            client.wait_expected_sides,
            [None, "Left", None, "Right"],
        )
        self.assertEqual(
            client.run_sides,
            ["Right", "Left", "Left", "Right"],
        )
        self.assertEqual(
            client.run_first_sides,
            ["Right", "Right", "Left", "Left"],
        )

    def test_reverse_cycle_uses_a_mirrored_prediction_for_both_passes(self):
        self.assertEqual(
            target_profile_for_first_crossing_side("Right"),
            "default_forward",
        )
        self.assertEqual(
            target_profile_for_first_crossing_side("Left"),
            "reverse",
        )
        self.assertAlmostEqual(
            DEFAULT_REVERSE_TARGET_Z,
            -DEFAULT_TARGET_Z,
        )

        client = FakeFlipClient(("Left", "Right"))
        result = run_obstacle_cycle(client, 1, 2, 150.0, 5.0)
        self.assertEqual(result, (0, "", "Left", "Right"))
        self.assertEqual(client.run_sides, ["Left", "Right"])
        self.assertEqual(client.run_first_sides, ["Left", "Left"])

    def test_failed_pass_stops_the_current_obstacle_without_retry(self):
        client = FakeFlipClient(("Right", "Left", "Right"), (0, 11, 0))
        result = run_obstacle_cycle(client, 1, 2, 150.0, 5.0)
        self.assertEqual(
            result,
            (11, "commissioned test failure", "Right", "Left"),
        )
        self.assertEqual(client.run_sides, ["Right", "Left"])
        self.assertEqual(client.wait_expected_sides, [None, "Left"])

    def test_cycle_reset_requires_complete_wait_before_falling_edge_idle(self):
        startup_idle = cycle_trace("Idle", reason="crossing_reset")
        complete_wait = cycle_trace(
            "CompleteWaitObstacleClear",
            reason="both_sides_completed",
        )
        falling_edge_idle = cycle_trace(
            "Idle",
            reason="obstacle_trigger_falling_edge",
        )
        self.assertFalse(obstacle_cycle_complete_wait_matches(
            startup_idle, "Right"
        ))
        self.assertFalse(obstacle_cycle_idle_reset_matches(startup_idle))
        self.assertTrue(obstacle_cycle_complete_wait_matches(
            complete_wait, "Right"
        ))
        self.assertTrue(obstacle_cycle_idle_reset_matches(falling_edge_idle))

    def test_cycle_reset_wait_does_not_finish_without_obstacle_falling_edge(self):
        client = SingleFlipClient.__new__(SingleFlipClient)
        client._condition = threading.Condition()
        client._trace = cycle_trace(
            "CompleteWaitObstacleClear",
            reason="both_sides_completed",
        )
        client._last_failure = ""
        self.assertFalse(client.wait_for_obstacle_cycle_reset(
            "Right", "Left", timeout=0.02
        ))
        self.assertIn("Idle reset", client.last_failure)

    def test_cycle_reset_wait_rejects_an_already_idle_startup_trace(self):
        client = SingleFlipClient.__new__(SingleFlipClient)
        client._condition = threading.Condition()
        client._trace = cycle_trace("Idle", reason="crossing_reset")
        client._last_failure = ""
        self.assertFalse(client.wait_for_obstacle_cycle_reset(
            "Right", "Left", timeout=0.2
        ))
        self.assertIn("before the normal two-pass", client.last_failure)

    def test_cycle_reset_wait_accepts_only_subsequent_normal_idle(self):
        client = SingleFlipClient.__new__(SingleFlipClient)
        client._condition = threading.Condition()
        client._trace = cycle_trace(
            "CompleteWaitObstacleClear",
            reason="both_sides_completed",
        )
        client._last_failure = ""

        def publish_idle_reset():
            time.sleep(0.02)
            with client._condition:
                client._trace = cycle_trace(
                    "Idle", reason="obstacle_trigger_falling_edge"
                )
                client._condition.notify_all()

        publisher = threading.Thread(target=publish_idle_reset)
        publisher.start()
        try:
            self.assertTrue(client.wait_for_obstacle_cycle_reset(
                "Right", "Left", timeout=0.2
            ))
        finally:
            publisher.join()


if __name__ == "__main__":
    unittest.main()
