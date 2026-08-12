#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from dls_reach_controller import DLSReachController  # noqa: E402
from reach_policy import Config, SafetyLimiter, load_fk_model  # noqa: E402


class _LinearKinematics:
    def evaluate(self, q):
        q = np.asarray(q, dtype=np.float32)
        return q[:3].copy(), np.asarray((1.0, 0.0, 0.0), dtype=np.float32)

    def relative_x_axis(self, q, *, reference_body, axis_body=None):
        _ = q, reference_body, axis_body
        return np.asarray((1.0, 0.0, 0.0), dtype=np.float32)


class DLSReachControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = PACKAGE_DIR / "config" / "reach_rl_config.yaml"

    def setUp(self):
        self.cfg = Config.load(self.config_path)
        self.controller = DLSReachController(
            kinematics=_LinearKinematics(),
            runtime=self.cfg.runtime,
            obs_ref_origin_root=np.zeros(3, dtype=np.float32),
            obs_ref_rot_root=np.eye(3, dtype=np.float32),
            params=self.cfg.deploy.dls,
        )

    def tearDown(self):
        self.controller.close()

    def test_reachable_goal_reduces_cartesian_error(self):
        q = np.zeros(4, dtype=np.float32)
        goal = np.asarray((0.4, -0.3, 0.2), dtype=np.float32)
        self.controller.set_goal(
            goal, np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
            q_start=q, known_reachable=False,
        )

        target = self.controller.compute(q, np.zeros(4, dtype=np.float32), goal,
                                         np.asarray((1.0, 0.0, 0.0), dtype=np.float32))

        self.assertLess(np.linalg.norm(goal - target[:3]), np.linalg.norm(goal - q[:3]))

    def test_target_lead_and_joint_margin_are_enforced(self):
        q = np.zeros(4, dtype=np.float32)
        goal = np.asarray((10.0, 10.0, 10.0), dtype=np.float32)
        self.controller.set_goal(
            goal, np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
            q_start=q, known_reachable=False,
        )
        target = q.copy()
        for _ in range(20):
            target = self.controller.compute(
                q, np.zeros(4, dtype=np.float32), goal,
                np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
            )
        max_lead = 0.8 * self.cfg.runtime.effort_limits / self.cfg.runtime.stiffness
        np.testing.assert_array_less(np.abs(target - q), max_lead + 1.0e-6)
        self.assertTrue(np.all(target <= self.cfg.runtime.hard_upper - 0.05 + 1.0e-6))
        self.assertTrue(np.all(target >= self.cfg.runtime.hard_lower + 0.05 - 1.0e-6))

    def test_velocity_limit_can_match_hardware_safety_rate(self):
        hardware_cfg = Config.load(self.config_path, platform="hardware")
        slow = DLSReachController(
            kinematics=_LinearKinematics(),
            runtime=hardware_cfg.runtime,
            obs_ref_origin_root=np.zeros(3, dtype=np.float32),
            obs_ref_rot_root=np.eye(3, dtype=np.float32),
            params=hardware_cfg.deploy.dls,
            velocity_limits=np.full(
                4,
                hardware_cfg.deploy.safety.max_joint_vel,
                dtype=np.float32,
            ),
        )
        try:
            q = np.zeros(4, dtype=np.float32)
            goal = np.asarray((10.0, 10.0, 10.0), dtype=np.float32)
            slow.set_goal(
                goal, np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
                q_start=q, known_reachable=False,
            )
            target = slow.compute(
                q, np.zeros(4, dtype=np.float32), goal,
                np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
            )
            np.testing.assert_allclose(
                target,
                np.asarray((0.004, 0.004, 0.004, 0.0), dtype=np.float32),
                atol=1.0e-6,
            )
            slow.sync_applied_target(np.zeros(4, dtype=np.float32))
            np.testing.assert_allclose(slow._target_q, 0.0, atol=1.0e-6)
        finally:
            slow.close()

    def test_global_ik_projects_ood_goal_into_operating_range(self):
        q = np.zeros(4, dtype=np.float32)
        goal = np.asarray((2.0, 0.1, -0.2), dtype=np.float32)
        self.controller.set_goal(
            goal, np.asarray((1.0, 0.0, 0.0), dtype=np.float32), q_start=q
        )
        deadline = time.monotonic() + 3.0
        while not self.controller._plan_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.controller._accept_completed_plan()

        self.assertIsNotNone(self.controller._planned_q)
        self.assertAlmostEqual(
            float(self.controller._projected_goal_point[0]),
            float(self.cfg.runtime.sample_upper[0]),
            places=3,
        )

    def test_newer_planner_generation_wins(self):
        def fake_solver(generation, q_start, goal, axis):
            if goal[0] < 0.15:
                time.sleep(0.05)
            planned = np.full(4, goal[0], dtype=np.float32)
            return generation, planned, goal.copy(), axis.copy()

        self.controller._solve_global_ik = fake_solver
        q = np.zeros(4, dtype=np.float32)
        axis = np.asarray((1.0, 0.0, 0.0), dtype=np.float32)
        self.controller.set_goal(np.asarray((0.1, 0.0, 0.0), dtype=np.float32), axis, q_start=q)
        self.controller.set_goal(np.asarray((0.2, 0.0, 0.0), dtype=np.float32), axis, q_start=q)
        deadline = time.monotonic() + 1.0
        while not self.controller._plan_future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.controller._accept_completed_plan()

        np.testing.assert_allclose(self.controller._planned_q, 0.2, atol=1.0e-6)


class DLSDeploymentContractTest(unittest.TestCase):
    def test_platform_selects_safety_profile(self):
        path = PACKAGE_DIR / "config" / "reach_rl_config.yaml"
        gazebo = Config.load(path, platform="gazebo")
        hardware = Config.load(path, platform="hardware")

        self.assertEqual(gazebo.deploy.mode, "dls")
        self.assertEqual(gazebo.deploy.safety.max_joint_vel, 4.0)
        self.assertEqual(hardware.deploy.safety.max_joint_vel, 0.10)
        self.assertEqual(hardware.deploy.safety.target_step_clip, 0.5)

    def test_dls_safety_velocity_uses_dls_update_period(self):
        cfg = Config.load(
            PACKAGE_DIR / "config" / "reach_rl_config.yaml", platform="hardware"
        )
        limiter = SafetyLimiter(cfg)
        q = np.zeros(4, dtype=np.float32)

        target = limiter.apply(np.ones(4, dtype=np.float32), q)

        np.testing.assert_allclose(
            target,
            cfg.deploy.safety.max_joint_vel / cfg.deploy.dls.control_rate_hz,
            atol=1.0e-7,
        )

    def test_both_urdfs_expose_active_orientation_axis(self):
        urdf_dir = PACKAGE_DIR / "models" / "gp11_urdf"
        for anchor_side in ("left", "right"):
            with self.subTest(anchor_side=anchor_side):
                km, _, ref_rot, _, _ = load_fk_model(anchor_side, urdf_dir)
                self.assertIsNotNone(km)
                active_side = "right" if anchor_side == "left" else "left"
                self.assertEqual(km.orientation_body, f"{active_side}_second_leg")
                _, axis_root = km.evaluate(np.zeros(4, dtype=np.float32))
                np.testing.assert_allclose(
                    ref_rot.T @ axis_root,
                    np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
                    atol=1.0e-5,
                )


if __name__ == "__main__":
    unittest.main()
