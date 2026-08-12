#!/usr/bin/env python3
"""有界全局 IK 与阻尼最小二乘 Reach 控制器。

本模块不依赖 ROS，输入实测关节状态和 obs_ref 下的目标，输出有界关节位置
目标。全局 IK 在单个后台线程中运行，新目标会使旧规划结果失效。

移植来源：b29_locomotion commit 6ee65d4（Fix global DLS reach planning）。
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from typing import Optional

import numpy as np

from reach_policy import DLSParams, RuntimeParams


def _normalize(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-8:
        return np.zeros_like(values)
    return (values / norm).astype(np.float32, copy=False)


def _axis_error(current_axis: np.ndarray, goal_axis: np.ndarray) -> np.ndarray:
    return np.cross(_normalize(current_axis), _normalize(goal_axis)).astype(
        np.float32, copy=False
    )


def _root_to_ref_point(
    point_root: np.ndarray,
    ref_origin_root: np.ndarray,
    ref_rot_root: np.ndarray,
) -> np.ndarray:
    return (ref_rot_root.T @ (point_root - ref_origin_root)).astype(
        np.float32, copy=False
    )


def _root_to_ref_vector(vector_root: np.ndarray, ref_rot_root: np.ndarray) -> np.ndarray:
    return (ref_rot_root.T @ vector_root).astype(np.float32, copy=False)


class DLSReachController:
    """带有界关节分支规划的有状态 4-DOF 笛卡尔控制器。"""

    def __init__(
        self,
        *,
        kinematics,
        runtime: RuntimeParams,
        obs_ref_origin_root: np.ndarray,
        obs_ref_rot_root: np.ndarray,
        params: DLSParams,
        velocity_limits: Optional[np.ndarray] = None,
    ) -> None:
        self.kinematics = kinematics
        self.const = runtime
        self.obs_ref_origin_root = np.asarray(obs_ref_origin_root, dtype=np.float32)
        self.obs_ref_rot_root = np.asarray(obs_ref_rot_root, dtype=np.float32)
        self.params = params
        self.velocity_limits = np.asarray(
            runtime.velocity_limits if velocity_limits is None else velocity_limits,
            dtype=np.float32,
        ).reshape(4)
        if np.any(self.velocity_limits <= 0.0):
            raise ValueError("DLS velocity_limits must be positive")

        if params.control_rate_hz <= 0.0:
            raise ValueError("DLS control_rate_hz must be positive")
        if params.position_gain <= 0.0 or params.orientation_gain <= 0.0:
            raise ValueError("DLS position and orientation gains must be positive")
        if params.orientation_weight <= 0.0 or params.damping <= 0.0:
            raise ValueError("DLS orientation weight and damping must be positive")
        if params.jacobian_epsilon <= 0.0:
            raise ValueError("DLS jacobian_epsilon must be positive")
        if params.joint_limit_margin < 0.0:
            raise ValueError("DLS joint_limit_margin must be non-negative")
        if params.limit_avoidance_distance <= 0.0:
            raise ValueError("DLS limit_avoidance_distance must be positive")
        if params.limit_avoidance_gain < 0.0:
            raise ValueError("DLS limit_avoidance_gain must be non-negative")
        if not 0.0 < params.max_target_lead_fraction <= 1.0:
            raise ValueError("DLS max_target_lead_fraction must be in (0, 1]")
        if params.joint_plan_gain <= 0.0 or params.joint_plan_blend_distance <= 0.0:
            raise ValueError("DLS joint planning gains must be positive")
        if params.velocity_damping < 0.0:
            raise ValueError("DLS velocity_damping must be non-negative")

        self.command_lower = runtime.hard_lower + params.joint_limit_margin
        self.command_upper = runtime.hard_upper - params.joint_limit_margin
        self.operating_lower = runtime.sample_lower.copy()
        self.operating_upper = runtime.sample_upper.copy()
        if np.any(self.command_lower >= self.command_upper):
            raise ValueError("DLS joint_limit_margin leaves an empty command range")
        if np.any(self.operating_lower < self.command_lower) or np.any(
            self.operating_upper > self.command_upper
        ):
            raise ValueError("DLS operating range must lie inside command range")

        self._target_q: Optional[np.ndarray] = None
        self._goal_point: Optional[np.ndarray] = None
        self._goal_axis: Optional[np.ndarray] = None
        self._projected_goal_point: Optional[np.ndarray] = None
        self._projected_goal_axis: Optional[np.ndarray] = None
        self._planned_q: Optional[np.ndarray] = None
        self._plan_future: Optional[Future] = None
        self._plan_generation = 0
        self._planner = ThreadPoolExecutor(max_workers=1, thread_name_prefix="b29_dls_ik")

        seed_rng = np.random.default_rng(90210)
        self._global_ik_seeds = [np.zeros(4, dtype=np.float32)] + [
            seed_rng.uniform(self.operating_lower, self.operating_upper).astype(np.float32)
            for _ in range(7)
        ]

    def close(self) -> None:
        self._plan_generation += 1
        if self._plan_future is not None:
            self._plan_future.cancel()
            self._plan_future = None
        try:
            self._planner.shutdown(wait=False, cancel_futures=True)
        except TypeError:
            self._planner.shutdown(wait=False)

    def reset(self) -> None:
        self._target_q = None
        self._goal_point = None
        self._goal_axis = None
        self._projected_goal_point = None
        self._projected_goal_axis = None
        self._planned_q = None
        self._plan_generation += 1
        if self._plan_future is not None:
            self._plan_future.cancel()
            self._plan_future = None

    def sync_applied_target(self, target_q: np.ndarray) -> None:
        """同步最后实际下发目标，避免安全限幅后的目标在内部继续积累。"""
        target = np.asarray(target_q, dtype=np.float32).reshape(4)
        self._target_q = np.clip(target, self.command_lower, self.command_upper).copy()

    def set_goal(
        self,
        goal_point_ref: np.ndarray,
        goal_axis_ref: np.ndarray,
        *,
        q_start: np.ndarray,
        known_reachable: bool = True,
    ) -> None:
        goal_point = np.asarray(goal_point_ref, dtype=np.float32).reshape(3).copy()
        goal_axis = _normalize(np.asarray(goal_axis_ref, dtype=np.float32).reshape(3))
        start = np.clip(
            np.asarray(q_start, dtype=np.float32).reshape(4),
            self.operating_lower,
            self.operating_upper,
        )
        self._goal_point = goal_point
        self._goal_axis = goal_axis
        self._projected_goal_point = goal_point.copy()
        self._projected_goal_axis = goal_axis.copy()
        self._planned_q = None
        self._plan_generation += 1
        generation = self._plan_generation
        if self._plan_future is not None:
            self._plan_future.cancel()
            self._plan_future = None
        if known_reachable:
            self._plan_future = self._planner.submit(
                self._solve_global_ik,
                generation,
                start.copy(),
                goal_point.copy(),
                goal_axis.copy(),
            )

    def _global_ik_residual(
        self,
        q: np.ndarray,
        goal_point_ref: np.ndarray,
        goal_axis_ref: np.ndarray,
    ) -> np.ndarray:
        point_ref, axis_ref = self._evaluate_in_ref(np.asarray(q, dtype=np.float32))
        return np.concatenate(
            [
                point_ref - goal_point_ref,
                self.params.orientation_weight * _axis_error(axis_ref, goal_axis_ref),
            ]
        ).astype(np.float64)

    def _solve_global_ik(
        self,
        generation: int,
        q_start: np.ndarray,
        goal_point_ref: np.ndarray,
        goal_axis_ref: np.ndarray,
    ):
        try:
            from scipy.optimize import least_squares
        except ImportError as exc:
            raise RuntimeError(
                "DLS global IK requires scipy; install it in the b29 runtime environment"
            ) from exc

        best_q = None
        best_cost = float("inf")
        for seed in [q_start] + self._global_ik_seeds:
            result = least_squares(
                self._global_ik_residual,
                np.clip(seed, self.operating_lower, self.operating_upper),
                args=(goal_point_ref, goal_axis_ref),
                bounds=(self.operating_lower, self.operating_upper),
                diff_step=1.0e-3,
                max_nfev=100,
                ftol=1.0e-8,
                xtol=1.0e-8,
                gtol=1.0e-8,
            )
            cost = float(result.fun @ result.fun)
            if cost < best_cost:
                best_q = np.asarray(result.x, dtype=np.float32)
                best_cost = cost
            if (
                float(np.linalg.norm(result.fun[:3])) < 1.0e-3
                and float(np.linalg.norm(result.fun[3:]))
                < 1.0e-2 * self.params.orientation_weight
            ):
                break
        if best_q is None:
            raise RuntimeError("DLS global IK produced no candidate")
        point_ref, axis_ref = self._evaluate_in_ref(best_q)
        return generation, best_q, point_ref, axis_ref

    def _accept_completed_plan(self) -> None:
        future = self._plan_future
        if future is None or not future.done():
            return
        self._plan_future = None
        try:
            generation, planned_q, point_ref, axis_ref = future.result()
        except Exception as exc:
            self._planned_q = None
            raise RuntimeError(f"DLS global IK failed: {exc}") from exc
        if generation != self._plan_generation:
            return
        self._planned_q = planned_q
        if self.params.project_changed_goals:
            self._projected_goal_point = np.asarray(point_ref, dtype=np.float32)
            self._projected_goal_axis = _normalize(np.asarray(axis_ref, dtype=np.float32))

    def _evaluate_in_ref(self, q: np.ndarray):
        point_root, axis_root = self.kinematics.evaluate(q)
        point_ref = _root_to_ref_point(
            point_root, self.obs_ref_origin_root, self.obs_ref_rot_root
        )
        if self.params.relative_axis_start_body is None:
            axis_ref = _root_to_ref_vector(axis_root, self.obs_ref_rot_root)
        else:
            axis_ref = self.kinematics.relative_x_axis(
                q,
                reference_body=self.params.relative_axis_start_body,
            )
        return point_ref, _normalize(axis_ref)

    def _jacobian(self, q: np.ndarray, goal_axis_ref: np.ndarray) -> np.ndarray:
        jacobian = np.zeros((6, 4), dtype=np.float64)
        eps = self.params.jacobian_epsilon
        for joint_index in range(4):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[joint_index] += eps
            q_minus[joint_index] -= eps
            point_plus, axis_plus = self._evaluate_in_ref(q_plus)
            point_minus, axis_minus = self._evaluate_in_ref(q_minus)
            jacobian[:3, joint_index] = (point_plus - point_minus) / (2.0 * eps)
            jacobian[3:, joint_index] = (
                _axis_error(axis_plus, goal_axis_ref)
                - _axis_error(axis_minus, goal_axis_ref)
            ) / (2.0 * eps)
        return jacobian

    def _solve_velocity(self, jacobian: np.ndarray, task_velocity: np.ndarray) -> np.ndarray:
        lhs = jacobian @ jacobian.T + (self.params.damping ** 2) * np.eye(6)
        return jacobian.T @ np.linalg.solve(lhs, task_velocity)

    def compute(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        goal_point_ref: np.ndarray,
        goal_axis_ref: np.ndarray,
    ) -> np.ndarray:
        self._accept_completed_plan()
        q = np.asarray(q, dtype=np.float32).reshape(4)
        dq = np.asarray(dq, dtype=np.float32).reshape(4)
        goal_point = self._projected_goal_point
        goal_axis = self._projected_goal_axis
        if goal_point is None or goal_axis is None:
            self.set_goal(goal_point_ref, goal_axis_ref, q_start=q)
            goal_point = self._projected_goal_point
            goal_axis = self._projected_goal_axis
        assert goal_point is not None and goal_axis is not None

        point_ref, axis_ref = self._evaluate_in_ref(q)
        position_error = np.asarray(goal_point - point_ref, dtype=np.float64)
        position_norm = float(np.linalg.norm(position_error))
        if position_norm > self.params.max_position_error:
            position_error *= self.params.max_position_error / position_norm
        orientation_error = np.asarray(_axis_error(axis_ref, goal_axis), dtype=np.float64)
        orientation_norm = float(np.linalg.norm(orientation_error))
        if orientation_norm > self.params.max_orientation_error:
            orientation_error *= self.params.max_orientation_error / orientation_norm

        jacobian = self._jacobian(q, goal_axis)
        row_scale = np.asarray(
            (1.0, 1.0, 1.0,
             self.params.orientation_weight,
             self.params.orientation_weight,
             self.params.orientation_weight),
            dtype=np.float64,
        )
        scaled_jacobian = row_scale[:, None] * jacobian
        task_velocity = np.concatenate(
            [
                self.params.position_gain * position_error,
                -self.params.orientation_gain * orientation_error,
            ]
        )
        if self.params.velocity_damping > 0.0:
            task_velocity -= self.params.velocity_damping * (
                jacobian @ np.asarray(dq, dtype=np.float64)
            )
        task_velocity *= row_scale
        qdot = self._solve_velocity(scaled_jacobian, task_velocity)

        if self._planned_q is not None:
            joint_error = np.asarray(self._planned_q - q, dtype=np.float64)
            joint_qdot = self.params.joint_plan_gain * joint_error
            blend = np.clip(
                float(np.max(np.abs(joint_error))) / self.params.joint_plan_blend_distance,
                0.0,
                1.0,
            )
            qdot = (1.0 - blend) * qdot + blend * joint_qdot

        lower_distance = q - self.command_lower
        upper_distance = self.command_upper - q
        distance = self.params.limit_avoidance_distance
        lower_scale = np.clip(lower_distance / distance, 0.0, 1.0)
        upper_scale = np.clip(upper_distance / distance, 0.0, 1.0)
        qdot = np.where(qdot < 0.0, qdot * lower_scale, qdot)
        qdot = np.where(qdot > 0.0, qdot * upper_scale, qdot)
        if self.params.limit_avoidance_gain > 0.0:
            lower_push = np.clip((distance - lower_distance) / distance, 0.0, 1.0)
            upper_push = np.clip((distance - upper_distance) / distance, 0.0, 1.0)
            qdot += self.params.limit_avoidance_gain * (lower_push - upper_push)

        qdot = np.clip(qdot, -self.velocity_limits, self.velocity_limits)
        control_dt = 1.0 / self.params.control_rate_hz
        max_step = self.velocity_limits * control_dt
        current_target = q if self._target_q is None else self._target_q
        next_target = np.clip(
            current_target + qdot.astype(np.float32) * control_dt,
            current_target - max_step,
            current_target + max_step,
        )
        max_target_lead = self.params.max_target_lead_fraction * (
            self.const.effort_limits / np.maximum(self.const.stiffness, 1.0e-6)
        )
        next_target = np.clip(next_target, q - max_target_lead, q + max_target_lead)
        self._target_q = np.clip(
            next_target, self.command_lower, self.command_upper
        ).astype(np.float32, copy=False)
        return self._target_q.copy()


__all__ = ["DLSReachController"]
