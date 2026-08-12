#!/usr/bin/env python3
"""
rl_inference_node.py — GP11 Reach RL 推理节点（仿真/实机共享）

【职责】
  - 订阅 /joint_states (sensor_msgs/JointState) 与 /gp11/rl/target_point_local
  - 用 reach_policy 模块构建 28D 观测，按配置调用 ONNX 或 DLS
  - 应用 SafetyLimiter 限幅，发布 /gp11/rl/joint_targets (Float64MultiArray)
  - 同步发布观测 / 原始 action / 限幅前 target，便于离线对齐与回放

【触发方式】callback-driven
  /joint_states 到达即推理，避免 timer 与传感器异步引入的相位差。
  仿真 (mujoco_sim_node) 和实机执行器都按 50Hz 发布 /joint_states。

【迁移】sim → real 切换零改动。换执行端节点即可。

【启动】
  rosrun b29_control rl_inference_node.py _config:=$(rospack find b29_control)/config/reach_rl_config.yaml
  或在 launch 文件中以 <node ... args="--config $(find b29_control)/config/reach_rl_config.yaml"/> 运行
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

# ---- 路径：reach_policy 与本文件同目录 ---- #
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from reach_policy import (  # noqa: E402
    Config,
    JointMapper,
    ObservationBuilder,
    PolicyRunner,
    SafetyLimiter,
    load_fk_model,
)
from dls_reach_controller import DLSReachController  # noqa: E402

# ---- 导入训练侧正运动学（延迟到 __init__ 以避免 mujoco 依赖）---- #
UrdfKinematicModel = None


# ---- ROS 导入：在 main 内 lazy import 以便单元测试加载本文件 ---- #
def _import_ros():
    import rospy  # type: ignore
    from sensor_msgs.msg import JointState  # type: ignore
    from std_msgs.msg import Float64MultiArray  # type: ignore
    from visualization_msgs.msg import Marker  # type: ignore
    return rospy, JointState, Float64MultiArray, Marker


# --------------------------------------------------------------------------- #
# 节点                                                                        #
# --------------------------------------------------------------------------- #

class RLInferenceNode:
    def __init__(self, cfg: Config) -> None:
        rospy, JointState, Float64MultiArray, Marker = _import_ros()
        self._rospy = rospy
        self._JointState = JointState
        self._F64MA = Float64MultiArray
        self._Marker = Marker

        self.cfg = cfg
        self.rt = cfg.runtime
        self.topics = cfg.deploy.topics

        self.mode = cfg.deploy.mode
        self.platform = cfg.deploy.platform
        self.joint_mapper = JointMapper(self.rt)
        self.obs_builder = ObservationBuilder(self.rt)
        self.runner = PolicyRunner(cfg) if self.mode == "onnx" else None
        self.safety = SafetyLimiter(cfg)

        self._lock = threading.Lock()
        self._latest_q: np.ndarray | None = None
        self._latest_dq: np.ndarray | None = None
        self._latest_torque: np.ndarray = np.zeros(4, dtype=np.float32)
        self._latest_target_pos: np.ndarray | None = None
        self._dq_filtered: np.ndarray = np.zeros(4, dtype=np.float32)
        self._dq_alpha: float = float(cfg.deploy.dq_alpha)
        self._prev_target_q: np.ndarray | None = None  # 用于检测突变
        self._last_state_monotonic: float | None = None
        self._target_generation = 0
        self._last_output: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
        self._fault_reason: str | None = None
        self._track_error_since: float | None = None
        self._dq_violation_since: float | None = None
        self._dls_stop = threading.Event()
        self._dls_wakeup = threading.Event()
        # 训练里 goal_x_axis_in_ref 来自目标姿态，部署阶段先用 +X 轴占位，
        # 后续若需要支持姿态目标再扩展话题契约。
        self._target_x_axis: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        # ---- 发布器 ---- #
        self.pub_joint_targets = rospy.Publisher(
            self.topics["joint_targets"], Float64MultiArray, queue_size=1
        )
        self.pub_joint_targets_raw = rospy.Publisher(
            self.topics["joint_targets_raw"], Float64MultiArray, queue_size=1
        )
        self.pub_obs = rospy.Publisher(
            self.topics["observation"], Float64MultiArray, queue_size=1
        )
        self.pub_action_raw = rospy.Publisher(
            self.topics["action_raw"], Float64MultiArray, queue_size=1
        )
        self.pub_fk_marker = rospy.Publisher(
            "/gp11/rl/fk_target_marker", Marker, queue_size=1
        )
        self.pub_fk_marker_raw = rospy.Publisher(
            "/gp11/rl/fk_target_raw_marker", Marker, queue_size=1
        )

        # ---- 订阅器 ---- #
        rospy.Subscriber(
            self.topics["target_point"], Float64MultiArray, self._on_target_point,
            queue_size=1,
        )
        rospy.Subscriber(
            self.topics["joint_states"], JointState, self._on_joint_states,
            queue_size=1, tcp_nodelay=True,
        )

        rospy.loginfo("[rl_inference] anchor_side=%s active_dof=%s",
                      self.rt.anchor_side, self.rt.active_dof_names)
        rospy.loginfo("[rl_inference] mode=%s platform=%s", self.mode, self.platform)
        if self.runner is not None:
            rospy.loginfo("[rl_inference] onnx=%s", self.runner.model_path)
        rospy.loginfo("[rl_inference] safety: enable=%s vel<=%.3f step<=%.3f",
                      self.cfg.deploy.safety.enable,
                      self.cfg.deploy.safety.max_joint_vel,
                      self.cfg.deploy.safety.target_step_clip)

        # obs_ref frame：与训练侧 obs_ref_body 一致（固定端 second_leg）
        self._obs_ref_frame = f"{self.rt.anchor_side}_second_leg"

        # 正运动学：用包内预置训练侧 URDF，确保 obs_ref 坐标系与训练一致
        self._kinematics, self._obs_ref_origin, self._obs_ref_rot, \
            self._tool_left_body, self._tool_right_body = load_fk_model(
                self.rt.anchor_side,
                urdf_dir=_THIS_DIR.parent / "models" / "gp11_urdf",
            )
        if self._kinematics is not None:
            rospy.loginfo("[rl_inference] FK loaded: anchor=%s obs_ref_x=%s",
                          self.rt.anchor_side, self._obs_ref_rot[0].tolist())
        else:
            if self.mode == "dls":
                raise RuntimeError("DLS mode requires the package FK model")
            rospy.loginfo("[rl_inference] FK unavailable: tool/fk markers disabled")

        self._dls = None
        if self.mode == "dls":
            dls_velocity_limits = self.rt.velocity_limits.copy()
            if cfg.deploy.safety.enable:
                dls_velocity_limits = np.minimum(
                    dls_velocity_limits,
                    np.full(4, cfg.deploy.safety.max_joint_vel, dtype=np.float32),
                )
            self._dls = DLSReachController(
                kinematics=self._kinematics,
                runtime=self.rt,
                obs_ref_origin_root=self._obs_ref_origin,
                obs_ref_rot_root=self._obs_ref_rot,
                params=cfg.deploy.dls,
                velocity_limits=dls_velocity_limits,
            )
            self._dls_worker = threading.Thread(
                target=self._dls_loop, name="b29_dls_control", daemon=True
            )
            self._dls_publisher = threading.Thread(
                target=self._dls_publish_loop, name="b29_dls_publish", daemon=True
            )
            self._dls_worker.start()
            self._dls_publisher.start()
            rospy.loginfo(
                "[rl_inference] DLS control=%sHz publish=%sHz",
                cfg.deploy.dls.control_rate_hz,
                cfg.deploy.control_rate_hz,
            )
        if hasattr(rospy, "on_shutdown"):
            rospy.on_shutdown(self._shutdown)

    # ---- callbacks ---- #
    def _on_target_point(self, msg) -> None:
        if len(msg.data) < 3:
            return
        with self._lock:
            prev = self._latest_target_pos
            self._latest_target_pos = np.asarray(msg.data[:3], dtype=np.float32)
            self._target_generation += 1
            self._dls_wakeup.set()
        if prev is None:
            self._rospy.loginfo(
                "[rl_inference] target_point received: [%.3f, %.3f, %.3f]",
                msg.data[0], msg.data[1], msg.data[2],
            )

    def _on_joint_states(self, msg) -> None:
        try:
            q = self.joint_mapper.reorder(msg.name, msg.position)
            dq = (
                self.joint_mapper.reorder(msg.name, msg.velocity)
                if len(msg.velocity) >= len(msg.name)
                else np.zeros(4, dtype=np.float32)
            )
            torque = (
                self.joint_mapper.reorder(msg.name, msg.effort)
                if len(msg.effort) >= len(msg.name)
                else np.zeros(4, dtype=np.float32)
            )
        except KeyError as exc:
            self._rospy.logwarn_throttle(2.0, "[rl_inference] joint_states missing: %s", exc)
            return

        with self._lock:
            self._latest_q = q
            self._latest_dq = dq
            # 低通滤波：消除 Gazebo 速度估算噪声（有限差分放大高频抖动）
            self._dq_filtered = (self._dq_alpha * dq
                                 + (1.0 - self._dq_alpha) * self._dq_filtered)
            dq_filtered = self._dq_filtered.copy()
            self._latest_torque = torque
            target_pos = self._latest_target_pos
            target_x_axis = self._target_x_axis.copy()
            self._last_state_monotonic = time.monotonic()

        if target_pos is None:
            self._rospy.loginfo_throttle(2.0, "[rl_inference] waiting for target_point...")
            return

        if self.mode == "onnx":
            self._step(q, dq_filtered, torque, target_pos, target_x_axis)
        else:
            self._update_watchdog(q, dq_filtered)

    # ---- DLS 控制与发布 ---- #
    def _dls_loop(self) -> None:
        assert self._dls is not None
        period = 1.0 / float(self.cfg.deploy.dls.control_rate_hz)
        next_deadline = time.monotonic()
        seen_generation = -1
        while not self._dls_stop.is_set() and not self._rospy.is_shutdown():
            self._dls_wakeup.wait(max(0.0, next_deadline - time.monotonic()))
            self._dls_wakeup.clear()
            now = time.monotonic()
            if now < next_deadline:
                continue
            next_deadline = now + period

            with self._lock:
                q = None if self._latest_q is None else self._latest_q.copy()
                dq = None if self._latest_dq is None else self._dq_filtered.copy()
                torque = self._latest_torque.copy()
                goal = None if self._latest_target_pos is None else self._latest_target_pos.copy()
                generation = self._target_generation
                fault = self._fault_reason
            if q is None or dq is None or goal is None:
                continue
            if fault is not None:
                continue

            try:
                if generation != seen_generation:
                    self._dls.set_goal(
                        goal,
                        self._target_x_axis,
                        q_start=q,
                        known_reachable=True,
                    )
                    seen_generation = generation
                raw_target = self._dls.compute(q, dq, goal, self._target_x_axis)
                safe_target = self.safety.apply(raw_target, q)
                self._dls.sync_applied_target(safe_target)
            except Exception as exc:
                self._enter_fault(f"DLS control failed: {exc}")
                continue

            action = (
                2.0 * (raw_target - self.rt.hard_lower)
                / (self.rt.hard_upper - self.rt.hard_lower)
                - 1.0
            ).astype(np.float32, copy=False)
            obs = self.obs_builder.build(
                q=q,
                dq=dq,
                last_action=action,
                joint_torques=torque,
                goal_pos_in_ref=goal,
                goal_x_axis_in_ref=self._target_x_axis,
            )
            with self._lock:
                self._last_output = (
                    raw_target.copy(),
                    safe_target.copy(),
                    obs.copy(),
                    action.copy(),
                )

            self._rospy.loginfo_throttle(
                5.0,
                "[rl_inference] DLS goal=[%.3f,%.3f,%.3f] q=[%.2f,%.2f,%.2f,%.2f] "
                "target=[%.2f,%.2f,%.2f,%.2f]",
                goal[0], goal[1], goal[2], q[0], q[1], q[2], q[3],
                safe_target[0], safe_target[1], safe_target[2], safe_target[3],
            )

    def _dls_publish_loop(self) -> None:
        period = 1.0 / float(self.cfg.deploy.control_rate_hz)
        next_deadline = time.monotonic()
        while not self._dls_stop.is_set() and not self._rospy.is_shutdown():
            now = time.monotonic()
            if now < next_deadline:
                time.sleep(next_deadline - now)
            next_deadline = time.monotonic() + period

            with self._lock:
                last_state = self._last_state_monotonic
                output = self._last_output
            if output is None:
                continue
            if (
                self.platform == "hardware"
                and last_state is not None
                and time.monotonic() - last_state > self.cfg.deploy.watchdog.feedback_timeout_s
            ):
                self._enter_fault("joint_states feedback timeout")
            self._publish_dls_output(output)

    def _publish_dls_output(self, output) -> None:
        raw_target, safe_target, obs, action = output
        self.pub_joint_targets.publish(self._F64MA(data=safe_target.tolist()))
        self.pub_joint_targets_raw.publish(self._F64MA(data=raw_target.tolist()))
        self.pub_obs.publish(self._F64MA(data=obs.tolist()))
        self.pub_action_raw.publish(self._F64MA(data=action.tolist()))
        if self._kinematics is None:
            return
        try:
            stamp = self._rospy.Time.now()
            self._publish_fk_marker(
                safe_target,
                stamp,
                self.pub_fk_marker,
                marker_ns="dls_fk_target_safe",
                marker_id=0,
                color=(0.95, 0.50, 0.05, 0.90),
                diameter=0.04,
            )
            self._publish_fk_marker(
                raw_target,
                stamp,
                self.pub_fk_marker_raw,
                marker_ns="dls_fk_target_raw",
                marker_id=1,
                color=(0.10, 0.55, 0.95, 0.90),
                diameter=0.032,
            )
        except Exception as exc:
            self._rospy.logwarn_throttle(5.0, "[rl_inference] DLS FK marker failed: %s", exc)

    def _update_watchdog(self, q: np.ndarray, dq: np.ndarray) -> None:
        if self.platform != "hardware":
            return
        with self._lock:
            output = self._last_output
        if output is None or self._fault_reason is not None:
            return
        safe_target = output[1]
        now = time.monotonic()
        track_bad = bool(
            np.max(np.abs(safe_target - q)) > self.cfg.deploy.watchdog.track_error_rad
        )
        dq_bad = bool(np.max(np.abs(dq)) > self.cfg.deploy.watchdog.dq_limit_rad_s)
        with self._lock:
            if track_bad:
                self._track_error_since = self._track_error_since or now
            else:
                self._track_error_since = None
            if dq_bad:
                self._dq_violation_since = self._dq_violation_since or now
            else:
                self._dq_violation_since = None
            track_since = self._track_error_since
            dq_since = self._dq_violation_since
        if track_since is not None and now - track_since >= self.cfg.deploy.watchdog.track_error_persist_s:
            self._enter_fault("joint target tracking error persisted")
        elif dq_since is not None and now - dq_since >= self.cfg.deploy.watchdog.dq_persist_s:
            self._enter_fault("joint velocity limit violation persisted")

    def _enter_fault(self, reason: str) -> None:
        with self._lock:
            if self._fault_reason is not None:
                return
            self._fault_reason = str(reason)
        self._rospy.logerr("[rl_inference] DLS FAULT: %s; holding last target", reason)

    def _shutdown(self) -> None:
        self._dls_stop.set()
        self._dls_wakeup.set()
        if self._dls is not None:
            self._dls.close()

    # ---- 推理一步 ---- #
    def _step(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        torque: np.ndarray,
        target_pos: np.ndarray,
        target_x_axis: np.ndarray,
    ) -> None:
        assert self.runner is not None
        last_action = self.runner.last_action
        obs = self.obs_builder.build(
            q=q,
            dq=dq,
            last_action=last_action,
            joint_torques=torque,
            goal_pos_in_ref=target_pos,
            goal_x_axis_in_ref=target_x_axis,
        )
        raw_action, action, target_q_raw = self.runner.step(obs)
        target_q_safe = self.safety.apply(target_q_raw, q)

        # ---- 抽搐检测：与上一步对比，发现突变立即打印 ----
        if self._prev_target_q is not None:
            delta = target_q_safe - self._prev_target_q
            if np.any(np.abs(delta) > 0.05):  # 单步超过 0.05 rad 认为是突变
                self._rospy.logwarn(
                    "[rl_inference] JUMP detected! delta_target=%s "
                    "raw_action=%s dq_norm=%s q_norm=%s goal=%s",
                    np.round(delta, 3).tolist(),
                    np.round(raw_action, 3).tolist(),
                    np.round(dq / np.array([4.,4.,4.,4.]), 3).tolist(),
                    np.round((q - np.array([-0.,0.,-0.,0.])) / np.array([1.1,4.71,1.1,4.71]), 3).tolist(),
                    np.round(target_pos, 3).tolist(),
                )
        self._prev_target_q = target_q_safe.copy()

        self._rospy.loginfo_throttle(
            5.0,
            "[rl_inference] goal=[%.3f,%.3f,%.3f] q=[%.2f,%.2f,%.2f,%.2f] "
            "target=[%.2f,%.2f,%.2f,%.2f] action=[%.2f,%.2f,%.2f,%.2f]",
            target_pos[0], target_pos[1], target_pos[2],
            q[0], q[1], q[2], q[3],
            target_q_safe[0], target_q_safe[1], target_q_safe[2], target_q_safe[3],
            raw_action[0], raw_action[1], raw_action[2], raw_action[3],
        )

        # ---- 发布 ---- #
        self.pub_joint_targets.publish(self._F64MA(data=target_q_safe.tolist()))
        self.pub_joint_targets_raw.publish(self._F64MA(data=target_q_raw.tolist()))
        self.pub_obs.publish(self._F64MA(data=obs.tolist()))
        self.pub_action_raw.publish(self._F64MA(data=raw_action.tolist()))

        stamp = self._rospy.Time.now()
        if self._kinematics is not None:
            try:
                # 橙球：经过 SafetyLimiter 后真正下发的目标末端
                self._publish_fk_marker(
                    target_q_safe,
                    stamp,
                    self.pub_fk_marker,
                    marker_ns="rl_fk_target_safe",
                    marker_id=0,
                    color=(0.95, 0.50, 0.05, 0.90),
                    diameter=0.04,
                )
                # 蓝球：网络输出映射后的 raw joint target（未经过 SafetyLimiter）
                self._publish_fk_marker(
                    target_q_raw,
                    stamp,
                    self.pub_fk_marker_raw,
                    marker_ns="rl_fk_target_raw",
                    marker_id=1,
                    color=(0.10, 0.55, 0.95, 0.90),
                    diameter=0.032,
                )
            except Exception as e:
                self._rospy.logwarn_throttle(5.0, "[rl_inference] FK marker failed: %s", e)

    def _publish_fk_marker(
        self,
        target_q: np.ndarray,
        stamp,
        publisher,
        *,
        marker_ns: str,
        marker_id: int,
        color: tuple[float, float, float, float],
        diameter: float,
    ) -> None:
        anchor_name = self._obs_ref_frame
        transforms = self._kinematics._compute_link_transforms(target_q)
        T_anchor = transforms[anchor_name]
        T_tool_l = transforms[self._kinematics.tool_left_body]
        T_tool_r = transforms[self._kinematics.tool_right_body]
        tool_base = 0.5 * (T_tool_l[:3, 3] + T_tool_r[:3, 3])
        tgt_ref = T_anchor[:3, :3].T @ (tool_base - T_anchor[:3, 3])

        marker = self._Marker()
        marker.header.frame_id = self._obs_ref_frame
        marker.header.stamp = stamp
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = self._Marker.SPHERE
        marker.action = self._Marker.ADD
        marker.pose.position.x = float(tgt_ref[0])
        marker.pose.position.y = float(tgt_ref[1])
        marker.pose.position.z = float(tgt_ref[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        publisher.publish(marker)


# --------------------------------------------------------------------------- #
# 入口                                                                        #
# --------------------------------------------------------------------------- #

def _resolve_config_options(rospy) -> tuple[Path, str | None]:
    """解析配置和显式平台参数，不根据运行环境自动猜测。"""
    # ROS 参数
    cfg_param = rospy.get_param("~config", "")
    platform_param = rospy.get_param("~platform", "")
    platform = str(platform_param).strip().lower() or None
    if cfg_param:
        return Path(cfg_param).expanduser().resolve(), platform

    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--config", type=str, default="")
    parser.add_argument("--platform", choices=("gazebo", "hardware"), default="")
    # 屏蔽 rosrun 注入的 __name:= / __log:= 等 remap 参数
    cli_args = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = parser.parse_known_args(cli_args)
    platform = args.platform or platform
    if args.config:
        return Path(args.config).expanduser().resolve(), platform
    # 默认：脚本同级目录向上找 config/reach_rl_config.yaml
    default = _THIS_DIR.parent / "config" / "reach_rl_config.yaml"
    return default.resolve(), platform


def main() -> int:
    rospy, _, _, _ = _import_ros()
    rospy.init_node("gp11_rl_inference", anonymous=False, disable_signals=False)
    cfg_path, platform = _resolve_config_options(rospy)
    rospy.loginfo("[rl_inference] loading config: %s", cfg_path)
    cfg = Config.load(cfg_path, platform=platform)

    node = RLInferenceNode(cfg)
    rospy.loginfo("[rl_inference] ready, mode=%s, platform=%s", cfg.deploy.mode, cfg.deploy.platform)
    rospy.spin()
    _ = node  # keep alive
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
