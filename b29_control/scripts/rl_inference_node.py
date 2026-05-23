#!/usr/bin/env python3
"""
rl_inference_node.py — GP11 Reach RL 推理节点（仿真/实机共享）

【职责】
  - 订阅 /joint_states (sensor_msgs/JointState) 与 /gp11/rl/target_point_local
  - 用 reach_policy 模块构建 32D 观测，调用 ONNX 推理得到 4D action
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
)

# ---- 导入训练侧正运动学 ---- #
try:
    from sim2sim_mujoco.gp11_reach_runtime import UrdfKinematicModel  # noqa: E402
except ImportError:
    UrdfKinematicModel = None  # type: ignore


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

        self.joint_mapper = JointMapper(self.rt)
        self.obs_builder = ObservationBuilder(self.rt)
        self.runner = PolicyRunner(cfg)
        self.safety = SafetyLimiter(cfg)

        self._lock = threading.Lock()
        self._latest_q: np.ndarray | None = None
        self._latest_dq: np.ndarray | None = None
        self._latest_torque: np.ndarray = np.zeros(4, dtype=np.float32)
        self._latest_target_pos: np.ndarray | None = None
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
        rospy.loginfo("[rl_inference] onnx=%s", self.runner.model_path)
        rospy.loginfo("[rl_inference] safety: enable=%s vel<=%.3f step<=%.3f",
                      self.cfg.deploy.safety.enable,
                      self.cfg.deploy.safety.max_joint_vel,
                      self.cfg.deploy.safety.target_step_clip)

        # obs_ref frame：与训练侧 obs_ref_body 一致（固定端 second_leg）
        self._obs_ref_frame = f"{self.rt.anchor_side}_second_leg"

        # 正运动学：用于计算 target_q → 期望末端位置
        # anchor_side=left  → 右臂运动，tool=r_gripper_left_uprod/r_gripper_right_uprod
        # anchor_side=right → 左臂运动，tool=l_gripper_left_up/l_gripper_right_up
        _SIDE_TOOL_BODIES = {
            "left":  ("r_gripper_left_uprod", "r_gripper_right_uprod"),
            "right": ("l_gripper_left_up",    "l_gripper_right_up"),
        }
        self._kinematics = None
        if UrdfKinematicModel is not None:
            try:
                urdf_xacro = _THIS_DIR.parent / "urdf" / "b29" / "b29.urdf.xacro"
                urdf_flat = _THIS_DIR.parent / "urdf" / "b29" / f"b29_flat_{self.rt.anchor_side}.urdf"

                # 如果 flat URDF 不存在，自动生成
                if not urdf_flat.exists():
                    import subprocess
                    rospy.loginfo("[rl_inference] Generating flat URDF: %s", urdf_flat)
                    result = subprocess.run(
                        ["rosrun", "xacro", "xacro", str(urdf_xacro), f"anchor_side:={self.rt.anchor_side}"],
                        capture_output=True, text=True, check=True
                    )
                    urdf_flat.write_text(result.stdout)
                    rospy.loginfo("[rl_inference] Flat URDF generated successfully")

                tool_left, tool_right = _SIDE_TOOL_BODIES[self.rt.anchor_side]
                self._kinematics = UrdfKinematicModel.from_urdf(
                    urdf_flat,
                    tool_left_body=tool_left,
                    tool_right_body=tool_right,
                    orientation_body=f"{self.rt.anchor_side}_second_leg",
                )
                rospy.loginfo("[rl_inference] FK model loaded: tool=%s+%s", tool_left, tool_right)
            except Exception as e:
                rospy.logwarn("[rl_inference] FK init failed: %s", e)
                self._kinematics = None

    # ---- callbacks ---- #
    def _on_target_point(self, msg) -> None:
        if len(msg.data) < 3:
            return
        with self._lock:
            prev = self._latest_target_pos
            self._latest_target_pos = np.asarray(msg.data[:3], dtype=np.float32)
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
            self._latest_torque = torque
            target_pos = self._latest_target_pos
            target_x_axis = self._target_x_axis

        if target_pos is None:
            self._rospy.loginfo_throttle(2.0, "[rl_inference] waiting for target_point...")
            return

        self._step(q, dq, torque, target_pos, target_x_axis)

    # ---- 推理一步 ---- #
    def _step(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        torque: np.ndarray,
        target_pos: np.ndarray,
        target_x_axis: np.ndarray,
    ) -> None:
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

        # 发布正运动学期望末端位置（橙色球，obs_ref 坐标系）
        stamp = self._rospy.Time.now()
        if self._kinematics is not None:
            try:
                tool_mid, _ = self._kinematics.evaluate(target_q_safe)
                fk_marker = self._Marker()
                fk_marker.header.frame_id = self._obs_ref_frame
                fk_marker.header.stamp = stamp
                fk_marker.ns = "rl_fk_target"
                fk_marker.id = 0
                fk_marker.type = self._Marker.SPHERE
                fk_marker.action = self._Marker.ADD
                fk_marker.pose.position.x = float(tool_mid[0])
                fk_marker.pose.position.y = float(tool_mid[1])
                fk_marker.pose.position.z = float(tool_mid[2])
                fk_marker.pose.orientation.w = 1.0
                fk_marker.scale.x = fk_marker.scale.y = fk_marker.scale.z = 0.04
                fk_marker.color.r = 0.95
                fk_marker.color.g = 0.5
                fk_marker.color.b = 0.05
                fk_marker.color.a = 0.9
                self.pub_fk_marker.publish(fk_marker)
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# 入口                                                                        #
# --------------------------------------------------------------------------- #

def _resolve_config_path(rospy) -> Path:
    """优先从 ROS 参数 ~config 读取；否则用 --config 命令行；最后默认同包 config 目录。"""
    # ROS 参数
    cfg_param = rospy.get_param("~config", "")
    if cfg_param:
        return Path(cfg_param).expanduser().resolve()

    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--config", type=str, default="")
    # 屏蔽 rosrun 注入的 __name:= / __log:= 等 remap 参数
    cli_args = [a for a in sys.argv[1:] if not a.startswith("__")]
    args, _ = parser.parse_known_args(cli_args)
    if args.config:
        return Path(args.config).expanduser().resolve()
    # 默认：脚本同级目录向上找 config/reach_rl_config.yaml
    default = _THIS_DIR.parent / "config" / "reach_rl_config.yaml"
    return default.resolve()


def main() -> int:
    rospy, _, _, _ = _import_ros()
    rospy.init_node("gp11_rl_inference", anonymous=False, disable_signals=False)
    cfg_path = _resolve_config_path(rospy)
    rospy.loginfo("[rl_inference] loading config: %s", cfg_path)
    cfg = Config.load(cfg_path)

    node = RLInferenceNode(cfg)
    rospy.loginfo("[rl_inference] ready, spinning at sensor rate.")
    rospy.spin()
    _ = node  # keep alive
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
