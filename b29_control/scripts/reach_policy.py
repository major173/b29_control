"""
reach_policy.py — GP11 Reach 任务的策略运行时核心模块

【职责】纯 Python，零 ROS 依赖：
  - 配置加载 (Config)
  - 关节顺序映射 (JointMapper)
  - 32D 观测构建 (ObservationBuilder)
  - Action → joint target 映射 (ActionMapper)
  - PD 力矩计算 (compute_pd_torques)
  - ONNX 推理封装 (PolicyRunner)
  - 安全限幅 (SafetyLimiter)

【一致性】所有数值算法逐位复刻
  sim2sim_mujoco/gp11_reach_runtime.py 中的：
    build_actor_observation / clip_observations / clip_actions /
    map_actions_to_targets / compute_pd_torques / OnnxPolicy
  改动训练侧公式后，必须同步本文件。

【迁移】仿真节点 (mujoco_sim_node) 与实机执行节点共用本模块，
       仅替换执行端节点即可完成 sim2real。

【使用】
    from deploy.reach_policy import Config, PolicyRunner, SafetyLimiter
    cfg = Config.load("deploy/config.yaml")
    runner = PolicyRunner(cfg)
    obs = runner.build_observation(q, dq, last_action, applied_tau,
                                    goal_pos_in_ref, goal_x_axis_in_ref)
    raw_action, action, target_q = runner.step(obs)
    safe_target = SafetyLimiter(cfg).apply(target_q, q_current)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


# --------------------------------------------------------------------------- #
# 配置加载                                                                     #
# --------------------------------------------------------------------------- #

@dataclass
class RuntimeParams:
    """训练侧锁定常量。改动需与 gp11_reach_runtime.py 同步。"""
    anchor_side: str
    active_dof_names: List[str]
    sim_dt: float
    control_decimation: int
    hard_lower: np.ndarray
    hard_upper: np.ndarray
    velocity_limits: np.ndarray
    effort_limits: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    clip_observations: float
    clip_actions: float
    goal_refresh_interval_s: float

    @property
    def control_dt(self) -> float:
        return self.sim_dt * self.control_decimation

    @property
    def control_rate_hz(self) -> float:
        return 1.0 / self.control_dt

    @property
    def active_side_one_hot(self) -> np.ndarray:
        # 约定与 gp11_reach_runtime.py 中 _RUNTIME_SIDE_SPECS 一致
        if self.anchor_side == "right":
            return np.array([1.0, 0.0], dtype=np.float32)
        if self.anchor_side == "left":
            return np.array([0.0, 1.0], dtype=np.float32)
        raise ValueError(f"unsupported anchor_side: {self.anchor_side}")


@dataclass
class SafetyParams:
    enable: bool
    max_joint_vel: float  # rad/s
    target_step_clip: float  # rad
    deadman_required: bool


@dataclass
class DeployParams:
    onnx_model_path: str
    control_rate_hz: float
    topics: Dict[str, str]
    safety: SafetyParams
    dq_alpha: float              # 速度低通滤波系数
    mujoco_runtime_dir: str
    mujoco_seed: int
    mujoco_publish_target_point: bool


@dataclass
class Config:
    runtime: RuntimeParams
    deploy: DeployParams
    source_path: Path

    @classmethod
    def load(cls, path) -> "Config":
        try:
            import yaml  # type: ignore
        except ImportError as exc:
            raise RuntimeError("PyYAML is required to load deploy/config.yaml") from exc
        path = Path(path)
        with path.open("r") as f:
            raw = yaml.safe_load(f)
        rt_raw = raw["runtime"]
        runtime = RuntimeParams(
            anchor_side=str(rt_raw["anchor_side"]).strip().lower(),
            active_dof_names=list(rt_raw["active_dof_names"]),
            sim_dt=float(rt_raw["sim_dt"]),
            control_decimation=int(rt_raw["control_decimation"]),
            hard_lower=np.asarray(rt_raw["hard_lower"], dtype=np.float32),
            hard_upper=np.asarray(rt_raw["hard_upper"], dtype=np.float32),
            velocity_limits=np.asarray(rt_raw["velocity_limits"], dtype=np.float32),
            effort_limits=np.asarray(rt_raw["effort_limits"], dtype=np.float32),
            stiffness=np.asarray(rt_raw["stiffness"], dtype=np.float32),
            damping=np.asarray(rt_raw["damping"], dtype=np.float32),
            clip_observations=float(rt_raw["clip_observations"]),
            clip_actions=float(rt_raw["clip_actions"]),
            goal_refresh_interval_s=float(rt_raw["goal_refresh_interval_s"]),
        )
        if len(runtime.active_dof_names) != 4:
            raise ValueError("active_dof_names must have 4 entries")
        for name in ("hard_lower", "hard_upper", "velocity_limits",
                     "effort_limits", "stiffness", "damping"):
            arr = getattr(runtime, name)
            if arr.shape != (4,):
                raise ValueError(f"runtime.{name} must have shape (4,), got {arr.shape}")

        dp_raw = raw["deploy"]
        sf_raw = dp_raw["safety"]
        mj_raw = dp_raw.get("mujoco", {})
        deploy = DeployParams(
            onnx_model_path=str(dp_raw["onnx_model_path"]),
            control_rate_hz=float(dp_raw["control_rate_hz"]),
            topics=dict(dp_raw["topics"]),
            safety=SafetyParams(
                enable=bool(sf_raw["enable"]),
                max_joint_vel=float(sf_raw["max_joint_vel"]),
                target_step_clip=float(sf_raw["target_step_clip"]),
                deadman_required=bool(sf_raw.get("deadman_required", False)),
            ),
            dq_alpha=float(dp_raw.get("obs_filter", {}).get("dq_alpha", 1.0)),
            mujoco_runtime_dir=str(mj_raw.get("runtime_dir",
                                              "sim2sim_mujoco/gp11_description/runtime")),
            mujoco_seed=int(mj_raw.get("seed", 0)),
            mujoco_publish_target_point=bool(mj_raw.get("publish_target_point", True)),
        )
        return cls(runtime=runtime, deploy=deploy, source_path=path.resolve())


# --------------------------------------------------------------------------- #
# 工具：向量校验                                                                #
# --------------------------------------------------------------------------- #

def _require_vector(name: str, values, expected_size: int) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float32)
    if arr.shape != (expected_size,):
        raise ValueError(f"{name} must have shape ({expected_size},), got {arr.shape}")
    return arr


# --------------------------------------------------------------------------- #
# 关节顺序映射：name ↔ index                                                    #
# --------------------------------------------------------------------------- #

class JointMapper:
    """根据 JointState.name 顺序，重排到训练侧的 active_dof_names 顺序。

    实机和 Gazebo 的 /joint_states 顺序均不可靠，必须以 name 字典查询。
    """

    def __init__(self, runtime: RuntimeParams) -> None:
        self._target_names: List[str] = list(runtime.active_dof_names)
        self._cached_indices: Optional[np.ndarray] = None
        self._cached_source_names: Optional[Tuple[str, ...]] = None

    @property
    def target_names(self) -> List[str]:
        return list(self._target_names)

    def reorder(self, source_names: Sequence[str], values: Sequence[float]) -> np.ndarray:
        """从 (source_names, values) 中按 target_names 顺序抽取。"""
        source_names_t = tuple(source_names)
        if self._cached_source_names != source_names_t:
            name_to_idx = {n: i for i, n in enumerate(source_names)}
            try:
                indices = np.asarray(
                    [name_to_idx[n] for n in self._target_names], dtype=np.int64
                )
            except KeyError as exc:
                missing = set(self._target_names) - set(source_names_t)
                raise KeyError(f"missing joints in JointState: {sorted(missing)}") from exc
            self._cached_indices = indices
            self._cached_source_names = source_names_t
        values_arr = np.asarray(values, dtype=np.float32)
        return values_arr[self._cached_indices].astype(np.float32, copy=False)


# --------------------------------------------------------------------------- #
# 观测构建：32D                                                                 #
# --------------------------------------------------------------------------- #

class ObservationBuilder:
    """逐位复刻 gp11_reach_runtime.build_actor_observation。

    维度： q_norm(4) + dq_norm(4) + last_action(4)
         + lower_margin_norm(4) + upper_margin_norm(4)
         + goal_pos_in_ref(3) + goal_x_axis_in_ref(3)
         + active_side_one_hot(2) = 28
    """

    OBS_SIZE = 28

    def __init__(self, runtime: RuntimeParams) -> None:
        self._rt = runtime
        rt = runtime
        self._q_mid = (0.5 * (rt.hard_lower + rt.hard_upper)).astype(np.float32)
        self._q_half = np.maximum(0.5 * (rt.hard_upper - rt.hard_lower), 1.0e-6).astype(np.float32)
        self._range = np.maximum(rt.hard_upper - rt.hard_lower, 1.0e-6).astype(np.float32)
        self._vel_div = np.maximum(rt.velocity_limits, 1.0).astype(np.float32)
        self._eff_div = np.maximum(rt.effort_limits, 1.0).astype(np.float32)
        self._side_one_hot = rt.active_side_one_hot.copy()

    def build(
            self,
            *,
            q: np.ndarray,
            dq: np.ndarray,
            last_action: np.ndarray,
            joint_torques: np.ndarray,  # 保留参数兼容性，但不使用
            goal_pos_in_ref: np.ndarray,
            goal_x_axis_in_ref: np.ndarray,
    ) -> np.ndarray:
        q = _require_vector("q", q, 4)
        dq = _require_vector("dq", dq, 4)
        last_action = _require_vector("last_action", last_action, 4)
        goal_pos = _require_vector("goal_pos_in_ref", goal_pos_in_ref, 3)
        goal_x_axis = _require_vector("goal_x_axis_in_ref", goal_x_axis_in_ref, 3)

        rt = self._rt
        q_norm = (q - self._q_mid) / self._q_half
        dq_norm = dq / self._vel_div
        lower_margin = (q - rt.hard_lower) / self._range
        upper_margin = (rt.hard_upper - q) / self._range

        obs = np.concatenate(
            [
                q_norm,
                dq_norm,
                last_action,
                lower_margin,
                upper_margin,
                goal_pos,
                goal_x_axis,
                self._side_one_hot,
            ],
            axis=0,
        ).astype(np.float32, copy=False)

        return np.clip(obs, -rt.clip_observations, rt.clip_observations)


# --------------------------------------------------------------------------- #
# Action 映射 / PD 力矩                                                         #
# --------------------------------------------------------------------------- #

class ActionMapper:
    """逐位复刻 gp11_reach_runtime.map_actions_to_targets / clip_actions。"""

    def __init__(self, runtime: RuntimeParams) -> None:
        self._rt = runtime

    def clip(self, action: np.ndarray) -> np.ndarray:
        action = _require_vector("action", action, 4)
        return np.clip(action, -self._rt.clip_actions, self._rt.clip_actions)

    def to_target_q(self, action: np.ndarray) -> np.ndarray:
        action = _require_vector("action", action, 4)
        rt = self._rt
        return (rt.hard_lower + 0.5 * (action + 1.0) * (rt.hard_upper - rt.hard_lower)).astype(
            np.float32, copy=False
        )


def compute_pd_torques(
        q: np.ndarray,
        dq: np.ndarray,
        target_q: np.ndarray,
        runtime: RuntimeParams,
) -> np.ndarray:
    """逐位复刻 gp11_reach_runtime.compute_pd_torques。"""
    q = _require_vector("q", q, 4)
    dq = _require_vector("dq", dq, 4)
    target_q = _require_vector("target_q", target_q, 4)
    tau = runtime.stiffness * (target_q - q) - runtime.damping * dq
    return np.clip(tau, -runtime.effort_limits, runtime.effort_limits).astype(np.float32, copy=False)


# --------------------------------------------------------------------------- #
# ONNX 推理封装                                                                 #
# --------------------------------------------------------------------------- #

class PolicyRunner:
    """ONNX 推理 + last_action 状态机 + Action 映射。

    用法：
        runner = PolicyRunner(cfg, model_path=...)
        raw_action, action, target_q = runner.step(obs)
        runner.last_action 自动更新为本步 action
    """

    def __init__(self, cfg: Config, model_path: Optional[str] = None) -> None:
        try:
            import onnxruntime as ort  # type: ignore
        except ImportError as exc:
            raise RuntimeError("onnxruntime required for PolicyRunner") from exc

        rt = cfg.runtime
        path_str = model_path or cfg.deploy.onnx_model_path
        _models_dir = Path(__file__).resolve().parent.parent / "models" / "reach"
        if not path_str:
            # 空字符串：用默认 stage0.onnx
            path_str = str(_models_dir / "stage0.onnx")

        candidate = Path(path_str).expanduser()
        if not candidate.is_absolute() or not candidate.exists():
            # 非绝对路径或绝对路径不存在：在 models/reach/ 目录下查找
            local = _models_dir / candidate.name
            if local.exists():
                candidate = local

        self.model_path = candidate.resolve()
        if not self.model_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {self.model_path}")
        self._session = ort.InferenceSession(
            str(self.model_path), providers=["CPUExecutionProvider"]
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self._mapper = ActionMapper(rt)
        self._rt = rt
        self.last_action = np.zeros(4, dtype=np.float32)

    def reset(self) -> None:
        self.last_action[:] = 0.0

    def infer(self, obs: np.ndarray) -> np.ndarray:
        obs = _require_vector("obs", obs, ObservationBuilder.OBS_SIZE)
        outputs = self._session.run(
            [self._output_name],
            {self._input_name: obs[np.newaxis, :].astype(np.float32, copy=False)},
        )[0]
        return np.asarray(outputs, dtype=np.float32).reshape(4)

    def step(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """obs(32D) → (raw_action, action_clipped, target_q)，并更新 last_action。"""
        raw_action = self.infer(obs)
        action = self._mapper.clip(raw_action)
        target_q = self._mapper.to_target_q(action)
        self.last_action = action.copy()
        return raw_action, action, target_q


# --------------------------------------------------------------------------- #
# 安全限幅器                                                                    #
# --------------------------------------------------------------------------- #

class SafetyLimiter:
    """部署在 PolicyRunner 输出之后、控制器输入之前的限幅层。

    两道护栏：
      1) 速度限幅： |target - last_target| / dt <= max_joint_vel
      2) 步长钳位： |target - q_current| <= target_step_clip

    仿真阶段 max_joint_vel 与训练值对齐 (4.0)，target_step_clip 较松；
    实机阶段从极小值起步 (0.05 rad/s, 0.02 rad)，验证后逐步放开。
    """

    def __init__(self, cfg: Config) -> None:
        self._sf = cfg.deploy.safety
        self._rt = cfg.runtime
        self._dt = 1.0 / float(cfg.deploy.control_rate_hz)
        self._last_target: Optional[np.ndarray] = None

    def reset(self, q_current: Optional[np.ndarray] = None) -> None:
        if q_current is not None:
            self._last_target = _require_vector("q_current", q_current, 4).copy()
        else:
            self._last_target = None

    def apply(self, target_q: np.ndarray, q_current: np.ndarray) -> np.ndarray:
        target = _require_vector("target_q", target_q, 4).copy()
        q_now = _require_vector("q_current", q_current, 4)
        rt = self._rt

        if not self._sf.enable:
            return np.clip(target, rt.hard_lower, rt.hard_upper)

        # 1) 步长钳位：|target - q_current| <= target_step_clip
        clip = float(self._sf.target_step_clip)
        if clip > 0.0:
            target = np.clip(target, q_now - clip, q_now + clip)

        # 2) 速度限幅：|Δtarget|/dt <= max_joint_vel
        if self._last_target is not None:
            max_step = float(self._sf.max_joint_vel) * self._dt
            target = np.clip(target,
                             self._last_target - max_step,
                             self._last_target + max_step)

        # 3) 硬限位
        target = np.clip(target, rt.hard_lower, rt.hard_upper)
        self._last_target = target.copy()
        return target.astype(np.float32, copy=False)


# --------------------------------------------------------------------------- #
# FK 加载工具（推理节点和键盘节点共用）                                           #
# --------------------------------------------------------------------------- #

_SIDE_TOOL_BODIES = {
    "left":  ("r_gripper_left_uprod", "r_gripper_right_uprod"),
    "right": ("l_gripper_left_up",    "l_gripper_right_up"),
}

_SIDE_URDF_NAMES = {
    "left":  "gp11_scene_left_gripper_root_coacd.urdf",
    "right": "gp11_scene_right_gripper_root_coacd.urdf",
}


def load_fk_model(anchor_side: str, urdf_dir: Optional[Path] = None):
    """加载训练侧 FK 模型（UrdfKinematicModel）。

    Returns:
        (kinematics, obs_ref_origin, obs_ref_rot, tool_left_body, tool_right_body)
        失败时返回 (None, None, None, None, None)
    """
    import importlib, os, sys

    # 尝试多个路径找到 b29_locomotion
    for _candidate in [
        os.environ.get("B29_LOCOMOTION_ROOT", ""),
        # 从 PYTHONPATH 里找已有的
        next((p for p in sys.path if "b29_locomotion" in p), ""),
        # 相对于本文件向上查找（scripts → b29_control → b29_control → src → B29 → usetest → ~ → RL/b29_locomotion）
        str(Path(__file__).resolve().parent.parent.parent.parent.parent / "RL" / "b29_locomotion"),
        str(Path.home() / "usetest" / "RL" / "b29_locomotion"),
    ]:
        if _candidate and Path(_candidate).exists() and _candidate not in sys.path:
            sys.path.insert(0, _candidate)

    try:
        _mod = importlib.import_module("sim2sim_mujoco.gp11_reach_runtime")
        _UrdfKinematicModel = getattr(_mod, "UrdfKinematicModel")
    except Exception as e:
        print(f"[load_fk_model] import failed: {e}")
        return None, None, None, None, None

    if urdf_dir is None:
        urdf_dir = Path(__file__).resolve().parent.parent / "models" / "gp11_urdf"

    urdf_path = urdf_dir / _SIDE_URDF_NAMES[anchor_side]
    tool_left, tool_right = _SIDE_TOOL_BODIES[anchor_side]
    obs_ref_body = f"{anchor_side}_second_leg"

    try:
        km = _UrdfKinematicModel.from_urdf(
            urdf_path,
            tool_left_body=tool_left,
            tool_right_body=tool_right,
            orientation_body=obs_ref_body,
        )
        nom = km.nominal_link_transform(obs_ref_body)
        obs_ref_origin = nom[:3, 3].astype(np.float32)
        obs_ref_rot    = nom[:3, :3].astype(np.float32)
        return km, obs_ref_origin, obs_ref_rot, tool_left, tool_right
    except Exception as e:
        import traceback
        print(f"[load_fk_model] failed: {e}")
        traceback.print_exc()
        return None, None, None, None, None


__all__ = [
    "Config",
    "RuntimeParams",
    "DeployParams",
    "SafetyParams",
    "JointMapper",
    "ObservationBuilder",
    "ActionMapper",
    "PolicyRunner",
    "SafetyLimiter",
    "compute_pd_torques",
    "load_fk_model",
]
