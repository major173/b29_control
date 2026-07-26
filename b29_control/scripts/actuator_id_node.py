#!/usr/bin/env python3
"""
actuator_id_node.py — 执行器辨识激励节点（01 文档测试 A/B/C 的实机侧）

对单关节顺序发送小幅阶跃 / 正弦扫频，其余关节持续钉在安全位姿；
四层安全防护（前置自检 / 运行时看门狗 / 固定位姿安全停 / 分组人工闸门），
curses TUI 实时面板 + 文件日志 + 自动录 bag + 每组 npz 数据落盘。

用法（详见 docs/actuator-id.md）：
  python3 actuator_id_node.py                 # 完整协议，TUI
  python3 actuator_id_node.py --minimal       # 仅 second_leg 最小协议
  python3 actuator_id_node.py --dry-run       # 不连 ROS，校验配置并打印计划
  python3 actuator_id_node.py --no-tui        # 纯日志行模式（终端不支持 curses 时）
  python3 actuator_id_node.py --mock-ns /mock # 联调：指向 mock_plant 的话题前缀

安全须知：session 全程必须有人守在急停旁。软件保护只能缩短反应时间。
"""

from __future__ import annotations

import argparse
import datetime
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

_SCRIPTS_DIR = Path(__file__).resolve().parent
_DEFAULT_CFG = _SCRIPTS_DIR.parent / "config" / "actuator_id.yaml"


# --------------------------------------------------------------------------- #
# 配置与激励计划（纯逻辑，可离线测试）                                            #
# --------------------------------------------------------------------------- #

@dataclass
class Segment:
    """一段恒定/正弦目标。duration_s 内 target(t) 由 kind 决定。"""
    kind: str                  # "hold" | "sine"
    duration_s: float
    offset: float = 0.0        # 相对组基准位姿的偏移 (rad)
    amp: float = 0.0           # sine 幅值
    freq_hz: float = 0.0       # sine 频率

    def target(self, t_in_seg: float) -> float:
        if self.kind == "sine":
            return self.offset + self.amp * np.sin(2.0 * np.pi * self.freq_hz * t_in_seg)
        return self.offset


@dataclass
class Group:
    name: str                  # 例 "B/left_second_leg/+0.03"
    joint: str
    kind: str                  # "step" | "sweep"
    segments: List[Segment]
    needs_confirm: bool = False   # 高频扫频组需单独确认
    meta: dict = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        return sum(s.duration_s for s in self.segments)

    def peak_offset(self) -> float:
        peak = 0.0
        for s in self.segments:
            peak = max(peak, abs(s.offset) + abs(s.amp))
        return peak


def build_plan(cfg: dict, minimal: bool) -> List[Group]:
    joints = cfg["minimal_joints"] if minimal else cfg["joints"]
    proto = cfg["protocol"]
    groups: List[Group] = []
    step = proto["step"]
    amps = step["amplitudes"][:1] if minimal else step["amplitudes"]
    for joint in joints:
        for amp in amps:
            for direction in step["directions"]:
                segs: List[Segment] = []
                for _ in range(int(step["reps"])):
                    segs.append(Segment("hold", float(step["settle_s"]), 0.0))
                    segs.append(Segment("hold", float(step["hold_s"]), direction * float(amp)))
                segs.append(Segment("hold", float(step["settle_s"]), 0.0))
                sign = "+" if direction > 0 else "-"
                groups.append(Group(
                    name=f"B/{joint}/{sign}{amp}",
                    joint=joint, kind="step", segments=segs,
                    meta={"amplitude": direction * float(amp), "reps": int(step["reps"])},
                ))
    sweep = proto["sweep"]
    for joint in joints:
        low_segs: List[Segment] = []
        for f in sweep["freqs_hz"]:
            cycles = sweep["cycles_low"] if f < 1.0 else sweep["cycles_high"]
            seg = Segment("sine", cycles / float(f), 0.0, float(sweep["amplitude"]), float(f))
            if f > float(sweep["high_freq_confirm_hz"]):
                groups.append(Group(
                    name=f"C/{joint}/{f}Hz", joint=joint, kind="sweep",
                    segments=[Segment("hold", 1.0, 0.0), seg, Segment("hold", 1.0, 0.0)],
                    needs_confirm=True, meta={"freq_hz": f},
                ))
            else:
                low_segs += [Segment("hold", 1.0, 0.0), seg]
        if low_segs:
            # 低频扫频合成一组，排在该关节高频组之前；无高频组时追加到末尾
            insert_at = next((i for i, g in enumerate(groups)
                              if g.joint == joint and g.needs_confirm), len(groups))
            groups.insert(
                insert_at,
                Group(name=f"C/{joint}/low", joint=joint, kind="sweep",
                      segments=low_segs + [Segment("hold", 1.0, 0.0)],
                      meta={"freqs": [f for f in sweep["freqs_hz"]
                                      if f <= float(sweep["high_freq_confirm_hz"])]}),
            )
    return groups


def check_plan_limits(groups: List[Group], start_pose: np.ndarray, joints_all: List[str],
                      hard_lower: np.ndarray, hard_upper: np.ndarray, margin: float) -> List[str]:
    """返回违例描述列表；空 = 通过。"""
    problems = []
    for g in groups:
        j = joints_all.index(g.joint)
        peak = g.peak_offset()
        lo, hi = start_pose[j] - peak, start_pose[j] + peak
        if lo < hard_lower[j] + margin or hi > hard_upper[j] - margin:
            problems.append(
                f"{g.name}: 轨迹范围 [{lo:.3f},{hi:.3f}] 超出限位安全区 "
                f"[{hard_lower[j]+margin:.3f},{hard_upper[j]-margin:.3f}]"
            )
    return problems


# --------------------------------------------------------------------------- #
# 看门狗（纯逻辑，可离线测试）                                                   #
# --------------------------------------------------------------------------- #

class SafetyMonitor:
    def __init__(self, cfg: dict, dt: float):
        s = cfg["safety"]
        self.dt = dt
        self.fb_timeout = float(s["feedback_timeout_s"])
        self.track_err = float(s["track_err_rad"])
        self.track_n = max(1, int(round(float(s["track_err_persist_s"]) / dt)))
        self.dq_limit = float(s["dq_limit_rad_s"])
        self.dq_n = max(1, int(round(float(s["dq_persist_s"]) / dt)))
        self.tau_limits = np.asarray(s["tau_limits"], dtype=np.float64)
        self.loop_overrun = float(s["loop_overrun_s"])
        self._track_count = np.zeros(4, dtype=np.int64)
        self._dq_count = np.zeros(4, dtype=np.int64)

    def reset(self) -> None:
        self._track_count[:] = 0
        self._dq_count[:] = 0

    def tick(self, fb_age_s: float, q: np.ndarray, dq: np.ndarray,
             tau: np.ndarray, cmd: np.ndarray) -> Optional[str]:
        if fb_age_s > self.fb_timeout:
            return f"反馈断流 {fb_age_s*1e3:.0f}ms > {self.fb_timeout*1e3:.0f}ms"
        # 漏积分计数（超限 +1、正常 -1）：振荡信号每周期短暂回落也能累积触发，
        # 普通"连续超限清零"计数对自激震荡是盲的
        err = np.abs(cmd - q)
        self._track_count = np.clip(
            self._track_count + np.where(err > self.track_err, 1, -1), 0, 4 * self.track_n)
        if (self._track_count >= self.track_n).any():
            j = int(np.argmax(self._track_count))
            return f"跟踪误差异常 joint[{j}] err={err[j]:.3f}rad 持续超限"
        self._dq_count = np.clip(
            self._dq_count + np.where(np.abs(dq) > self.dq_limit, 1, -1), 0, 4 * self.dq_n)
        if (self._dq_count >= self.dq_n).any():
            j = int(np.argmax(self._dq_count))
            return f"速度异常（疑似自激/疯机）joint[{j}] |dq|={abs(dq[j]):.2f}rad/s 持续超限"
        if tau is not None and len(tau) == 4 and (np.abs(tau) > self.tau_limits).any():
            j = int(np.argmax(np.abs(tau) - self.tau_limits))
            return f"力矩超限 joint[{j}] |tau|={abs(tau[j]):.1f}"
        return None


# --------------------------------------------------------------------------- #
# Session 运行器（ROS 在这里才出现）                                            #
# --------------------------------------------------------------------------- #

class SessionState:
    """TUI 与控制线程共享的状态快照。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "INIT"           # INIT/PRECHECK/GATE/RUN/SAFE_STOP/ABORTED/DONE
        self.group_idx = 0
        self.group_total = 0
        self.group_name = ""
        self.seg_progress = 0.0
        self.q = np.zeros(4); self.dq = np.zeros(4); self.tau = np.zeros(4)
        self.cmd = np.zeros(4)
        self.fb_age_ms = 1e9
        self.watchdog_msg = "-"
        self.events: List[str] = []
        self.gate_prompt = ""
        self.last_summary = ""
        self.pending_key: Optional[str] = None

    def log_event(self, msg: str) -> None:
        with self.lock:
            self.events.append(f"{time.strftime('%H:%M:%S')} {msg}")
            self.events = self.events[-8:]

    def push_key(self, key: str) -> None:
        with self.lock:
            self.pending_key = key

    def pop_key(self) -> Optional[str]:
        with self.lock:
            key, self.pending_key = self.pending_key, None
            return key


class ActuatorIdSession:
    def __init__(self, cfg: dict, groups: List[Group], outdir: Path,
                 state: SessionState, logger: logging.Logger, mock_ns: str = ""):
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        self.rospy = rospy
        self.cfg = cfg
        self.groups = groups
        self.outdir = outdir
        self.state = state
        self.log = logger
        self.joints: List[str] = list(cfg["joints"])
        self.rate_hz = float(cfg["runtime"]["control_rate_hz"])
        self.dt = 1.0 / self.rate_hz
        self.hard_lower = np.asarray(cfg["runtime"]["hard_lower"], dtype=np.float64)
        self.hard_upper = np.asarray(cfg["runtime"]["hard_upper"], dtype=np.float64)
        self.margin = float(cfg["runtime"]["limit_margin"])
        self.monitor = SafetyMonitor(cfg, self.dt)
        self.safe_stop_publish_s = float(cfg["safety"]["safe_stop_publish_s"])

        ns = mock_ns or cfg["topics"]["controller_ns"]
        self.pubs = [
            rospy.Publisher(f"{ns}/{j}_position_controller/command", Float64, queue_size=1)
            for j in self.joints
        ]
        self._fb_lock = threading.Lock()
        self._fb_time = 0.0
        self._fb_q = np.zeros(4); self._fb_dq = np.zeros(4); self._fb_tau = np.zeros(4)
        rospy.Subscriber(cfg["topics"]["joint_states"], JointState, self._on_joint_state,
                         queue_size=1)
        self.start_pose: Optional[np.ndarray] = None
        self.aborted = False
        self._bag_proc: Optional[subprocess.Popen] = None

    # ---- ROS I/O ---- #

    def _on_joint_state(self, msg) -> None:
        try:
            idx = [list(msg.name).index(j) for j in self.joints]
        except ValueError:
            return
        with self._fb_lock:
            self._fb_time = time.monotonic()
            self._fb_q = np.array([msg.position[i] for i in idx], dtype=np.float64)
            if len(msg.velocity) > max(idx):
                self._fb_dq = np.array([msg.velocity[i] for i in idx], dtype=np.float64)
            if len(msg.effort) > max(idx):
                self._fb_tau = np.array([msg.effort[i] for i in idx], dtype=np.float64)

    def _feedback(self):
        with self._fb_lock:
            age = time.monotonic() - self._fb_time if self._fb_time > 0 else 1e9
            return age, self._fb_q.copy(), self._fb_dq.copy(), self._fb_tau.copy()

    def _publish(self, cmd: np.ndarray) -> None:
        cmd = np.clip(cmd, self.hard_lower, self.hard_upper)
        for pub, val in zip(self.pubs, cmd):
            pub.publish(float(val))
        with self.state.lock:
            self.state.cmd = cmd.copy()

    # ---- 安全停 ---- #

    def safe_stop(self, reason: str) -> None:
        if self.aborted:               # 已在停机流程中（另一线程触发），不重复跑
            return
        self.aborted = True
        self.log.error("SAFE-STOP: %s", reason)
        self.state.log_event(f"⛔ 安全停：{reason}")
        with self.state.lock:
            self.state.phase = "SAFE_STOP"
            self.state.watchdog_msg = reason
        if self.start_pose is None:    # 自检早期失败，从未发过指令，无位姿可保持
            with self.state.lock:
                self.state.phase = "ABORTED"
            return
        target = self.start_pose.copy()  # 固定安全位姿，绝不追测量值
        t_end = time.monotonic() + self.safe_stop_publish_s
        rate = self.rospy.Rate(self.rate_hz)
        while time.monotonic() < t_end and not self.rospy.is_shutdown():
            self._publish(target)
            rate.sleep()
        with self.state.lock:
            self.state.phase = "ABORTED"

    # ---- 前置自检 ---- #

    def precheck(self) -> bool:
        with self.state.lock:
            self.state.phase = "PRECHECK"
        self.log.info("等待 /joint_states ...")
        t0 = time.monotonic()
        while True:
            age, q, _, _ = self._feedback()
            if age < 0.2:
                break
            if time.monotonic() - t0 > 10.0:
                self.log.error("前置自检失败：10s 内没有新鲜的 /joint_states")
                return False
            time.sleep(0.1)
        pose_cfg = self.cfg["start_pose"]
        if pose_cfg == "current":
            self.start_pose = q.copy()
            self.log.info("起始位姿 = 当前实际位姿 %s", np.round(q, 4).tolist())
        else:
            self.start_pose = np.asarray(pose_cfg, dtype=np.float64)
            if (np.abs(q - self.start_pose) > 0.1).any():
                self.log.error("前置自检失败：当前位姿 %s 与配置起始位姿 %s 偏差 >0.1rad",
                               np.round(q, 3).tolist(), np.round(self.start_pose, 3).tolist())
                return False
        problems = check_plan_limits(self.groups, self.start_pose, self.joints,
                                     self.hard_lower, self.hard_upper, self.margin)
        for p in problems:
            self.log.error("限位校验失败：%s", p)
        if problems:
            return False
        # 静置基线：发 2s 起始位姿（=当前位姿，安全），量各关节静态跟踪误差。
        # 非激励关节的重力下垂/稳态误差若已逼近 track_err_rad，跑组途中必然误触发
        # 安全停——在这里提前暴露，而不是上机跑到一半才发现。
        rate = self.rospy.Rate(self.rate_hz)
        max_err = np.zeros(4)
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end and not self.rospy.is_shutdown():
            self._publish(self.start_pose)
            age, q, _, _ = self._feedback()
            if age < self.monitor.fb_timeout:
                max_err = np.maximum(max_err, np.abs(self.start_pose - q))
            rate.sleep()
        self.log.info("静置误差基线（保持起始位姿 2s）：%s rad", np.round(max_err, 4).tolist())
        if (max_err > self.monitor.track_err).any():
            self.log.error(
                "前置自检失败：静置跟踪误差 %.3f rad ≥ track_err_rad=%.2f，激励中必然误触发"
                "安全停。原因通常是重力下垂或控制器稳态误差——换姿态或调大 track_err_rad",
                float(max_err.max()), self.monitor.track_err)
            return False
        if (max_err > 0.5 * self.monitor.track_err).any():
            self.log.warning("静置误差 %.3f rad 已超 track_err_rad 的一半，留意运行中误触发",
                             float(max_err.max()))
            self.state.log_event(f"⚠ 静置误差 {float(max_err.max()):.3f}rad 偏大，留意误触发")
        self.log.info("前置自检通过：%d 组，总时长约 %.0fs",
                      len(self.groups), sum(g.duration_s for g in self.groups))
        return True

    # ---- bag 录制 ---- #

    def start_bag(self) -> None:
        if not self.cfg["output"].get("record_bag", True):
            return
        bag_path = self.outdir / "session"
        cmd = ["rosbag", "record", "-O", str(bag_path), "__name:=actuator_id_bag"]
        cmd += list(self.cfg["output"]["bag_topics"])
        self._bag_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
        self.log.info("rosbag record 已启动 -> %s.bag", bag_path)

    def stop_bag(self) -> None:
        if self._bag_proc is None:
            return
        self._bag_proc.send_signal(signal.SIGINT)
        try:
            self._bag_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._bag_proc.kill()
            self.log.warning("rosbag 10s 内未退出，已强制 kill（bag 可能未落盘完整）")
        self._bag_proc = None
        self.log.info("rosbag 已收尾")

    # ---- 组执行 ---- #

    def _wait_gate(self, prompt: str) -> str:
        """分组闸门：返回 'go'/'skip'/'redo'/'abort'/'watchdog:<原因>'。

        等待期间持续发安全位姿，**并保持看门狗在线**——闸门可能等很久（人去调姿态、
        接线、看曲线），这段时间掉线/下垂/自激都必须能被发现。
        """
        with self.state.lock:
            self.state.phase = "GATE"
            self.state.gate_prompt = prompt
        self.monitor.reset()
        rate = self.rospy.Rate(self.rate_hz)
        while not self.rospy.is_shutdown() and not self.aborted:
            self._publish(self.start_pose)
            age, q, dq, tau = self._feedback()
            reason = self.monitor.tick(age, q, dq, tau, self.start_pose)
            if reason:
                return f"watchdog:闸门等待中 {reason}"
            with self.state.lock:
                self.state.q, self.state.dq, self.state.tau = q, dq, tau
                self.state.fb_age_ms = age * 1e3
            key = self.state.pop_key()
            if key in (" ", "\n"):
                return "go"
            if key == "s":
                return "skip"
            if key == "r":
                return "redo"
            if key in ("a", "q"):
                return "abort"
            rate.sleep()
        return "abort"

    def _run_group(self, g: Group) -> Optional[str]:
        """跑一组。返回 None=正常完成，或安全停原因字符串。"""
        j = self.joints.index(g.joint)
        safe_name = g.name.replace("/", "_")
        self.monitor.reset()
        n_total = int(round(g.duration_s * self.rate_hz))
        rec_t = np.zeros(n_total, dtype=np.float64)
        rec_cmd = np.zeros((n_total, 4), dtype=np.float64)
        rec_q = np.zeros((n_total, 4), dtype=np.float64)
        rec_dq = np.zeros((n_total, 4), dtype=np.float64)
        rate = self.rospy.Rate(self.rate_hz)
        seg_iter = [(s, sum(x.duration_s for x in g.segments[:i]))
                    for i, s in enumerate(g.segments)]
        t_start = time.monotonic()
        last_tick = t_start
        i = 0

        def abort_with(reason: str) -> str:
            # 异常中止也把已采数据落盘——安全停那一组恰恰最有诊断价值。
            # 前缀 aborted_ 使其不被 fit_actuator_id.py 的 group_*.npz glob 误当完整组。
            if i > 25:      # 至少 0.5s 数据才值得存
                np.savez(self.outdir / f"aborted_group_{safe_name}.npz",
                         t=rec_t[:i], cmd=rec_cmd[:i], q=rec_q[:i], dq=rec_dq[:i],
                         joint=g.joint, joint_index=j, kind=g.kind, name=g.name,
                         start_pose=self.start_pose,
                         meta=str({**g.meta, "aborted": reason}))
                self.log.info("组 %s 异常中止，前 %d 拍已存 aborted_group_%s.npz",
                              g.name, i, safe_name)
            return reason

        for i in range(n_total):
            if self.rospy.is_shutdown() or self.aborted:
                return abort_with("外部停机（shutdown/其它线程安全停）")
            now = time.monotonic()
            if now - last_tick > self.monitor.loop_overrun + self.dt:
                self.log.warning("控制循环卡顿 %.0fms，跳过补发", (now - last_tick) * 1e3)
            last_tick = now
            t = now - t_start
            seg = seg_iter[-1][0]
            t_in = t - seg_iter[-1][1]
            for s, s_t0 in seg_iter:
                if t < s_t0 + s.duration_s:
                    seg, t_in = s, t - s_t0
                    break
            cmd = self.start_pose.copy()
            cmd[j] += seg.target(max(t_in, 0.0))
            age, q, dq, tau = self._feedback()
            reason = self.monitor.tick(age, q, dq, tau, cmd)
            if reason:
                return abort_with(reason)
            self._publish(cmd)
            rec_t[i], rec_cmd[i], rec_q[i], rec_dq[i] = t, cmd, q, dq
            key = self.state.pop_key()
            if key in ("a", "q"):
                return abort_with(f"操作员中止（按键 {key}）")
            with self.state.lock:
                self.state.q, self.state.dq, self.state.tau = q, dq, tau
                self.state.fb_age_ms = age * 1e3
                self.state.seg_progress = t / max(g.duration_s, 1e-9)
                self.state.watchdog_msg = "OK"
            rate.sleep()
        np.savez(self.outdir / f"group_{safe_name}.npz",
                 t=rec_t, cmd=rec_cmd, q=rec_q, dq=rec_dq,
                 joint=g.joint, joint_index=j, kind=g.kind, name=g.name,
                 start_pose=self.start_pose, meta=str(g.meta))
        with self.state.lock:
            self.state.last_summary = self._group_summary(g, j, rec_t, rec_cmd, rec_q, rec_dq)
        self.log.info("组 %s 完成：%s", g.name, self.state.last_summary)
        return None

    @staticmethod
    def _group_summary(g: Group, j: int, t, cmd, q, dq) -> str:
        if g.kind == "step":
            amp = abs(float(g.meta.get("amplitude", 0.0)))
            step_mask = np.abs(np.diff(cmd[:, j])) > amp * 0.5
            n_steps = int(step_mask.sum())
            err_end = float(np.abs(cmd[-1, j] - q[-1, j]))
            return (f"检出 {n_steps} 次阶跃, 末端稳态误差 {err_end:.4f} rad, "
                    f"max|dq|={np.abs(dq[:, j]).max():.2f} rad/s")
        amp_q = float(np.percentile(np.abs(q[:, j] - q[:, j].mean()), 98))
        amp_c = float(np.percentile(np.abs(cmd[:, j] - cmd[:, j].mean()), 98))
        return (f"响应/指令幅值比 ~{amp_q / max(amp_c, 1e-9):.2f}, "
                f"max|dq|={np.abs(dq[:, j]).max():.2f} rad/s")

    # ---- 主流程 ---- #

    def run(self) -> None:
        try:
            if not self.precheck():
                self.state.log_event("❌ 前置自检失败（详见 session.log），未开始任何激励")
                with self.state.lock:
                    self.state.phase = "ABORTED"
                return
            self.start_bag()
            with self.state.lock:
                self.state.group_total = len(self.groups)
            idx = 0
            while idx < len(self.groups) and not self.aborted:
                g = self.groups[idx]
                with self.state.lock:
                    self.state.group_idx = idx + 1
                    self.state.group_name = g.name
                confirm = "（高频组，确认后开始）" if g.needs_confirm else ""
                prompt = (f"下一组 [{idx+1}/{len(self.groups)}] {g.name} "
                          f"({g.duration_s:.0f}s){confirm} — SPACE 开始 / s 跳过 / "
                          f"r 重跑上一组 / a 安全中止")
                action = self._wait_gate(prompt)
                if action == "abort":
                    self.safe_stop("操作员在闸门处中止")
                    break
                if action.startswith("watchdog:"):
                    self.safe_stop(action.split(":", 1)[1])
                    break
                if action == "skip":
                    self.state.log_event(f"跳过 {g.name}")
                    self.log.info("跳过组 %s", g.name)
                    idx += 1
                    continue
                if action == "redo":
                    idx = max(0, idx - 1)
                    self.state.log_event("重跑上一组")
                    continue
                with self.state.lock:
                    self.state.phase = "RUN"
                self.state.log_event(f"▶ {g.name}")
                reason = self._run_group(g)
                if reason:
                    self.safe_stop(reason)
                    break
                idx += 1
            if not self.aborted:
                with self.state.lock:
                    self.state.phase = "DONE"
                self.state.log_event("✅ 全部组完成")
                self.log.info("SESSION 完成：%d 组", len(self.groups))
        finally:
            self.stop_bag()


# --------------------------------------------------------------------------- #
# TUI                                                                          #
# --------------------------------------------------------------------------- #

def run_tui(stdscr, state: SessionState, joints: List[str]) -> None:
    import curses

    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_RED, -1)
    curses.init_pair(4, curses.COLOR_CYAN, -1)
    C_OK, C_WARN, C_ERR, C_INFO = (curses.color_pair(i) for i in range(1, 5))

    while True:
        key = stdscr.getch()
        if key != -1:
            state.push_key(chr(key) if 0 < key < 256 else "")
        with state.lock:
            phase = state.phase
            snap = (state.group_idx, state.group_total, state.group_name,
                    state.seg_progress, state.q.copy(), state.dq.copy(),
                    state.cmd.copy(), state.fb_age_ms, state.watchdog_msg,
                    list(state.events), state.gate_prompt, state.last_summary)
        try:
            _draw(stdscr, curses, joints, phase, snap,
                  (C_OK, C_WARN, C_ERR, C_INFO))
        except curses.error:
            pass    # 终端太小/正在 resize：跳过这一帧，绝不让绘制异常杀掉 session
        if phase in ("ABORTED", "DONE") and key in (ord("q"), ord("Q")):
            return
        time.sleep(0.1)


def _draw(stdscr, curses, joints, phase, snap, colors) -> None:
    C_OK, C_WARN, C_ERR, C_INFO = colors
    (gi, gt, gname, prog, q, dq, cmd, fb_ms, wd, events, gate, summary) = snap
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    title = f" GP11 执行器辨识  [{phase}]  组 {gi}/{gt}  {gname} "
    stdscr.addnstr(0, 0, title.center(w, "─"), w - 1, C_INFO | curses.A_BOLD)

    row = 2
    stdscr.addnstr(row, 2, f"{'关节':24s} {'cmd':>9s} {'q':>9s} {'err':>8s} {'dq':>8s}",
                   w - 3, curses.A_BOLD)
    for i, name in enumerate(joints):
        err = cmd[i] - q[i]
        color = C_OK if abs(err) < 0.05 else (C_WARN if abs(err) < 0.15 else C_ERR)
        stdscr.addnstr(row + 1 + i, 2,
                       f"{name:24s} {cmd[i]:9.4f} {q[i]:9.4f} {err:8.4f} {dq[i]:8.3f}",
                       w - 3, color)
    row += 6
    fb_color = C_OK if fb_ms < 60 else (C_WARN if fb_ms < 100 else C_ERR)
    stdscr.addnstr(row, 2, f"反馈延迟 {fb_ms:6.0f} ms", w - 3, fb_color)
    wd_color = C_OK if wd in ("OK", "-") else C_ERR
    stdscr.addnstr(row, 26, f"看门狗: {wd}", w - 28, wd_color)
    bar_w = max(10, w - 20)
    filled = int(np.clip(prog, 0, 1) * bar_w)
    stdscr.addnstr(row + 1, 2, f"本组进度 [{'█'*filled}{'░'*(bar_w-filled)}]", w - 3)
    row += 3
    if phase == "GATE" and gate:
        stdscr.addnstr(row, 2, gate, w - 3, C_WARN | curses.A_BOLD)
    elif phase in ("SAFE_STOP", "ABORTED"):
        stdscr.addnstr(row, 2, "⛔ 已安全停：目标冻结在安全位姿。按 q 退出。",
                       w - 3, C_ERR | curses.A_BOLD)
    elif phase == "DONE":
        stdscr.addnstr(row, 2, "✅ 全部完成。按 q 退出。", w - 3, C_OK | curses.A_BOLD)
    if summary:
        stdscr.addnstr(row + 1, 2, f"上组摘要: {summary}", w - 3, C_INFO)
    row += 3
    stdscr.addnstr(row, 2, "最近事件:", w - 3, curses.A_BOLD)
    for i, ev in enumerate(events[-6:]):
        stdscr.addnstr(row + 1 + i, 4, ev, w - 5)
    stdscr.addnstr(h - 1, 0,
                   " SPACE 开始/继续 │ s 跳过 │ r 重跑 │ a 安全中止 │ q 退出 ".center(w - 1, "─"),
                   w - 1, C_INFO)
    stdscr.refresh()


def run_plain(state: SessionState) -> None:
    """--no-tui：闸门用回车确认，事件走 logging 输出。"""
    last_phase = ""
    while True:
        with state.lock:
            phase, gate = state.phase, state.gate_prompt
        if phase == "GATE" and last_phase != "GATE":
            print(f"\n{gate}\n[Enter]=开始 s=跳过 r=重跑 a=中止 > ", end="", flush=True)
            ans = sys.stdin.readline().strip() or " "
            state.push_key(ans[0])
        if phase in ("ABORTED", "DONE"):
            return
        last_phase = phase
        time.sleep(0.2)


# --------------------------------------------------------------------------- #
# main                                                                          #
# --------------------------------------------------------------------------- #

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(_DEFAULT_CFG))
    parser.add_argument("--minimal", action="store_true", help="仅 second_leg 最小协议")
    parser.add_argument("--dry-run", action="store_true", help="不连 ROS，校验并打印计划")
    parser.add_argument("--no-tui", action="store_true")
    parser.add_argument("--mock-ns", default="", help="联调：mock controller 命名空间")
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    groups = build_plan(cfg, args.minimal)

    if args.dry_run:
        total = sum(g.duration_s for g in groups)
        print(f"计划 {len(groups)} 组，总时长 {total:.0f}s（{total/60:.1f}min）：")
        for g in groups:
            mark = " [需确认]" if g.needs_confirm else ""
            print(f"  {g.name:36s} {g.duration_s:6.1f}s 峰值偏移 {g.peak_offset():.3f} rad{mark}")
        pose = cfg["start_pose"]
        if pose == "current":
            print("起始位姿 = current（限位校验将在实机自检时执行）")
        else:
            problems = check_plan_limits(
                groups, np.asarray(pose, dtype=np.float64), list(cfg["joints"]),
                np.asarray(cfg["runtime"]["hard_lower"]), np.asarray(cfg["runtime"]["hard_upper"]),
                float(cfg["runtime"]["limit_margin"]))
            print("限位校验：" + ("通过" if not problems else "\n".join(problems)))
        return

    import rospy
    rospy.init_node("actuator_id_node")

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    outdir = Path(os.path.expanduser(cfg["output"]["dir"])) / stamp
    outdir.mkdir(parents=True, exist_ok=True)
    # rospy.init_node 已配置 root logger，basicConfig 会变空操作 → 用显式 FileHandler
    logger = logging.getLogger("actuator_id")
    logger.setLevel(logging.INFO)
    _fh = logging.FileHandler(str(outdir / "session.log"))
    _fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(_fh)
    logger.propagate = False
    logger.info("session 开始：config=%s minimal=%s mock_ns=%s outdir=%s",
                args.config, args.minimal, args.mock_ns, outdir)
    print(f"⚠️  安全须知：session 全程必须有人守在急停旁。\n日志与数据目录：{outdir}")

    state = SessionState()
    session = ActuatorIdSession(cfg, groups, outdir, state, logger, mock_ns=args.mock_ns)
    rospy.on_shutdown(
        lambda: session.safe_stop("rospy shutdown")
        if not session.aborted and state.phase not in ("DONE", "ABORTED") else None)

    worker = threading.Thread(target=session.run, daemon=True)
    worker.start()
    try:
        if args.no_tui:
            run_plain(state)
        else:
            import curses
            curses.wrapper(run_tui, state, list(cfg["joints"]))
    except KeyboardInterrupt:
        pass
    finally:
        # ABORTED（如前置自检失败，此时可能连 start_pose 都没有）不再重复停机
        if not session.aborted and state.phase not in ("DONE", "ABORTED"):
            session.safe_stop("前台界面退出")
        worker.join(timeout=10)
        print(f"session 结束（{state.phase}）。数据目录：{outdir}")


if __name__ == "__main__":
    main()
