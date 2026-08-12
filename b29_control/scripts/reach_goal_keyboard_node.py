#!/usr/bin/env python3
"""
reach_goal_keyboard_node.py

订阅 /joint_states，发布 marker 可视化目标点（红）和当前末端中点（绿）。
启动时将 _goal_ref 初始化为当前末端在 obs_ref 坐标系下的位置，
使策略接管时臂保持不动。
"""

from __future__ import annotations

import numpy as np
import rospy
import sys
import threading
import time
from pathlib import Path
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker

# reach_policy: ObservationBuilder共用，load_fk_model用训练侧URDF计算FK
_SCRIPTS_DIR = Path(__file__).resolve().parent
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
from reach_policy import load_fk_model  # noqa: E402

# anchor_side → (obs_ref_link, tool_link)
_ANCHOR_TO_LINKS = {
    "left": ("left_second_leg", "r_gripper_left_uprod"),
    "right": ("right_second_leg", "l_gripper_left_up"),
}

ANCHOR_LINK = "left_second_leg"
TOOL_LINK = "r_gripper_left_uprod"
OBS_REF_FRAME = ANCHOR_LINK
STEP_DEFAULT = 0.02

_RST = "\033[0m"
_BOLD = "\033[1m"
_RED = "\033[31m"
_GRN = "\033[32m"
_YLW = "\033[33m"
_CYN = "\033[36m"
_HOME = "\033[H"
_ED = "\033[J"


def _write(text: str) -> None:
    sys.stdout.write(text.replace("\n", "\r\n"))
    sys.stdout.flush()


class ReachGoalKeyboardNode:
    def __init__(self, kinematics=None, tool_left_body=None, tool_right_body=None,
                 active_joint_names=None,
                 obs_ref_origin=None, obs_ref_rot=None) -> None:
        rospy.init_node("reach_goal_keyboard", anonymous=False)

        self._spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        self._spin_thread.start()

        self._lock = threading.RLock()

        self._goal_ref = np.zeros(3, dtype=np.float32)
        self._step = STEP_DEFAULT
        self._log_msgs: list[str] = []
        self._initialized = False

        # FK 模型（训练侧 URDF）
        self._kinematics = kinematics
        self._tool_left_body = tool_left_body
        self._tool_right_body = tool_right_body
        self._active_joint_names = active_joint_names or []
        self._latest_q: np.ndarray | None = None
        # obs_ref nominal transform（训练侧固定，与 anchor_side 对应的 obs_ref_body 在 q=0 时的变换）
        # 用 nominal 而非动态 FK，保证与训练侧坐标系一致
        self._obs_ref_origin: np.ndarray | None = obs_ref_origin  # shape (3,)
        self._obs_ref_rot: np.ndarray | None = obs_ref_rot        # shape (3, 3)

        self._pub_goal = rospy.Publisher(
            "/gp11/rl/target_point_local", Float64MultiArray, queue_size=1, latch=True
        )
        self._pub_goal_marker = rospy.Publisher("/gp11/rl/goal_marker", Marker, queue_size=1)
        self._pub_tool_marker = rospy.Publisher("/gp11/rl/tool_marker", Marker, queue_size=1)

        if self._kinematics is not None:
            from sensor_msgs.msg import JointState
            rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=1)

        self._running = True
        threading.Thread(target=self._marker_loop, daemon=True).start()

        time.sleep(1.5)
        self._init_goal_from_tool()

    # ------------------------------------------------------------------ #
    # Joint state
    # ------------------------------------------------------------------ #

    def _on_joint_states(self, msg) -> None:
        if not self._active_joint_names:
            return
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q = np.array([msg.position[name_to_idx[n]] for n in self._active_joint_names],
                         dtype=np.float32)
            with self._lock:
                self._latest_q = q
        except KeyError:
            pass

    def _get_tool_pos_ref(self):
        """运动端在训练侧 obs_ref 坐标系下的位置，用 FK 计算（不依赖 TF）。

        坐标系原点使用 nominal anchor transform（obs_ref_origin/rot），
        与训练侧 build_reference_frame_from_nominal 保持一致。
        """
        with self._lock:
            q = self._latest_q
        if q is None or self._kinematics is None:
            return None
        if self._obs_ref_origin is None or self._obs_ref_rot is None:
            return None
        try:
            transforms = self._kinematics._compute_link_transforms(q)
            T_l = transforms[self._tool_left_body]
            T_r = transforms[self._tool_right_body]
            tool_base = 0.5 * (T_l[:3, 3] + T_r[:3, 3])
            # 用 nominal obs_ref（与训练侧一致），而非动态 T_anchor
            return (self._obs_ref_rot.T @ (tool_base - self._obs_ref_origin)).astype(np.float32)
        except Exception:
            return None

    def _wall_stamp(self):
        t = rospy.Time.now()
        return t if t.to_sec() > 0 else rospy.Time.from_sec(time.time())

    # ------------------------------------------------------------------ #
    # 初始化：将 goal_ref 设为当前末端在 obs_ref 坐标系下的位置
    # ------------------------------------------------------------------ #

    def _init_goal_from_tool(self) -> None:
        deadline = time.time() + 10.0
        while time.time() < deadline and not rospy.is_shutdown():
            tool_ref = self._get_tool_pos_ref()
            if tool_ref is not None:
                with self._lock:
                    self._goal_ref = tool_ref.astype(np.float32)
                    self._initialized = True
                self._publish_goal()
                self._log(f"{_GRN}初始化目标 [{tool_ref[0]:+.3f},{tool_ref[1]:+.3f},{tool_ref[2]:+.3f}]{_RST}")
                return
            time.sleep(0.1)
        self._log(f"{_YLW}FK 初始化超时，goal_ref 保持 [0,0,0]{_RST}")
        self._publish_goal()

    # ------------------------------------------------------------------ #
    # Marker loop
    # ------------------------------------------------------------------ #

    def _marker_loop(self) -> None:
        while self._running and not rospy.is_shutdown():
            tool_ref = self._get_tool_pos_ref()
            stamp = self._wall_stamp()

            # 红球：目标点，obs_ref 坐标系
            with self._lock:
                goal_ref = self._goal_ref.copy()
            self._pub_goal_marker.publish(
                _sphere(0, goal_ref, (0.95, 0.15, 0.15, 0.85), OBS_REF_FRAME, stamp, 0.045)
            )

            # 绿球：训练侧 FK 计算的当前末端位置
            if tool_ref is not None:
                self._pub_tool_marker.publish(
                    _sphere(1, tool_ref, (0.15, 0.90, 0.20, 0.85), OBS_REF_FRAME, stamp, 0.030)
                )

            time.sleep(0.02)  # 50Hz

    # ------------------------------------------------------------------ #
    # 发布目标
    # ------------------------------------------------------------------ #

    def _publish_goal(self) -> None:
        with self._lock:
            data = [float(v) for v in self._goal_ref]
        self._pub_goal.publish(Float64MultiArray(data=data))

    # ------------------------------------------------------------------ #
    # 操作
    # ------------------------------------------------------------------ #

    def _try_move(self, delta: np.ndarray, label: str) -> None:
        with self._lock:
            self._goal_ref = (self._goal_ref + delta).astype(np.float32)
        self._publish_goal()
        self._log(f"{_GRN}{label}{_RST}")

    def _log(self, msg: str) -> None:
        with self._lock:
            self._log_msgs.append(msg)
            if len(self._log_msgs) > 4:
                self._log_msgs.pop(0)

    # ------------------------------------------------------------------ #
    # UI
    # ------------------------------------------------------------------ #

    def _render_ui(self) -> None:
        tool_ref = self._get_tool_pos_ref()
        with self._lock:
            goal = self._goal_ref.copy()
            step = self._step
            logs = list(self._log_msgs)
            ready = self._initialized

        fk_status = f"{_GRN}OK{_RST}" if tool_ref is not None else f"{_RED}等待 FK...{_RST}"

        dist_str = "—"
        if tool_ref is not None:
            dist = float(np.linalg.norm(goal - tool_ref))
            c = _GRN if dist < 0.05 else (_YLW if dist < 0.20 else _RED)
            dist_str = f"{c}{dist:.3f} m{_RST}"

        init_flag = f"{_GRN}已初始化{_RST}" if ready else f"{_YLW}等待初始化...{_RST}"

        lines = [
            f"{_HOME}{_ED}",
            f"{_BOLD}{_CYN}+--------------------------------------+{_RST}",
            f"{_BOLD}{_CYN}|  GP11 Reach Goal Keyboard Control    |{_RST}",
            f"{_BOLD}{_CYN}+--------------------------------------+{_RST}",
            f"",
            f"  FK ({OBS_REF_FRAME}) : {fk_status}   {init_flag}",
            f"  goal [obs_ref]: [{goal[0]:+.3f}  {goal[1]:+.3f}  {goal[2]:+.3f}]",
            f"  goal ↔ tool dist : {dist_str}",
            f"  step             : {_BOLD}{step:.3f} m{_RST}",
            f"",
            f"  {_BOLD}W/S{_RST} X+/-   {_BOLD}A/D{_RST} Y+/-   {_BOLD}Q/E{_RST} Z+/-",
            f"  {_BOLD}R{_RST} reset到末端  {_BOLD}[{_RST} step-   {_BOLD}]{_RST} step+   {_BOLD}^C{_RST} quit",
            f"",
            f"  red=goal  green=tool({TOOL_LINK})  frame={OBS_REF_FRAME}",
            f"",
            f"  --- log ---",
        ]
        for entry in (logs or ["(no ops yet)"]):
            lines.append(f"    {entry}")
        _write("\n".join(lines))

    # ------------------------------------------------------------------ #
    # 键盘主循环
    # ------------------------------------------------------------------ #

    def run_keyboard(self) -> None:
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        ui_thread = threading.Thread(target=self._ui_loop, daemon=True)

        try:
            tty.setraw(fd)
            ui_thread.start()
            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)
                if ch in ('\x03', '\x04'):
                    break
                s = self._step
                if ch in ('w', 'W'):
                    self._try_move(np.array([s, 0, 0], np.float32), f"X +{s:.3f}")
                elif ch in ('s', 'S'):
                    self._try_move(np.array([-s, 0, 0], np.float32), f"X -{s:.3f}")
                elif ch in ('a', 'A'):
                    self._try_move(np.array([0, s, 0], np.float32), f"Y +{s:.3f}")
                elif ch in ('d', 'D'):
                    self._try_move(np.array([0, -s, 0], np.float32), f"Y -{s:.3f}")
                elif ch in ('q', 'Q'):
                    self._try_move(np.array([0, 0, s], np.float32), f"Z +{s:.3f}")
                elif ch in ('e', 'E'):
                    self._try_move(np.array([0, 0, -s], np.float32), f"Z -{s:.3f}")
                elif ch == 'r':
                    tool_ref = self._get_tool_pos_ref()
                    if tool_ref is not None:
                        with self._lock:
                            self._goal_ref = tool_ref.astype(np.float32)
                        self._publish_goal()
                        self._log(f"{_CYN}reset → [{tool_ref[0]:+.3f},{tool_ref[1]:+.3f},{tool_ref[2]:+.3f}]{_RST}")
                    else:
                        self._log(f"{_YLW}reset 失败：TF 不可用{_RST}")
                elif ch == '[':
                    with self._lock:
                        self._step = max(self._step - 0.005, 0.005)
                    self._log(f"step → {self._step:.3f}m")
                elif ch == ']':
                    with self._lock:
                        self._step = min(self._step + 0.005, 0.10)
                    self._log(f"step → {self._step:.3f}m")
        finally:
            self._running = False
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            _write("\033[2J\033[H")

    def _ui_loop(self) -> None:
        while self._running and not rospy.is_shutdown():
            self._render_ui()
            time.sleep(0.1)


# ------------------------------------------------------------------ #
# Marker 工具
# ------------------------------------------------------------------ #

def _sphere(mid, pos, rgba, frame_id, stamp, scale=0.04) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "reach_goal"
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2])
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.lifetime = rospy.Duration(0.2)
    return m


# ------------------------------------------------------------------ #

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--anchor_side", choices=("left", "right"), default="left",
                        help="固定端：left=左臂固定右臂活动，right=右臂固定左臂活动")
    args, _ = parser.parse_known_args(
        [a for a in sys.argv[1:] if not a.startswith("__")]
    )

    global ANCHOR_LINK, TOOL_LINK, OBS_REF_FRAME
    ANCHOR_LINK, TOOL_LINK = _ANCHOR_TO_LINKS[args.anchor_side]
    OBS_REF_FRAME = ANCHOR_LINK

    # 加载训练侧 FK 模型（包内预置 URDF，不依赖外部路径）
    _THIS_DIR = Path(__file__).resolve().parent
    _ACTIVE_JOINT_NAMES = [
        "left_first_leg_joint", "left_second_leg_joint",
        "right_first_leg_joint", "right_second_leg_joint",
    ]
    km, obs_ref_origin, obs_ref_rot, tool_left, tool_right = load_fk_model(
        args.anchor_side,
        urdf_dir=_THIS_DIR.parent / "models" / "gp11_urdf",
    )
    if km is not None:
        print(f"[keyboard] FK loaded: anchor={args.anchor_side}")
    else:
        print("[keyboard] FK load failed, _get_tool_pos_ref will return None")

    node = ReachGoalKeyboardNode(km, tool_left, tool_right, _ACTIVE_JOINT_NAMES,
                                 obs_ref_origin=obs_ref_origin, obs_ref_rot=obs_ref_rot)
    node.run_keyboard()


if __name__ == "__main__":
    main()
