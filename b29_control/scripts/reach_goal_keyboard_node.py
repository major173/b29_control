#!/usr/bin/env python3
"""
reach_goal_keyboard_node.py

订阅/tf并发布marker可视化目标点和当前判定点的位置
"""

from __future__ import annotations

import numpy as np
import rospy
import sys
import tf
import threading
import time
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker

# anchor_side → (固定端 ref link, 运动端 tool link)
# 与 gp11_reach_runtime._RUNTIME_SIDE_SPECS 对应：
#   anchor_side=left  → 左臂固定(ref=left_second_leg),  右臂活动(tool=right_rod)
#   anchor_side=right → 右臂固定(ref=right_second_leg), 左臂活动(tool=left_rod)
_ANCHOR_TO_LINKS = {
    "left": ("left_second_leg", "right_rod"),
    "right": ("right_second_leg", "left_rod"),
}
ANCHOR_LINK = "left_second_leg"  # 由 main() 根据 --anchor_side 覆盖
TOOL_LINK = "right_rod"
WORLD_FRAME = "world"
STEP_DEFAULT = 0.02
GOAL_BOUNDS = 0.80

# ANSI（不含 \n，输出时统一加 \r\n）
_RST = "\033[0m"
_BOLD = "\033[1m"
_RED = "\033[31m"
_GRN = "\033[32m"
_YLW = "\033[33m"
_CYN = "\033[36m"
_HOME = "\033[H"  # 光标回左上角（不清屏，减少闪烁）
_ED = "\033[J"  # 清除光标以下


def _write(text: str) -> None:
    """raw 模式安全输出：\n → \r\n。"""
    sys.stdout.write(text.replace("\n", "\r\n"))
    sys.stdout.flush()


class ReachGoalKeyboardNode:
    def __init__(self) -> None:
        rospy.init_node("reach_goal_keyboard", anonymous=False)

        # spin 在独立线程，TF listener 的订阅回调才能运行
        self._spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        self._spin_thread.start()

        self._tf = tf.TransformListener()
        self._lock = threading.Lock()

        self._goal_ref = np.zeros(3, dtype=np.float32)
        self._step = STEP_DEFAULT
        self._log_msgs: list[str] = []

        self._pub_goal = rospy.Publisher(
            "/gp11/rl/target_point_local", Float64MultiArray, queue_size=1, latch=True
        )
        self._pub_goal_marker = rospy.Publisher(
            "/gp11/rl/goal_marker", Marker, queue_size=1
        )
        self._pub_tool_marker = rospy.Publisher(
            "/gp11/rl/tool_marker", Marker, queue_size=1
        )

        self._running = True
        threading.Thread(target=self._marker_loop, daemon=True).start()

        time.sleep(0.5)  # 等 TF 初始化
        self._publish_goal()

    # ------------------------------------------------------------------ #
    # TF
    # ------------------------------------------------------------------ #

    def _get_ref_pose(self):
        try:
            trans, q = self._tf.lookupTransform(WORLD_FRAME, ANCHOR_LINK, rospy.Time(0))
            x, y, z, w = q
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ], dtype=np.float32)
            return np.array(trans, dtype=np.float32), R
        except Exception:
            return None

    def _wall_stamp(self):
        t = rospy.Time.now()
        return t if t.to_sec() > 0 else rospy.Time.from_sec(time.time())

    # ------------------------------------------------------------------ #
    # Marker
    # ------------------------------------------------------------------ #

    def _get_tool_pos(self):
        """运动端 right_rod 在 world 坐标系中的位置。"""
        try:
            trans, _ = self._tf.lookupTransform(WORLD_FRAME, TOOL_LINK, rospy.Time(0))
            return np.array(trans, dtype=np.float32)
        except Exception:
            return None

    def _marker_loop(self) -> None:
        while self._running and not rospy.is_shutdown():
            pose = self._get_ref_pose()
            tool_pos = self._get_tool_pos()
            stamp = self._wall_stamp()
            if pose is not None:
                origin, rot = pose
                with self._lock:
                    goal_ref = self._goal_ref.copy()
                goal_world = rot @ goal_ref + origin
                self._pub_goal_marker.publish(
                    _sphere(0, goal_world, (0.95, 0.15, 0.15, 0.85), WORLD_FRAME, stamp, 0.045)
                )
            if tool_pos is not None:
                self._pub_tool_marker.publish(
                    _sphere(1, tool_pos, (0.15, 0.90, 0.20, 0.85), WORLD_FRAME, stamp, 0.030)
                )
            time.sleep(0.05)

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
            new_goal = self._goal_ref + delta
            if np.any(np.abs(new_goal) > GOAL_BOUNDS):
                self._log(f"{_YLW}超出 ±{GOAL_BOUNDS}m，忽略{_RST}")
                return
            self._goal_ref = new_goal.copy()
        self._publish_goal()
        self._log(f"{_GRN}{label}{_RST}")

    def _log(self, msg: str) -> None:
        with self._lock:
            self._log_msgs.append(msg)
            if len(self._log_msgs) > 4:
                self._log_msgs.pop(0)

    # ------------------------------------------------------------------ #
    # UI（raw 模式，\r\n 换行）
    # ------------------------------------------------------------------ #

    def _render_ui(self) -> None:
        pose = self._get_ref_pose()
        with self._lock:
            goal = self._goal_ref.copy()
            step = self._step
            logs = list(self._log_msgs)

        tf_status = f"{_GRN}OK{_RST}" if pose is not None else f"{_RED}等待 TF...{_RST}"
        dist = float(np.linalg.norm(goal))

        goal_world_str = "—"
        if pose is not None:
            origin, rot = pose
            gw = rot @ goal + origin
            goal_world_str = f"[{gw[0]:+.3f}  {gw[1]:+.3f}  {gw[2]:+.3f}]"

        dist_color = _GRN if dist < 0.3 else (_YLW if dist < 0.6 else _RED)

        lines = [
            f"{_HOME}{_ED}",
            f"{_BOLD}{_CYN}+--------------------------------------+{_RST}",
            f"{_BOLD}{_CYN}|  GP11 Reach Goal Keyboard Control    |{_RST}",
            f"{_BOLD}{_CYN}+--------------------------------------+{_RST}",
            f"",
            f"  TF ({ANCHOR_LINK}) : {tf_status}",
            f"  goal [ref]       : [{goal[0]:+.3f}  {goal[1]:+.3f}  {goal[2]:+.3f}]",
            f"  goal [world]     : {goal_world_str}",
            f"  dist to ref orig : {dist_color}{dist:.3f} m{_RST}",
            f"  step             : {_BOLD}{step:.3f} m{_RST}",
            f"",
            f"  {_BOLD}W/S{_RST} X+/-   {_BOLD}A/D{_RST} Y+/-   {_BOLD}Q/E{_RST} Z+/-",
            f"  {_BOLD}R{_RST} reset   {_BOLD}[{_RST} step-   {_BOLD}]{_RST} step+   {_BOLD}^C{_RST} quit",
            f"",
            f"  RViz: Fixed Frame=world",
            f"  red=goal_marker({ANCHOR_LINK} ref)  green=tool_marker({TOOL_LINK})",
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

        ui_thread = threading.Thread(
            target=self._ui_loop, daemon=True
        )

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
                    with self._lock:
                        self._goal_ref = np.zeros(3, dtype=np.float32)
                    self._publish_goal()
                    self._log(f"{_CYN}reset -> [0,0,0]{_RST}")
                elif ch == '[':
                    with self._lock:
                        self._step = max(self._step - 0.005, 0.005)
                    self._log(f"step -> {self._step:.3f}m")
                elif ch == ']':
                    with self._lock:
                        self._step = min(self._step + 0.005, 0.10)
                    self._log(f"step -> {self._step:.3f}m")
        finally:
            self._running = False
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            _write("\033[2J\033[H")  # 退出时清屏

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

    global ANCHOR_LINK, TOOL_LINK
    ANCHOR_LINK, TOOL_LINK = _ANCHOR_TO_LINKS[args.anchor_side]

    node = ReachGoalKeyboardNode()
    node.run_keyboard()


if __name__ == "__main__":
    main()
