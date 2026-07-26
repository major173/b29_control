#!/usr/bin/env python3
"""
mock_plant_node.py — 无硬件联调用的假被控对象

订阅 <ns>/<joint>_position_controller/command，按「纯延迟 → 速率限制 → 一阶惯性」
模拟执行器，50Hz 发布 /joint_states。

命名空间两种配法（任选，联调结果等价）：
  --ns /b29_controller              激励节点不加 --mock-ns（推荐，配置零改动）
  --ns /mock                        激励节点加 --mock-ns /mock

故障注入（联调看门狗用）：
  --fault none            正常（默认）
  --fault freeze:20       第 20s 起停发 /joint_states（测反馈断流看门狗）
  --fault stuck:20        第 20s 起关节卡死不动（测跟踪误差看门狗）
  --fault runaway:20      第 20s 起注入 3Hz 振荡（测速度异常看门狗）

用法示例见 docs/actuator-id.md「无硬件联调」一节。
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINTS = [
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
]
RATE_HZ = 50.0
DELAY_STEPS = 3          # 60ms 纯延迟
RATE_LIMIT = 6.0         # rad/s（对应 jointSpeedTarget）
TAU_M = 0.05             # s 一阶惯性


class MockPlant:
    def __init__(self, ns: str, fault: str, fault_t: float):
        self.fault, self.fault_t = fault, fault_t
        self.t0 = time.monotonic()
        self.cmd = np.zeros(len(JOINTS))
        self.delay_buf = np.zeros((DELAY_STEPS + 1, len(JOINTS)))
        self.rate_state = np.zeros(len(JOINTS))
        self.q = np.zeros(len(JOINTS))
        self.dq = np.zeros(len(JOINTS))
        for i, j in enumerate(JOINTS):
            rospy.Subscriber(f"{ns}/{j}_position_controller/command", Float64,
                             self._make_cb(i), queue_size=1)
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    def _make_cb(self, idx: int):
        def cb(msg: Float64) -> None:
            self.cmd[idx] = float(msg.data)
        return cb

    def step_and_publish(self, _evt) -> None:
        t = time.monotonic() - self.t0
        faulted = self.fault != "none" and t >= self.fault_t
        dt = 1.0 / RATE_HZ
        # 延迟
        self.delay_buf[1:] = self.delay_buf[:-1]
        self.delay_buf[0] = self.cmd
        target = self.delay_buf[DELAY_STEPS]
        # 速率限制
        step = np.clip(target - self.rate_state, -RATE_LIMIT * dt, RATE_LIMIT * dt)
        self.rate_state = self.rate_state + step
        # 一阶惯性
        alpha = dt / (TAU_M + dt)
        q_new = self.q + alpha * (self.rate_state - self.q)
        if faulted and self.fault == "stuck":
            q_new = self.q            # 关节卡死
        if faulted and self.fault == "runaway":
            q_new = q_new + 0.30 * np.sin(2 * np.pi * 3.0 * t)   # 3Hz 振荡，dq 峰值 ~5.7 rad/s
        self.dq = (q_new - self.q) / dt
        self.q = q_new
        if faulted and self.fault == "freeze":
            return                     # 停发反馈
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(JOINTS)
        msg.position = self.q.tolist()
        msg.velocity = self.dq.tolist()
        msg.effort = [0.0] * len(JOINTS)
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ns", default="/mock")
    parser.add_argument("--fault", default="none",
                        help="none | freeze:<t> | stuck:<t> | runaway:<t>")
    args = parser.parse_args(rospy.myargv()[1:])
    fault, fault_t = "none", 0.0
    if ":" in args.fault:
        fault, t_str = args.fault.split(":", 1)
        fault_t = float(t_str)
    elif args.fault != "none":
        fault, fault_t = args.fault, 0.0

    rospy.init_node("mock_plant_node")
    plant = MockPlant(args.ns, fault, fault_t)
    rospy.Timer(rospy.Duration(1.0 / RATE_HZ), plant.step_and_publish)
    rospy.loginfo("[mock_plant] ns=%s fault=%s@%.0fs", args.ns, fault, fault_t)
    rospy.spin()


if __name__ == "__main__":
    main()
