#!/usr/bin/env python3
"""Read-only audit of planned joint direction versus live B29 feedback."""

import math
import threading

import rospy
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String


LEG_JOINTS = (
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
)

# B29 and STM32 now share one robot-joint coordinate for both commands and
# feedback.  The audit mirrors the serial-bound values only for display; it
# never publishes the computed values.
B29_SERIAL_COMMAND_SIGN = {
    "left_first_leg_joint": 1.0,
    "left_second_leg_joint": 1.0,
    "right_first_leg_joint": 1.0,
    "right_second_leg_joint": 1.0,
}


class PlanDirectionAudit(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._positions = {}
        self._anchor = "unknown"
        self._large_flip_threshold = float(
            rospy.get_param("~large_flip_threshold", math.radians(150.0))
        )
        self._large_flip_right_second_target = float(
            rospy.get_param("~large_flip_right_second_target", math.pi)
        )
        rospy.Subscriber("/joint_states", JointState, self._joint_state_cb, queue_size=10)
        rospy.Subscriber(
            "/gp11_moveit/runtime_anchor", String, self._anchor_cb, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param(
                "~display_trajectory_topic", "/move_group/display_planned_path"
            ),
            DisplayTrajectory,
            self._trajectory_cb,
            queue_size=5,
        )
        rospy.logwarn(
            "[plan direction audit] READ-ONLY; execution is disabled by default. "
            "Drag the goal and click Plan only."
        )

    def _joint_state_cb(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name in LEG_JOINTS and math.isfinite(position):
                    self._positions[name] = float(position)

    def _anchor_cb(self, msg):
        with self._lock:
            self._anchor = str(msg.data).strip().lower() or "unknown"

    @staticmethod
    def _degrees(value):
        return math.degrees(float(value))

    def _trajectory_cb(self, msg):
        if not msg.trajectory:
            rospy.logwarn("[plan direction audit] displayed trajectory is empty")
            return
        trajectory = msg.trajectory[-1].joint_trajectory
        if not trajectory.points or not trajectory.joint_names:
            rospy.logwarn("[plan direction audit] joint trajectory is empty")
            return

        first = trajectory.points[0]
        final = trajectory.points[-1]
        if (len(first.positions) != len(trajectory.joint_names) or
                len(final.positions) != len(trajectory.joint_names)):
            rospy.logerr("[plan direction audit] malformed trajectory point sizes")
            return

        planned_start = dict(zip(trajectory.joint_names, first.positions))
        planned_goal = dict(zip(trajectory.joint_names, final.positions))
        with self._lock:
            live = dict(self._positions)
            anchor = self._anchor

        locked_second = {
            "left": "left_second_leg_joint",
            "right": "right_second_leg_joint",
        }.get(anchor, "")
        rospy.logwarn(
            "[plan direction audit] anchor=%s locked_second=%s points=%d duration=%.3fs",
            anchor,
            locked_second or "unknown",
            len(trajectory.points),
            final.time_from_start.to_sec(),
        )
        left_second = "left_second_leg_joint"
        if (anchor == "left" and left_second in planned_goal and left_second in live):
            raw_delta = planned_goal[left_second] - live[left_second]
            shortest_delta = math.atan2(math.sin(raw_delta), math.cos(raw_delta))
            if abs(shortest_delta) > self._large_flip_threshold:
                rospy.logwarn(
                    "[plan direction audit] RIGHT_SECOND_OVERRIDE_WILL_TRIGGER: "
                    "|left_second delta|=%.3fdeg > %.3fdeg; Execute will replace "
                    "MoveIt right_second with a smooth target of %+.3fdeg",
                    abs(self._degrees(shortest_delta)),
                    self._degrees(self._large_flip_threshold),
                    self._degrees(self._large_flip_right_second_target),
                )
            else:
                rospy.logwarn(
                    "[plan direction audit] RIGHT_SECOND_OVERRIDE_WILL_NOT_TRIGGER: "
                    "|left_second delta|=%.3fdeg <= %.3fdeg; Execute will preserve "
                    "MoveIt right_second",
                    abs(self._degrees(shortest_delta)),
                    self._degrees(self._large_flip_threshold),
                )
        for name in LEG_JOINTS:
            if name not in planned_goal:
                rospy.logwarn("[plan direction audit] %s absent from trajectory", name)
                continue
            start = float(planned_start.get(name, planned_goal[name]))
            goal = float(planned_goal[name])
            current = live.get(name)
            marker = " LOCKED_SECOND" if name == locked_second else ""
            if current is None:
                rospy.logwarn(
                    "[plan direction audit]%s %s start=%+.3fdeg goal=%+.3fdeg "
                    "plan_delta=%+.3fdeg live=missing",
                    marker,
                    name,
                    self._degrees(start),
                    self._degrees(goal),
                    self._degrees(goal - start),
                )
            else:
                rospy.logwarn(
                    "[plan direction audit]%s %s live=%+.3fdeg start=%+.3fdeg "
                    "goal=%+.3fdeg plan_delta=%+.3fdeg goal_minus_live=%+.3fdeg",
                    marker,
                    name,
                    self._degrees(current),
                    self._degrees(start),
                    self._degrees(goal),
                    self._degrees(goal - start),
                    self._degrees(goal - current),
                )
            serial_goal = B29_SERIAL_COMMAND_SIGN[name] * goal
            serial_start = B29_SERIAL_COMMAND_SIGN[name] * start
            rospy.logwarn(
                "[plan direction audit]%s %s B29_serial_start=%+.3fdeg "
                "B29_serial_goal=%+.3fdeg serial_delta=%+.3fdeg sign=%+.0f",
                marker,
                name,
                self._degrees(serial_start),
                self._degrees(serial_goal),
                self._degrees(serial_goal - serial_start),
                B29_SERIAL_COMMAND_SIGN[name],
            )


def main():
    rospy.init_node("gp11_plan_direction_audit")
    PlanDirectionAudit()
    rospy.spin()


if __name__ == "__main__":
    main()
