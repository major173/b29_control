#!/usr/bin/env python3
"""Bridge B29 SMC grip state → GP11 support_plugin/attached topics."""
import rospy
from b29_smc_auto_controller.msg import AutoStateTrace
from std_msgs.msg import Bool


class B29AttachBridge(object):
    def __init__(self):
        self._left_pub = rospy.Publisher(
            "/gp11/support_plugin/left/attached", Bool, queue_size=1, latch=True)
        self._right_pub = rospy.Publisher(
            "/gp11/support_plugin/right/attached", Bool, queue_size=1, latch=True)
        # The commissioned sequence starts with the left/front arm locked and
        # the right/rear arm free.  The SMC crossing side names the free arm,
        # so the MoveIt anchor is always its opposite.
        self._anchor_side = "left"
        self._publish_anchor()
        rospy.Subscriber(
            "/b29_controller/b29_smc_auto_controller/state_trace",
            AutoStateTrace, self._callback, queue_size=1)
        rospy.loginfo(
            "B29 attach bridge ready (initial roles: left locked, right free)"
        )

    def _publish_anchor(self):
        self._left_pub.publish(Bool(data=self._anchor_side == "left"))
        self._right_pub.publish(Bool(data=self._anchor_side == "right"))

    def _callback(self, msg):
        anchor_side = {
            "Right": "left",
            "Left": "right",
        }.get(msg.obstacle_crossing_side)
        if not anchor_side or anchor_side == self._anchor_side:
            return

        previous = self._anchor_side
        self._anchor_side = anchor_side
        self._publish_anchor()
        rospy.logwarn(
            "Confirmed crossing-role switch: anchor %s -> %s; stage=%s free_side=%s",
            previous,
            anchor_side,
            msg.obstacle_crossing_stage,
            msg.obstacle_crossing_side,
        )


def main():
    rospy.init_node("gp11_b29_attach_bridge")
    B29AttachBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
