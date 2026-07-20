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
        # Default: assume left clamped (real robot, STM32 grip feedback wip).
        self._left_pub.publish(Bool(data=True))
        self._right_pub.publish(Bool(data=False))
        rospy.Subscriber(
            "/b29_controller/b29_smc_auto_controller/state_trace",
            AutoStateTrace, self._callback, queue_size=1)
        rospy.loginfo("B29 attach bridge ready (default: left attached)")

    def _callback(self, msg):
        # Once STM32 feedback is live, use real grip state.
        if msg.grip_confirmed:
            self._left_pub.publish(Bool(data=True))
            self._right_pub.publish(Bool(data=False))


def main():
    rospy.init_node("gp11_b29_attach_bridge")
    B29AttachBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
