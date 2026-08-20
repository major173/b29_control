#!/usr/bin/env python3
"""Continuously run two commissioned flip passes for each obstacle cycle."""

import sys

import rospy

from gp11.single_flip_client import SingleFlipClient


OPPOSITE_SIDE = {
    "Left": "Right",
    "Right": "Left",
}


def run_obstacle_cycle(client, obstacle_index, pass_count,
                       execution_timeout, handoff_timeout):
    """Run the existing commissioned passes once, with cycle-local side state."""
    first_crossing_side = None
    final_crossing_side = None
    for pass_index in range(1, pass_count + 1):
        expected_side = (
            None if first_crossing_side is None else
            OPPOSITE_SIDE[first_crossing_side]
        )
        crossing_side = client.wait_for_disconnect_side(
            timeout=None,
            expected_side=expected_side,
        )
        if crossing_side is None:
            failure = (
                "Obstacle {}: SMC did not reach DisconnectDoneWaitFlip on "
                "the expected side {}"
            ).format(
                obstacle_index,
                expected_side or "selected from SMC cruise direction",
            )
            return 1, failure, first_crossing_side, final_crossing_side
        if first_crossing_side is None:
            first_crossing_side = crossing_side
        final_crossing_side = crossing_side
        rospy.loginfo(
            "Obstacle %d pass %d/%d will follow SMC latched side %s%s",
            obstacle_index,
            pass_index,
            pass_count,
            crossing_side,
            " (opposite of first pass)" if pass_index > 1 else "",
        )
        result = client.run(
            startup_timeout=None,
            state_timeout=None,
            execution_timeout=execution_timeout,
            handoff_timeout=handoff_timeout,
            crossing_side=crossing_side,
            first_crossing_side=first_crossing_side,
            pass_index=pass_index,
            pass_count=pass_count,
        )
        if result != 0:
            return (
                result,
                client.last_failure or
                "Obstacle {} pass {} failed".format(
                    obstacle_index, pass_index
                ),
                first_crossing_side,
                final_crossing_side,
            )
        if pass_index < pass_count:
            rospy.logwarn(
                "Obstacle %d pass %d/%d handed off successfully; waiting "
                "for lower-level completion, confirmed regrip, anchor "
                "switch, and next disconnect",
                obstacle_index, pass_index, pass_count,
            )
    return 0, "", first_crossing_side, final_crossing_side


def main():
    rospy.init_node("gp11_automatic_flip")
    pass_count = int(rospy.get_param("~pass_count", 2))
    if pass_count not in (1, 2):
        rospy.logfatal("pass_count must be 1 or 2, got %d", pass_count)
        return 1
    continuous_obstacles = bool(rospy.get_param(
        "~continuous_obstacles", True
    ))
    if continuous_obstacles and pass_count != 2:
        rospy.logfatal(
            "continuous_obstacles requires the commissioned two-pass cycle; "
            "got pass_count=%d", pass_count,
        )
        return 1
    rospy.loginfo(
        "Automatic disconnect-to-flip integration is active for %d pass(es) "
        "per obstacle; continuous_obstacles=%s; SMC cruise direction selects "
        "each obstacle's first side",
        pass_count,
        continuous_obstacles,
    )
    client = SingleFlipClient()
    execution_timeout = float(rospy.get_param("~execution_timeout", 150.0))
    handoff_timeout = float(rospy.get_param("~handoff_timeout", 5.0))
    failure = ""
    result = 1
    obstacle_index = 1
    while not rospy.is_shutdown():
        result, failure, first_crossing_side, final_crossing_side = (
            run_obstacle_cycle(
                client,
                obstacle_index,
                pass_count,
                execution_timeout,
                handoff_timeout,
            )
        )
        if result != 0:
            break
        rospy.loginfo(
            "Obstacle %d: all %d commissioned MoveIt flip pass(es) reached "
            "lower-level RemoteControl handoff",
            obstacle_index,
            pass_count,
        )
        if not continuous_obstacles:
            return 0

        rospy.logwarn(
            "Obstacle %d second handoff succeeded; waiting for final remote "
            "completion, confirmed regrip, CompleteWaitObstacleClear, and "
            "the obstacle falling edge before accepting another obstacle",
            obstacle_index,
        )
        if not client.wait_for_obstacle_cycle_reset(
                first_crossing_side,
                final_crossing_side,
                timeout=None):
            if rospy.is_shutdown():
                return 0
            result = 13
            failure = (
                client.last_failure or
                "Obstacle {} did not complete its normal SMC reset".format(
                    obstacle_index
                )
            )
            break
        rospy.loginfo(
            "Obstacle %d cycle reset is complete; previous side selection "
            "has been discarded and the next obstacle will be selected from "
            "new SMC cruise direction",
            obstacle_index,
        )
        obstacle_index += 1

    if rospy.is_shutdown():
        return 0

    failure = client.last_failure or failure or "unknown automatic flip failure"
    reminder_period = max(
        1.0, float(rospy.get_param("~failure_reminder_period", 5.0))
    )
    banner = (
        "AUTOMATIC FLIP STOPPED SAFELY (code {}): {} | "
        "PlannerControl remains in safe hold; restart start.launch after inspection."
    ).format(result, failure)
    rospy.logfatal("%s", banner)
    while not rospy.is_shutdown():
        rospy.sleep(reminder_period)
        if not rospy.is_shutdown():
            rospy.logfatal("%s", banner)
    return result


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)
