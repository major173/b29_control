#!/usr/bin/env python3

import os
import yaml
import rospy

from b29_smc_auto_controller.msg import AutoDebugOverride


FIELD_MAP = {
    "auto_start_requested": AutoDebugOverride.FIELD_AUTO_START_REQUESTED,
    "manual_reset_requested": AutoDebugOverride.FIELD_MANUAL_RESET_REQUESTED,
    "emergency_stop": AutoDebugOverride.FIELD_EMERGENCY_STOP,
    "lower_alive": AutoDebugOverride.FIELD_LOWER_ALIVE,
    "imu_ready": AutoDebugOverride.FIELD_IMU_READY,
    "posture_ready": AutoDebugOverride.FIELD_POSTURE_READY,
    "grip_confirmed": AutoDebugOverride.FIELD_GRIP_CONFIRMED,
    "joint_fault": AutoDebugOverride.FIELD_JOINT_FAULT,
    "grip_fault": AutoDebugOverride.FIELD_GRIP_FAULT,
    "obstacle_detected": AutoDebugOverride.FIELD_OBSTACLE_DETECTED,
    "obstacle_type": AutoDebugOverride.FIELD_OBSTACLE_TYPE,
    "classification_stable": AutoDebugOverride.FIELD_CLASSIFICATION_STABLE,
    "range_to_obstacle": AutoDebugOverride.FIELD_RANGE_TO_OBSTACLE,
    "at_crossing_position": AutoDebugOverride.FIELD_AT_CROSSING_POSITION,
    "crossing_step_done": AutoDebugOverride.FIELD_CROSSING_STEP_DONE,
    "crossing_complete": AutoDebugOverride.FIELD_CROSSING_COMPLETE,
    "post_check_passed": AutoDebugOverride.FIELD_POST_CHECK_PASSED,
    "post_check_failed": AutoDebugOverride.FIELD_POST_CHECK_FAILED,
}


def resolve_scenario_path(path_value: str) -> str:
    if os.path.isabs(path_value):
        return path_value
    return os.path.abspath(path_value)


def compute_field_mask(event: dict) -> int:
    if "field_mask" in event:
        return int(event["field_mask"])

    field_mask = 0
    for field_name in event.get("fields", []):
        if field_name not in FIELD_MAP:
            raise KeyError(f"unknown field name in scenario: {field_name}")
        field_mask |= FIELD_MAP[field_name]
    return field_mask


def build_message(event: dict) -> AutoDebugOverride:
    msg = AutoDebugOverride()
    msg.enabled = event.get("enabled", True)
    msg.field_mask = compute_field_mask(event)
    msg.auto_start_requested = event.get("auto_start_requested", False)
    msg.manual_reset_requested = event.get("manual_reset_requested", False)
    msg.emergency_stop = event.get("emergency_stop", False)
    msg.lower_alive = event.get("lower_alive", False)
    msg.imu_ready = event.get("imu_ready", False)
    msg.posture_ready = event.get("posture_ready", False)
    msg.grip_confirmed = event.get("grip_confirmed", False)
    msg.joint_fault = event.get("joint_fault", False)
    msg.grip_fault = event.get("grip_fault", False)
    msg.obstacle_detected = event.get("obstacle_detected", False)
    msg.obstacle_type = event.get("obstacle_type", AutoDebugOverride.OBSTACLE_UNKNOWN)
    msg.classification_stable = event.get("classification_stable", False)
    msg.range_to_obstacle = event.get("range_to_obstacle", 0.0)
    msg.at_crossing_position = event.get("at_crossing_position", False)
    msg.crossing_step_done = event.get("crossing_step_done", False)
    msg.crossing_complete = event.get("crossing_complete", False)
    msg.post_check_passed = event.get("post_check_passed", False)
    msg.post_check_failed = event.get("post_check_failed", False)
    return msg


def main() -> None:
    rospy.init_node("b29_smc_replay")
    topic_name = rospy.get_param("~topic", "/b29_controller/b29_smc_auto_controller/debug_override")
    scenario_param = rospy.get_param("~scenario")
    scenario_path = resolve_scenario_path(scenario_param)

    with open(scenario_path, "r", encoding="utf-8") as handle:
        scenario = yaml.safe_load(handle)

    events = scenario.get("events", [])
    publisher = rospy.Publisher(topic_name, AutoDebugOverride, queue_size=1)
    rate = rospy.Rate(50)
    start_time = rospy.Time.now()
    index = 0

    while not rospy.is_shutdown() and index < len(events):
      elapsed = (rospy.Time.now() - start_time).to_sec()
      event = events[index]
      if elapsed >= float(event["time"]):
          msg = build_message(event)
          msg.header.stamp = rospy.Time.now()
          publisher.publish(msg)
          index += 1
      rate.sleep()


if __name__ == "__main__":
    main()
