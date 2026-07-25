// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/controller_trace_builder.h>

#include <algorithm>

namespace b29_smc_auto_controller
{
namespace
{
const char* stageName(ObstacleCrossingStage stage)
{
  switch (stage)
  {
    case ObstacleCrossingStage::Idle:
      return "Idle";
    case ObstacleCrossingStage::CloseBothGrippers:
      return "CloseBothGrippers";
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
      return "OpenGripperBeforeGravityCompensation";
    case ObstacleCrossingStage::EnableGravityCompensation:
      return "EnableGravityCompensation";
    case ObstacleCrossingStage::Disconnecting:
      return "Disconnecting";
    case ObstacleCrossingStage::PlannerControl:
      return "PlannerControl";
    case ObstacleCrossingStage::RemoteControl:
      return "RemoteControl";
    case ObstacleCrossingStage::Regrip:
      return "Regrip";
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      return "ReopenBeforeRemoteControl";
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
      return "CompleteWaitObstacleClear";
    case ObstacleCrossingStage::ManualIntervention:
      return "ManualIntervention";
  }
  return "Unknown";
}

const char* sideName(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return "Left";
    case CrossingSide::Right:
      return "Right";
    case CrossingSide::None:
      return "None";
  }
  return "Unknown";
}

const char* sideReasonName(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return "left";
    case CrossingSide::Right:
      return "right";
    case CrossingSide::None:
    default:
      return "none";
  }
}

const char* failedOperationName(FailedOperation operation)
{
  switch (operation)
  {
    case FailedOperation::CloseBothGrippers:
      return "CloseBothGrippers";
    case FailedOperation::DisconnectCable:
      return "DisconnectCable";
    case FailedOperation::PlannerControl:
      return "PlannerControl";
    case FailedOperation::Regrip:
      return "Regrip";
    case FailedOperation::None:
      return "None";
  }
  return "Unknown";
}

const char* failedOperationReasonName(FailedOperation operation)
{
  switch (operation)
  {
    case FailedOperation::CloseBothGrippers:
      return "close_both_grippers";
    case FailedOperation::DisconnectCable:
      return "disconnect_cable";
    case FailedOperation::PlannerControl:
      return "planner_control";
    case FailedOperation::Regrip:
      return "regrip";
    case FailedOperation::None:
      return "none";
  }
  return "unknown";
}

const char* disconnectStepName(DisconnectCableStep step)
{
  switch (step)
  {
    case DisconnectCableStep::Idle:
      return "Idle";
    case DisconnectCableStep::Step1LoosenGripper:
      return "Step1LoosenGripper";
    case DisconnectCableStep::Step2WaitGripperRespond:
      return "Step2WaitGripperRespond";
    case DisconnectCableStep::Step3UpFirstJoint:
      return "Step3UpFirstJoint";
    case DisconnectCableStep::Step4MoveSecondJoint:
      return "Step4MoveSecondJoint";
    case DisconnectCableStep::Step5DownFirstJoint:
      return "Step5DownFirstJoint";
    case DisconnectCableStep::Step6MoveSecondJoint:
      return "Step6MoveSecondJoint";
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
      return "Step7CheckIfCableDisconnected";
    case DisconnectCableStep::Step8ReturnToZero:
      return "Step8ReturnToZero";
    case DisconnectCableStep::Failed:
      return "Failed";
  }
  return "Unknown";
}

const char* driveModeName(DriveMode mode)
{
  switch (mode)
  {
    case DriveMode::Forward:
      return "Forward";
    case DriveMode::Stop:
    default:
      return "Stop";
  }
}

double elapsedSec(const ros::Time& now, const ros::Time& start)
{
  if (start.isZero())
  {
    return 0.0;
  }
  return std::max(0.0, (now - start).toSec());
}

const char* disconnectStepExpectedCondition(DisconnectCableStep step)
{
  switch (step)
  {
    case DisconnectCableStep::Step1LoosenGripper:
      return "send_open_gripper_command";
    case DisconnectCableStep::Step2WaitGripperRespond:
      return "wait_for_gripper_response_timeout";
    case DisconnectCableStep::Step3UpFirstJoint:
    case DisconnectCableStep::Step4MoveSecondJoint:
    case DisconnectCableStep::Step5DownFirstJoint:
    case DisconnectCableStep::Step6MoveSecondJoint:
    case DisconnectCableStep::Step8ReturnToZero:
      return "wait_for_motion_segment_and_interval";
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
      return "abs_joint_feedback_displacement_reaches_threshold";
    case DisconnectCableStep::Failed:
      return "retry_or_manual_intervention";
    case DisconnectCableStep::Idle:
    default:
      return "";
  }
}

void applyEffectiveCommandToTrace(const AutoControlCommand& command, AutoStateTrace& trace)
{
  trace.command_reason = command.command_reason;
  trace.stop_all = command.stop_all;
  trace.freeze_joints = command.freeze_joints;
  trace.left_wheel_speed = command.left_wheel_speed;
  trace.right_wheel_speed = command.right_wheel_speed;

  switch (command.crossing_strategy)
  {
    case CrossingStrategy::LineClamp:
      trace.crossing_strategy = AutoStateTrace::CROSSING_LINE_CLAMP;
      break;
    case CrossingStrategy::Damper:
      trace.crossing_strategy = AutoStateTrace::CROSSING_DAMPER;
      break;
    case CrossingStrategy::None:
    default:
      trace.crossing_strategy = AutoStateTrace::CROSSING_NONE;
      break;
  }
}
}  // namespace

AutoStateTrace ControllerTraceBuilder::build(const ros::Time& stamp, const AutoControlCommand& effective_command,
                                             const RobotContextTraceState& robot_state,
                                             const ControllerTraceState& controller_state)
{
  AutoStateTrace trace;
  trace.header.stamp = stamp;
  trace.current_state = robot_state.current_state;
  trace.output_mode = robot_state.output_mode;
  trace.base_command_reason = robot_state.base_command.command_reason;
  if (!robot_state.last_transition.empty())
  {
    trace.transition_reason = robot_state.last_transition;
    const std::size_t arrow_pos = robot_state.last_transition.find("->");
    if (arrow_pos != std::string::npos)
    {
      trace.previous_state = robot_state.last_transition.substr(0, arrow_pos);
    }
  }
  if (!robot_state.last_error.empty())
  {
    trace.last_event = robot_state.last_error;
  }
  else if (!robot_state.last_alert.empty())
  {
    trace.last_event = robot_state.last_alert;
  }

  applyEffectiveCommandToTrace(effective_command, trace);
  trace.obstacle_crossing_active = controller_state.obstacle_crossing_stage != ObstacleCrossingStage::Idle;
  trace.obstacle_crossing_stage = stageName(controller_state.obstacle_crossing_stage);
  trace.obstacle_crossing_side = sideName(controller_state.crossing_side);
  trace.first_crossing_side = sideName(controller_state.first_crossing_side);
  trace.obstacle_crossing_stage_enter_time = controller_state.obstacle_crossing_stage_enter_time;
  trace.obstacle_crossing_stage_elapsed_sec =
      elapsedSec(stamp, controller_state.obstacle_crossing_stage_enter_time);
  trace.obstacle_crossing_transition_reason = controller_state.obstacle_crossing_transition_reason;
  trace.disconnect_step = disconnectStepName(controller_state.disconnect_step);
  trace.disconnect_step_enter_time = controller_state.disconnect_step_enter_time;
  trace.disconnect_step_elapsed_sec = elapsedSec(stamp, controller_state.disconnect_step_enter_time);
  trace.disconnect_step_transition_reason = controller_state.disconnect_step_transition_reason;
  trace.disconnect_step_expected_condition = disconnectStepExpectedCondition(controller_state.disconnect_step);
  trace.disconnect_check_displacement = controller_state.disconnect_check_displacement;
  trace.to_check_joint_pos = controller_state.to_check_joint_pos;
  trace.disconnect_cable_second_joint_success_threshold =
      controller_state.disconnect_cable_second_joint_success_threshold;
  trace.manual_intervention_active = controller_state.obstacle_crossing_stage == ObstacleCrossingStage::ManualIntervention;
  if (trace.manual_intervention_active)
  {
    trace.manual_intervention_reason =
        std::string("manual_intervention_") + failedOperationReasonName(controller_state.failed_operation);
  }
  trace.failed_action = failedOperationName(controller_state.failed_operation);
  trace.retry_count = controller_state.retry_count;
  trace.close_grippers_retry_count = controller_state.close_grippers_retry_count;
  trace.left_disconnect_retry_count = controller_state.left_disconnect_retry_count;
  trace.right_disconnect_retry_count = controller_state.right_disconnect_retry_count;
  trace.left_regrip_retry_count = controller_state.left_regrip_retry_count;
  trace.right_regrip_retry_count = controller_state.right_regrip_retry_count;
  trace.retry_limit = controller_state.retry_limit;
  trace.waiting_for_grip_confirmed =
      controller_state.obstacle_crossing_stage == ObstacleCrossingStage::CloseBothGrippers ||
      controller_state.obstacle_crossing_stage == ObstacleCrossingStage::Regrip;
  if (trace.waiting_for_grip_confirmed)
  {
    trace.grip_wait_elapsed_sec = elapsedSec(stamp, controller_state.gripper_wait_start_time);
  }
  trace.grip_wait_timed_out = trace.waiting_for_grip_confirmed &&
                              trace.grip_wait_elapsed_sec >= controller_state.wait_for_grip_respond_time;
  trace.last_failure_reason = controller_state.last_failure_reason;

  trace.planner_control_active = controller_state.obstacle_crossing_stage == ObstacleCrossingStage::PlannerControl;
  trace.planner_manual_release_enabled = controller_state.planner_manual_release_enabled;
  trace.planner_release_received = controller_state.planner_release_received;
  if (trace.planner_control_active)
  {
    trace.planner_control_wait_elapsed_sec =
        elapsedSec(stamp, controller_state.obstacle_crossing_stage_enter_time);
  }
  trace.planner_point_available = controller_state.planner_point_available;
  trace.planner_point_fresh = controller_state.planner_point_fresh;
  trace.planner_override_applied = controller_state.planner_override_applied;
  trace.remote_control_active = controller_state.obstacle_crossing_stage == ObstacleCrossingStage::RemoteControl;
  for (std::size_t index = 0; index < controller_state.remote_control_joint_targets.size(); ++index)
  {
    trace.remote_control_raw_increments[index] = controller_state.remote_control_raw_increments[index];
    trace.remote_control_applied_increments[index] = controller_state.remote_control_applied_increments[index];
    trace.remote_control_joint_targets[index] = controller_state.remote_control_joint_targets[index];
  }
  trace.remote_control_increments_valid = controller_state.remote_control_increments_valid;
  trace.remote_control_sample_sequence = controller_state.remote_control_sample_sequence;
  trace.remote_control_complete = controller_state.remote_control_complete;
  trace.remote_control_completion_rising_edge = controller_state.remote_control_completion_rising_edge;

  trace.drive_mode = driveModeName(effective_command.drive_mode);
  trace.gravity_compensation_mode = static_cast<uint8_t>(effective_command.gravity_compensation_mode);
  for (std::size_t index = 0; index < effective_command.joint_targets.size(); ++index)
  {
    trace.joint_targets[index] = effective_command.joint_targets[index];
  }
  trace.left_gripper_target = effective_command.joint_targets[static_cast<std::size_t>(JointIndex::LeftGripper)];
  trace.right_gripper_target = effective_command.joint_targets[static_cast<std::size_t>(JointIndex::RightGripper)];
  trace.command_dispatch_attempted = controller_state.command_dispatch_attempted;
  trace.command_dispatch_succeeded = controller_state.command_dispatch_succeeded;
  if (controller_state.crossing_side == CrossingSide::Left)
  {
    trace.current_side_gripper = sideReasonName(controller_state.crossing_side);
    trace.current_side_gripper_target =
        effective_command.joint_targets[static_cast<std::size_t>(JointIndex::LeftGripper)];
  }
  else if (controller_state.crossing_side == CrossingSide::Right)
  {
    trace.current_side_gripper = sideReasonName(controller_state.crossing_side);
    trace.current_side_gripper_target =
        effective_command.joint_targets[static_cast<std::size_t>(JointIndex::RightGripper)];
  }

  trace.debug_validation_enabled = controller_state.debug_validation_enabled;
  trace.simulation_only = controller_state.simulation_only;
  trace.debug_override_active = controller_state.debug_override_active;
  trace.debug_obstacle_detected = controller_state.debug_obstacle_detected;
  trace.debug_obstacle_distance = controller_state.debug_obstacle_distance;
  trace.software_emergency_stop_latched = controller_state.software_emergency_stop_latched;
  trace.manual_reset_requested = controller_state.manual_reset_requested;
  trace.lower_alive = controller_state.lower_alive;
  trace.imu_ready = controller_state.imu_ready;
  trace.posture_ready = controller_state.posture_ready;
  trace.grip_confirmed = controller_state.grip_confirmed;
  trace.joint_fault = controller_state.joint_fault;
  trace.grip_fault = controller_state.grip_fault;
  trace.obstacle_detected = controller_state.obstacle_detected;
  trace.range_to_obstacle = controller_state.range_to_obstacle;
  return trace;
}
}  // namespace b29_smc_auto_controller
