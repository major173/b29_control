// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/obstacle_crossing_runtime.h>

namespace b29_smc_auto_controller
{
void ObstacleCrossingRuntime::configure(const Config& config)
{
  config_ = config;
}

ObstacleCrossingRuntime::Actions ObstacleCrossingRuntime::update(const ros::Time& time, const Inputs& inputs,
                                                                   const Events& events)
{
  Actions actions;
  obstacle_trigger_rising_edge_ = false;
  obstacle_trigger_falling_edge_ = false;
  if (!obstacle_trigger_edges_initialized_)
  {
    obstacle_trigger_edges_initialized_ = true;
    previous_obstacle_crossing_trigger_ = inputs.obstacle_crossing_trigger;
    last_obstacle_trigger_rising_edge_sequence_ =
        inputs.obstacle_trigger_rising_edge_sequence;
    last_obstacle_trigger_falling_edge_sequence_ =
        inputs.obstacle_trigger_falling_edge_sequence;
  }
  else
  {
    obstacle_trigger_rising_edge_ =
        (!previous_obstacle_crossing_trigger_ && inputs.obstacle_crossing_trigger) ||
        (inputs.obstacle_trigger_edge_sequences_valid &&
         inputs.obstacle_trigger_rising_edge_sequence !=
             last_obstacle_trigger_rising_edge_sequence_);
    obstacle_trigger_falling_edge_ =
        (previous_obstacle_crossing_trigger_ && !inputs.obstacle_crossing_trigger) ||
        (inputs.obstacle_trigger_edge_sequences_valid &&
         inputs.obstacle_trigger_falling_edge_sequence !=
             last_obstacle_trigger_falling_edge_sequence_);
    previous_obstacle_crossing_trigger_ = inputs.obstacle_crossing_trigger;
    last_obstacle_trigger_rising_edge_sequence_ =
        inputs.obstacle_trigger_rising_edge_sequence;
    last_obstacle_trigger_falling_edge_sequence_ =
        inputs.obstacle_trigger_falling_edge_sequence;
  }

  if (inputs.safety_blocked)
  {
    return reset(time, inputs.grip_confirmed, "crossing_reset");
  }

  if (stage_ == ObstacleCrossingStage::Idle)
  {
    const bool lower_triggered_start =
        inputs.traversing && obstacle_trigger_rising_edge_;
    if (events.start_disconnect_requested || lower_triggered_start)
    {
      retry_counts_.reset();
      failed_operation_ = FailedOperation::None;
      last_failure_reason_.clear();
      first_crossing_side_ =
          firstSideFromCruiseDriveRequest(inputs.last_nonzero_cruise_drive_request);
      if (first_crossing_side_ == CrossingSide::None)
      {
        // A Stop frame is expected immediately before disconnect.  If no
        // non-zero direction has ever been observed, remain idle and wait for
        // a real direction; do not infer a side from encoders or start any
        // crossing action.
        refreshGravityCompensationLatch();
        return actions;
      }
      crossing_side_ = first_crossing_side_;
      grip_confirmation_low_seen_ = false;
      enterOpenGripperBeforeGravityCompensation(
          crossing_side_, time,
          lower_triggered_start ? "obstacle_trigger_rising_edge"
                                : "start_disconnect_requested");
    }
    refreshGravityCompensationLatch();
    return actions;
  }

  switch (stage_)
  {
    case ObstacleCrossingStage::CloseBothGrippers:
      if (consumeGripConfirmation(inputs.grip_confirmed))
      {
        retry_counts_.close_grippers = 0;
        enterOpenGripperBeforeGravityCompensation(
            first_crossing_side_, time, "grippers_closed_before_opening_crossing_side");
      }
      else if (gripWaitElapsed(time))
      {
        last_failure_reason_ = "close_both_grippers_confirmation_timeout";
        enterManualIntervention(FailedOperation::CloseBothGrippers,
                                first_crossing_side_, time, actions,
                                "grip_confirmation_timeout_no_retry");
      }
      break;
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
      if (gripWaitElapsed(time))
      {
        transition(ObstacleCrossingStage::EnableGravityCompensation, time,
                   "crossing_side_gripper_open_wait_completed");
      }
      break;
    case ObstacleCrossingStage::EnableGravityCompensation:
      transition(ObstacleCrossingStage::Disconnecting, time, "start_disconnect_side");
      actions.start_disconnect_process = true;
      break;
    case ObstacleCrossingStage::Disconnecting:
      if (events.disconnect == DisconnectCableProcess::Outcome::Completed)
      {
        if (int* retry_count = retryCounter(FailedOperation::DisconnectCable, crossing_side_))
        {
          *retry_count = 0;
        }
        transition(ObstacleCrossingStage::DisconnectDoneWaitFlip, time,
                   "disconnect_completed_waiting_flip_start");
        actions.reset_disconnect_process = true;
        actions.disconnect_succeeded_wait_flip_entered = true;
      }
      else if (events.disconnect == DisconnectCableProcess::Outcome::Failed)
      {
        last_failure_reason_ = std::string("disconnect_") + crossingSideReasonName(crossing_side_) +
                               "_validation_failed_hold";
        enterManualIntervention(FailedOperation::DisconnectCable, crossing_side_, time, actions,
                                "disconnect_validation_failed_hold");
        actions.disconnect_failed_hold_entered = true;
      }
      break;
    case ObstacleCrossingStage::DisconnectDoneWaitFlip:
      if (events.start_flip_requested)
      {
        transition(ObstacleCrossingStage::PlannerControl, time, "start_flip_requested");
        actions.planner_action = PlannerAction::Start;
      }
      break;
    case ObstacleCrossingStage::PlannerControl:
      if (events.planner == PlannerOutcome::Completed)
      {
        enterRemoteControl(time, "planner_completion_confirmed", actions);
      }
      else if (events.planner == PlannerOutcome::ManualReleased)
      {
        enterRemoteControl(time, "planner_manual_release_received", actions);
      }
      else if (events.planner == PlannerOutcome::TotalWatchdogExpired)
      {
        last_failure_reason_ = std::string("planner_") + crossingSideReasonName(crossing_side_) +
                               "_total_watchdog_timeout";
        enterManualIntervention(FailedOperation::PlannerControl, crossing_side_, time, actions,
                                "planner_total_watchdog_timeout");
        // The Planner coordinator already stopped the session with the watchdog exit reason.
        actions.planner_action = PlannerAction::None;
      }
      break;
    case ObstacleCrossingStage::RemoteControl:
      if (events.remote_control_completion_rising_edge)
      {
        transition(ObstacleCrossingStage::Regrip, time, "remote_control_completion_rising_edge");
        actions.remote_control_completed_regrip_entered = true;
        gripper_wait_start_time_ = time;
        grip_confirmation_low_seen_ = !inputs.grip_confirmed;
      }
      break;
    case ObstacleCrossingStage::Regrip:
      if (consumeGripConfirmation(inputs.grip_confirmed))
      {
        if (int* retry_count = retryCounter(FailedOperation::Regrip, crossing_side_))
        {
          *retry_count = 0;
        }
        if (crossing_side_ == first_crossing_side_)
        {
          gravity_compensation_latched_mode_ = GravityCompensationMode::Off;
          enterOpenGripperBeforeGravityCompensation(
              oppositeSide(crossing_side_), time, "grippers_closed_before_opening_crossing_side");
        }
        else
        {
          enterCompleteWaitObstacleClear(time, "both_sides_completed", actions);
        }
      }
      else if (regripWaitElapsed(time))
      {
        int* retry_count = retryCounter(FailedOperation::Regrip, crossing_side_);
        if (retry_count == nullptr)
        {
          enterManualIntervention(FailedOperation::Regrip, crossing_side_, time, actions,
                                  "regrip_invalid_side");
        }
        else
        {
          ++(*retry_count);
          if (*retry_count >= config_.retry_limit)
          {
            last_failure_reason_ = std::string("regrip_") + crossingSideReasonName(crossing_side_) +
                                   "_retry_limit_reached";
            enterManualIntervention(FailedOperation::Regrip, crossing_side_, time, actions,
                                    "retry_limit_reached");
          }
          else
          {
            transition(ObstacleCrossingStage::ReopenBeforeRemoteControl, time,
                       "regrip_retry_reopen_before_remote_control");
            gripper_wait_start_time_ = time;
          }
        }
      }
      break;
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      if (gripWaitElapsed(time))
      {
        enterRemoteControl(time, "regrip_retry_reopen_completed", actions);
      }
      break;
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
      if (obstacle_trigger_falling_edge_)
      {
        actions = reset(time, inputs.grip_confirmed,
                        "obstacle_trigger_falling_edge");
      }
      break;
    case ObstacleCrossingStage::ManualIntervention:
      if (failed_operation_ == FailedOperation::DisconnectCable)
      {
        break;
      }
      if (inputs.grip_confirmed)
      {
        const FailedOperation operation = failed_operation_;
        const CrossingSide side = crossing_side_;
        if (int* retry_count = retryCounter(operation, side))
        {
          *retry_count = 0;
        }
        failed_operation_ = FailedOperation::None;
        actions.reset_disconnect_process = true;
        if (operation == FailedOperation::CloseBothGrippers)
        {
          enterOpenGripperBeforeGravityCompensation(side, time,
                                                     "manual_intervention_close_grippers_completed");
        }
        else if (side == first_crossing_side_)
        {
          gravity_compensation_latched_mode_ = GravityCompensationMode::Off;
          enterOpenGripperBeforeGravityCompensation(
              oppositeSide(side), time, "manual_intervention_current_side_completed");
        }
        else if (side != CrossingSide::None)
        {
          enterCompleteWaitObstacleClear(time, "manual_intervention_current_side_completed", actions);
        }
        else
        {
          actions = reset(time, inputs.grip_confirmed, "crossing_reset");
        }
      }
      break;
    case ObstacleCrossingStage::Idle:
    default:
      break;
  }

  refreshGravityCompensationLatch();
  return actions;
}

ObstacleCrossingRuntime::Actions ObstacleCrossingRuntime::reset(const ros::Time& time, bool grip_confirmed,
                                                                  const std::string& reason)
{
  Actions actions;
  actions.reset_disconnect_process = true;
  actions.planner_action = PlannerAction::RevokeForSafety;
  actions.reset_remote_control_session = true;
  if (grip_confirmed)
  {
    gravity_compensation_latched_mode_ = GravityCompensationMode::Off;
  }
  transition(ObstacleCrossingStage::Idle, time, reason);
  crossing_side_ = CrossingSide::None;
  first_crossing_side_ = CrossingSide::Left;
  failed_operation_ = FailedOperation::None;
  retry_counts_.reset();
  gripper_wait_start_time_ = ros::Time{};
  grip_confirmation_low_seen_ = false;
  last_failure_reason_.clear();
  return actions;
}

void ObstacleCrossingRuntime::reportFailure(const std::string& reason)
{
  last_failure_reason_ = reason;
}

const CrossingSideProfile* ObstacleCrossingRuntime::currentProfile() const
{
  return crossingSideProfile(crossing_side_);
}

bool ObstacleCrossingRuntime::stageUsesLastEffectiveTargets() const
{
  return stage_ != ObstacleCrossingStage::ManualIntervention;
}

std::string ObstacleCrossingRuntime::manualInterventionCommandReason() const
{
  return std::string("manual_intervention_") + failedOperationReasonName(failed_operation_);
}

ObstacleCrossingRuntime::TraceState ObstacleCrossingRuntime::traceState() const
{
  TraceState state;
  state.stage = stage_;
  state.crossing_side = crossing_side_;
  state.first_crossing_side = first_crossing_side_;
  state.stage_enter_time = stage_enter_time_;
  state.transition_reason = transition_reason_;
  state.failed_operation = failed_operation_;
  const int* retry_count = retryCounter(failedOperationForStage(), crossing_side_);
  if (retry_count != nullptr)
  {
    state.retry_count = static_cast<std::uint32_t>(*retry_count);
  }
  state.close_grippers_retry_count = static_cast<std::uint32_t>(retry_counts_.close_grippers);
  state.left_disconnect_retry_count = static_cast<std::uint32_t>(retry_counts_.disconnect[0]);
  state.right_disconnect_retry_count = static_cast<std::uint32_t>(retry_counts_.disconnect[1]);
  state.left_regrip_retry_count = static_cast<std::uint32_t>(retry_counts_.regrip[0]);
  state.right_regrip_retry_count = static_cast<std::uint32_t>(retry_counts_.regrip[1]);
  state.retry_limit = static_cast<std::uint32_t>(config_.retry_limit);
  state.gripper_wait_start_time = gripper_wait_start_time_;
  state.wait_for_grip_respond_time =
      stage_ == ObstacleCrossingStage::Regrip ?
      config_.regrip_confirmation_timeout : config_.wait_for_grip_respond_time;
  state.last_failure_reason = last_failure_reason_;
  state.obstacle_crossing_trigger = previous_obstacle_crossing_trigger_;
  state.obstacle_trigger_rising_edge = obstacle_trigger_rising_edge_;
  state.obstacle_trigger_falling_edge = obstacle_trigger_falling_edge_;
  state.obstacle_trigger_rising_edge_sequence =
      last_obstacle_trigger_rising_edge_sequence_;
  state.obstacle_trigger_falling_edge_sequence =
      last_obstacle_trigger_falling_edge_sequence_;
  return state;
}

void ObstacleCrossingRuntime::RetryCounts::reset()
{
  close_grippers = 0;
  disconnect = {{0, 0}};
  regrip = {{0, 0}};
}

void ObstacleCrossingRuntime::transition(ObstacleCrossingStage stage, const ros::Time& time,
                                         const std::string& reason)
{
  stage_ = stage;
  stage_enter_time_ = time;
  transition_reason_ = reason;
}

void ObstacleCrossingRuntime::enterManualIntervention(FailedOperation operation, CrossingSide side,
                                                       const ros::Time& time, Actions& actions,
                                                       const std::string& reason)
{
  transition(ObstacleCrossingStage::ManualIntervention, time, reason);
  crossing_side_ = side;
  failed_operation_ = operation;
  actions.reset_disconnect_process = true;
  actions.planner_action = PlannerAction::RevokeForManualIntervention;
  actions.reset_remote_control_session = true;
}

void ObstacleCrossingRuntime::enterRemoteControl(const ros::Time& time, const std::string& reason,
                                                  Actions& actions)
{
  transition(ObstacleCrossingStage::RemoteControl, time, reason);
  actions.reset_disconnect_process = true;
  actions.start_remote_control_session = true;
}

void ObstacleCrossingRuntime::enterOpenGripperBeforeGravityCompensation(CrossingSide side,
                                                                          const ros::Time& time,
                                                                          const std::string& reason)
{
  crossing_side_ = side;
  transition(ObstacleCrossingStage::OpenGripperBeforeGravityCompensation, time, reason);
  gripper_wait_start_time_ = time;
}

void ObstacleCrossingRuntime::enterCompleteWaitObstacleClear(const ros::Time& time, const std::string& reason,
                                                              Actions& actions)
{
  gravity_compensation_latched_mode_ = GravityCompensationMode::Off;
  transition(ObstacleCrossingStage::CompleteWaitObstacleClear, time, reason);
  crossing_side_ = CrossingSide::None;
  actions.reset_disconnect_process = true;
  actions.reset_wheel_travel = true;
}

void ObstacleCrossingRuntime::refreshGravityCompensationLatch()
{
  bool requires_compensation = false;
  switch (stage_)
  {
    case ObstacleCrossingStage::EnableGravityCompensation:
    case ObstacleCrossingStage::Disconnecting:
    case ObstacleCrossingStage::DisconnectDoneWaitFlip:
    case ObstacleCrossingStage::PlannerControl:
    case ObstacleCrossingStage::RemoteControl:
    case ObstacleCrossingStage::Regrip:
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      requires_compensation = true;
      break;
    case ObstacleCrossingStage::ManualIntervention:
      requires_compensation = failed_operation_ != FailedOperation::None &&
                              failed_operation_ != FailedOperation::CloseBothGrippers;
      break;
    default:
      break;
  }
  if (!requires_compensation)
  {
    return;
  }
  const CrossingSideProfile* profile = currentProfile();
  if (profile != nullptr)
  {
    gravity_compensation_latched_mode_ = profile->gravity_compensation_mode;
  }
}

bool ObstacleCrossingRuntime::consumeGripConfirmation(bool grip_confirmed)
{
  if (!grip_confirmed)
  {
    grip_confirmation_low_seen_ = true;
    return false;
  }
  if (!grip_confirmation_low_seen_)
  {
    return false;
  }
  grip_confirmation_low_seen_ = false;
  return true;
}

bool ObstacleCrossingRuntime::gripWaitElapsed(const ros::Time& time) const
{
  return (time - gripper_wait_start_time_) >= ros::Duration(config_.wait_for_grip_respond_time);
}

bool ObstacleCrossingRuntime::regripWaitElapsed(const ros::Time& time) const
{
  return (time - gripper_wait_start_time_) >= ros::Duration(config_.regrip_confirmation_timeout);
}

int* ObstacleCrossingRuntime::retryCounter(FailedOperation operation, CrossingSide side)
{
  const auto* runtime = static_cast<const ObstacleCrossingRuntime*>(this);
  return const_cast<int*>(runtime->retryCounter(operation, side));
}

const int* ObstacleCrossingRuntime::retryCounter(FailedOperation operation, CrossingSide side) const
{
  switch (operation)
  {
    case FailedOperation::CloseBothGrippers:
      return &retry_counts_.close_grippers;
    case FailedOperation::DisconnectCable:
      if (side == CrossingSide::Left)
      {
        return &retry_counts_.disconnect[0];
      }
      if (side == CrossingSide::Right)
      {
        return &retry_counts_.disconnect[1];
      }
      return nullptr;
    case FailedOperation::Regrip:
      if (side == CrossingSide::Left)
      {
        return &retry_counts_.regrip[0];
      }
      if (side == CrossingSide::Right)
      {
        return &retry_counts_.regrip[1];
      }
      return nullptr;
    case FailedOperation::PlannerControl:
    case FailedOperation::None:
      return nullptr;
  }
  return nullptr;
}

FailedOperation ObstacleCrossingRuntime::failedOperationForStage() const
{
  switch (stage_)
  {
    case ObstacleCrossingStage::ManualIntervention:
      return failed_operation_;
    case ObstacleCrossingStage::CloseBothGrippers:
      return FailedOperation::CloseBothGrippers;
    case ObstacleCrossingStage::Disconnecting:
      return FailedOperation::DisconnectCable;
    case ObstacleCrossingStage::Regrip:
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      return FailedOperation::Regrip;
    default:
      return FailedOperation::None;
  }
}

CrossingSide ObstacleCrossingRuntime::oppositeSide(CrossingSide side)
{
  if (side == CrossingSide::Left)
  {
    return CrossingSide::Right;
  }
  if (side == CrossingSide::Right)
  {
    return CrossingSide::Left;
  }
  return CrossingSide::None;
}

CrossingSide ObstacleCrossingRuntime::firstSideFromCruiseDriveRequest(
    CruiseDriveRequest request)
{
  // The commissioned profiles are tied to the lower-level travel command:
  // Forward selects the Left/free-side pass; Reverse mirrors that to Right.
  // Stop and Invalid carry no direction and are handled by the caller as a
  // no-op.
  if (request == CruiseDriveRequest::Forward)
  {
    return CrossingSide::Left;
  }
  if (request == CruiseDriveRequest::Reverse)
  {
    return CrossingSide::Right;
  }
  return CrossingSide::None;
}

const char* ObstacleCrossingRuntime::failedOperationReasonName(FailedOperation operation)
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
    default:
      return "none";
  }
}
}  // namespace b29_smc_auto_controller
