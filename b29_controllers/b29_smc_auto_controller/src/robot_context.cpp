// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/robot_context.h>

#include <utility>

#include <b29_smc_auto_controller/output_mode.h>

namespace
{
constexpr double kCruiseSpeedMps = 0.10;
constexpr double kApproachSpeedMps = 0.03;
constexpr std::uint32_t kReconnectTimeoutTicks = 250;
constexpr std::uint32_t kAutoInitTimeoutTicks = 100;
}

namespace b29_smc_auto_controller
{
void applyCommandToTrace(const AutoControlCommand& command, AutoStateTrace& trace)
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
}  // namespace b29_smc_auto_controller

RobotContext::RobotContext() : fsm_(*this)
{
}

void RobotContext::start()
{
  if (started_)
  {
    return;
  }

  started_ = true;
  fsm_.enterStartState();
}

void RobotContext::tick50Hz()
{
  if (!started_)
  {
    start();
  }

  const int current_state_id = fsm_.getState().getId();

  if (input_.emergency_stop || hasSafetyFault())
  {
    auto_start_requested_ = false;
    fsm_.evEmergencyStop();
    return;
  }

  if (current_state_id == RobotFSM::CommsLoss.getId())
  {
    auto_start_requested_ = false;

    if (input_.lower_alive)
    {
      fsm_.evCommsRestored();
      return;
    }

    if (reconnect_timer_active_)
    {
      ++reconnect_timer_ticks_;
      if (reconnect_timer_ticks_ >= kReconnectTimeoutTicks)
      {
        fsm_.evReconnectTimeout();
        return;
      }
    }

    fsm_.evTick();
    return;
  }

  if (current_state_id == RobotFSM::AutoInit.getId())
  {
    if (isReadyToTraverse())
    {
      fsm_.evTick();
      return;
    }

    if (auto_init_timer_active_)
    {
      ++auto_init_timer_ticks_;
      if (auto_init_timer_ticks_ >= kAutoInitTimeoutTicks)
      {
        fsm_.evInitFailed();
        return;
      }
    }

    fsm_.evTick();
    return;
  }

  if (!input_.lower_alive)
  {
    auto_start_requested_ = false;
    fsm_.evCommsLost();
    return;
  }

  if (current_state_id == RobotFSM::SafeStop.getId())
  {
    auto_start_requested_ = false;

    if (manual_reset_requested_)
    {
      fsm_.evManualReset();
      manual_reset_requested_ = false;
      return;
    }

    fsm_.evTick();
    return;
  }

  if (current_state_id == RobotFSM::Idle.getId() && auto_start_requested_)
  {
    fsm_.evAutoStart();
    auto_start_requested_ = (fsm_.getState().getId() == RobotFSM::Idle.getId());
    return;
  }

  if (current_state_id == RobotFSM::Traversing.getId() && input_.auto_run_pause)
  {
    auto_start_requested_ = false;
    fsm_.evAutoRunPause();
    return;
  }

  fsm_.evTick();
}

void RobotContext::setInputSnapshot(const b29_smc_auto_controller::AutoInputSnapshot& input)
{
  const bool allow_auto_start_latch =
      !started_ || (fsm_.getState().getId() == RobotFSM::Idle.getId());

  if (allow_auto_start_latch && input.auto_start_requested && !last_input_auto_start_requested_)
  {
    auto_start_requested_ = true;
  }

  const bool allow_manual_reset_latch =
      started_ && (fsm_.getState().getId() == RobotFSM::SafeStop.getId());
  if (allow_manual_reset_latch && input.manual_reset_requested &&
      !last_input_manual_reset_requested_)
  {
    manual_reset_requested_ = true;
  }

  last_input_auto_start_requested_ = input.auto_start_requested;
  last_input_manual_reset_requested_ = input.manual_reset_requested;
  input_ = input;
}

void RobotContext::requestAutoStart()
{
  auto_start_requested_ = true;
}

void RobotContext::setTraceOutputMode(b29_smc_auto_controller::CommandDispatcher::OutputMode mode)
{
  trace_output_mode_ = mode;
}

const b29_smc_auto_controller::AutoControlCommand& RobotContext::currentCommand() const
{
  return command_;
}

std::string RobotContext::currentStateName() const
{
  if (!started_)
  {
    return "Unstarted";
  }

  const std::string state_name = const_cast<RobotFSMContext&>(fsm_).getState().getName();
  const std::size_t pos = state_name.rfind("::");
  if (pos == std::string::npos)
  {
    return state_name;
  }

  return state_name.substr(pos + 2);
}

b29_smc_auto_controller::AutoStateTrace RobotContext::buildTraceMessage(const ros::Time& stamp) const
{
  b29_smc_auto_controller::AutoStateTrace trace;
  trace.header.stamp = stamp;
  trace.current_state = currentStateName();
  trace.output_mode = b29_smc_auto_controller::toString(trace_output_mode_);
  b29_smc_auto_controller::applyCommandToTrace(command_, trace);

  if (!last_transition_.empty())
  {
    trace.transition_reason = last_transition_;
    const std::size_t arrow_pos = last_transition_.find("->");
    if (arrow_pos != std::string::npos)
    {
      trace.previous_state = last_transition_.substr(0, arrow_pos);
    }
  }

  if (!last_error_.empty())
  {
    trace.last_event = last_error_;
  }
  else if (!last_alert_.empty())
  {
    trace.last_event = last_alert_;
  }

  return trace;
}

bool RobotContext::isLowerAlive() const
{
  return input_.lower_alive;
}

bool RobotContext::isImuReady() const
{
  return input_.imu_ready;
}

bool RobotContext::isPostureReady() const
{
  return input_.posture_ready;
}

bool RobotContext::isGripConfirmed() const
{
  return input_.grip_confirmed;
}

bool RobotContext::isObstacleDetected() const
{
  return input_.obstacle_detected;
}

void RobotContext::setCruiseCommand()
{
  double target_speed = 0.0;
  switch (input_.cruise_drive_request)
  {
    case 1u:
      target_speed = getCruiseSpeed();
      break;
    case 2u:
      target_speed = -getCruiseSpeed();
      break;
    case 0u:
    default:
      break;
  }

  setDriveMode(target_speed == 0.0 ? robot_fsm::DriveMode::Stop
                                   : robot_fsm::DriveMode::Forward);
  setTargetSpeed(target_speed);
  command_.freeze_joints = false;
  setCommandReason("cruise_command");
}

void RobotContext::setApproachCommand()
{
  setDriveMode(robot_fsm::DriveMode::Forward);
  setTargetSpeed(getApproachSpeed());
  command_.freeze_joints = false;
  setCommandReason("approach_command");
}

void RobotContext::setSafeStopCommand(const std::string& reason)
{
  stopAllMotors();
  command_.freeze_joints = true;
  setCommandReason(reason);
}

void RobotContext::initAutoMode()
{
  command_.stop_all = false;
  command_.freeze_joints = false;
  setCommandReason("auto_mode_initialized");
}

void RobotContext::disableAutoMode()
{
  command_.freeze_joints = true;
}

void RobotContext::startInitSequence()
{
  command_.stop_all = false;
  command_.freeze_joints = false;
  auto_init_timer_active_ = true;
  auto_init_timer_ticks_ = 0;
  setCommandReason("auto_init_started");
}

void RobotContext::clearInitFlags()
{
  auto_init_timer_active_ = false;
  auto_init_timer_ticks_ = 0;
  setCommandReason("auto_init_flags_cleared");
}

void RobotContext::startReconnectTimer()
{
  reconnect_timer_active_ = true;
  reconnect_timer_ticks_ = 0;
  setCommandReason("reconnect_timer_started");
}

void RobotContext::stopReconnectTimer()
{
  reconnect_timer_active_ = false;
  reconnect_timer_ticks_ = 0;
  setCommandReason("reconnect_timer_stopped");
}

void RobotContext::reportError(std::string_view reason)
{
  last_error_ = std::string(reason);
  setCommandReason(reason);
}

void RobotContext::reportCommsLoss()
{
  setSafeStopCommand("comms_lost");
  reportError("comms_lost");
}

void RobotContext::reportEmergencyStop(std::string_view reason)
{
  last_error_ = std::string(reason);
  setSafeStopCommand(std::string(reason));
  setCommandReason(reason);
}

void RobotContext::alertOperator(robot_fsm::AlertType type)
{
  switch (type)
  {
    case robot_fsm::AlertType::CommsLoss:
      last_alert_ = "comms_loss";
      break;
    case robot_fsm::AlertType::CommsRestored:
      last_alert_ = "comms_restored";
      break;
    case robot_fsm::AlertType::SafeStop:
      last_alert_ = "safe_stop";
      break;
  }
}

void RobotContext::logTransition(std::string_view info)
{
  last_transition_ = std::string(info);
}

void RobotContext::resetFaultFlags()
{
  last_error_.clear();
}

bool RobotContext::canStartAuto() const
{
  return isLowerAlive() && isImuReady() && isPostureReady() && isGripConfirmed();
}

bool RobotContext::isReadyToTraverse() const
{
  return canStartAuto();
}

bool RobotContext::isObstacleNotDetected() const
{
  return !isObstacleDetected();
}

bool RobotContext::hasSafetyFault() const
{
  return input_.joint_fault || input_.grip_fault || !input_.imu_ready;;
}

std::string RobotContext::safetyStopReason(std::string_view fallback) const
{
  if (input_.joint_fault && input_.grip_fault)
  {
    return "joint_and_grip_fault";
  }

  if (input_.joint_fault)
  {
    return "joint_fault";
  }

  if (input_.grip_fault)
  {
    return "grip_fault";
  }

  if (!input_.imu_ready)
  {
    return "imu_not_ready";
  }

  return std::string(fallback);
}

void RobotContext::reportEmergencyStopIdle()
{
  reportEmergencyStop(safetyStopReason("emergency_stop_idle"));
}

void RobotContext::reportEmergencyStopInit()
{
  reportEmergencyStop(safetyStopReason("emergency_stop_init"));
}

void RobotContext::reportEmergencyStopTraversal()
{
  reportEmergencyStop(safetyStopReason("emergency_stop_traversal"));
}

void RobotContext::reportEmergencyStopCommsLoss()
{
  reportEmergencyStop(safetyStopReason("emergency_stop_comms_loss"));
}

void RobotContext::reportReconnectTimeout()
{
  setSafeStopCommand("reconnect_timeout");
  reportError("reconnect_timeout");
  logTransition("CommsLoss->SafeStop");
}

void RobotContext::reportErrorAutoInitFailed()
{
  setSafeStopCommand("auto_init_failed");
  reportError("auto_init_failed");
  logTransition("AutoInit->SafeStop");
}

void RobotContext::reportAutoRunPause()
{
  last_error_.clear();
  last_alert_ = "auto_run_pause";
  setSafeStopCommand("auto_run_pause");
  logTransition("Traversing->Idle");
}

void RobotContext::alertCommsLoss()
{
  alertOperator(robot_fsm::AlertType::CommsLoss);
}

void RobotContext::alertCommsRestored()
{
  alertOperator(robot_fsm::AlertType::CommsRestored);
}

void RobotContext::alertSafeStop()
{
  alertOperator(robot_fsm::AlertType::SafeStop);
}

void RobotContext::logIdleToAutoInit()
{
  logTransition("Idle->AutoInit");
}

void RobotContext::logAutoInitToTraversing()
{
  logTransition("AutoInit->Traversing");
}

void RobotContext::logCommsRestored()
{
  last_error_.clear();
  logTransition("CommsLoss->Idle");
}

void RobotContext::logSafeStopEntry()
{
  if (last_error_ != "reconnect_timeout" && last_error_ != "auto_init_failed")
  {
    logTransition("EnterSafeStop");
  }
}

void RobotContext::logSafeStopToIdle()
{
  logTransition("SafeStop->Idle");
}

double RobotContext::getCruiseSpeed() const
{
  return kCruiseSpeedMps;
}

double RobotContext::getApproachSpeed() const
{
  return kApproachSpeedMps;
}

void RobotContext::stopAllMotors()
{
  command_.drive_mode = b29_smc_auto_controller::DriveMode::Stop;
  command_.left_wheel_speed = 0.0;
  command_.right_wheel_speed = 0.0;
  command_.stop_all = true;
}

void RobotContext::freezeAllJoints()
{
  command_.freeze_joints = true;
}

void RobotContext::setTargetSpeed(double speed_mps)
{
  command_.left_wheel_speed = speed_mps;
  command_.right_wheel_speed = speed_mps;
  command_.stop_all = false;
}

void RobotContext::setDriveMode(robot_fsm::DriveMode mode)
{
  command_.drive_mode = toAutoDriveMode(mode);
}

void RobotContext::setCommandReason(std::string_view reason)
{
  command_.command_reason = std::string(reason);
}

b29_smc_auto_controller::DriveMode RobotContext::toAutoDriveMode(robot_fsm::DriveMode mode)
{
  switch (mode)
  {
    case robot_fsm::DriveMode::Forward:
      return b29_smc_auto_controller::DriveMode::Forward;
    case robot_fsm::DriveMode::Stop:
    default:
      return b29_smc_auto_controller::DriveMode::Stop;
  }
}
