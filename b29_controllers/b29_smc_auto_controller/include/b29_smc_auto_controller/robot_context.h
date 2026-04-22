// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>

#include <b29_smc_auto_controller/AutoStateTrace.h>
#include <b29_smc_auto_controller/auto_types.h>
#include <b29_smc_auto_controller/command_dispatcher.h>
#include <b29_smc_auto_controller/robot_actions.h>

#include <RobotFSM_sm.h>

namespace b29_smc_auto_controller
{
void applyCommandToTrace(const AutoControlCommand& command, AutoStateTrace& trace);
}

class RobotContext : public robot_fsm::RobotActions
{
public:
  RobotContext();
  ~RobotContext() override = default;

  RobotContext(const RobotContext&) = delete;
  RobotContext& operator=(const RobotContext&) = delete;

  void start();
  void tick50Hz();
  void setInputSnapshot(const b29_smc_auto_controller::AutoInputSnapshot& input);
  void requestAutoStart();
  void setTraceOutputMode(b29_smc_auto_controller::CommandDispatcher::OutputMode mode);
  const b29_smc_auto_controller::AutoControlCommand& currentCommand() const;
  std::string currentStateName() const;
  b29_smc_auto_controller::AutoStateTrace buildTraceMessage(const ros::Time& stamp) const;

  bool isLowerAlive() const override;
  bool isImuReady() const override;
  bool isPostureReady() const override;
  bool isGripConfirmed() const override;
  bool isObstacleDetected() const override;

  void setCruiseCommand() override;
  void setApproachCommand() override;
  void setSafeStopCommand(const std::string& reason) override;
  void stopAllMotors() override;
  void freezeAllJoints() override;

  void initAutoMode() override;
  void disableAutoMode() override;
  void startInitSequence() override;
  void clearInitFlags() override;

  void startReconnectTimer() override;
  void stopReconnectTimer() override;

  void reportCommsLoss() override;
  void alertOperator(robot_fsm::AlertType type) override;
  void logTransition(std::string_view info) override;
  void resetFaultFlags() override;

  bool canStartAuto() const;
  bool isReadyToTraverse() const;
  bool isObstacleNotDetected() const;

  void reportEmergencyStopIdle();
  void reportEmergencyStopInit();
  void reportEmergencyStopTraversal();
  void reportEmergencyStopCommsLoss();
  void reportReconnectTimeout();
  void reportErrorAutoInitFailed();
  void reportAutoRunPause();

  void alertCommsLoss();
  void alertCommsRestored();
  void alertSafeStop();

  void logIdleToAutoInit();
  void logAutoInitToTraversing();
  void logCommsRestored();
  void logSafeStopEntry();
  void logSafeStopToIdle();

private:
  double getCruiseSpeed() const;
  double getApproachSpeed() const;
  void setTargetSpeed(double speed_mps);
  void setDriveMode(robot_fsm::DriveMode mode);
  void reportError(std::string_view reason);
  void reportEmergencyStop(std::string_view reason);
  void setCommandReason(std::string_view reason);
  static b29_smc_auto_controller::DriveMode toAutoDriveMode(robot_fsm::DriveMode mode);

  b29_smc_auto_controller::AutoInputSnapshot input_{};
  b29_smc_auto_controller::AutoControlCommand command_{};
  RobotFSMContext fsm_;
  bool started_{false};
  bool auto_start_requested_{false};
  bool last_input_auto_start_requested_{false};
  bool reconnect_timer_active_{false};
  std::uint32_t reconnect_timer_ticks_{0};
  bool auto_init_timer_active_{false};
  std::uint32_t auto_init_timer_ticks_{0};
  b29_smc_auto_controller::CommandDispatcher::OutputMode trace_output_mode_{
      b29_smc_auto_controller::CommandDispatcher::OutputMode::kNormal};
  std::string last_error_;
  std::string last_alert_;
  std::string last_transition_;
};

namespace b29_smc_auto_controller
{
using RobotContext = ::RobotContext;
}  // namespace b29_smc_auto_controller
