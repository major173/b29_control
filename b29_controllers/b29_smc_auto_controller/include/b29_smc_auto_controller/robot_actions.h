// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include <b29_smc_auto_controller/auto_types.h>

namespace robot_fsm
{
using ObstacleType = b29_smc_auto_controller::ObstacleType;
using CrossingStrategy = b29_smc_auto_controller::CrossingStrategy;
using DriveMode = b29_smc_auto_controller::DriveMode;

enum class AlertType : uint8_t
{
  CommsLoss = 0,
  CommsRestored = 1,
  SafeStop = 2,
};

class RobotActions
{
public:
  virtual ~RobotActions() = default;

  virtual bool isLowerAlive() const = 0;
  virtual bool isImuReady() const = 0;
  virtual bool isPostureReady() const = 0;
  virtual bool isGripConfirmed() const = 0;
  virtual bool isObstacleDetected() const = 0;
  virtual bool isObstacleWithinCrossObstaclesDistance() const = 0;

  virtual void setCruiseCommand() = 0;
  virtual void setApproachCommand() = 0;
  virtual void setWheelStop() = 0;
  virtual void setSafeStopCommand(const std::string& reason) = 0;
  virtual void stopAllMotors() = 0;
  virtual void freezeAllJoints() = 0;

  virtual void initAutoMode() = 0;
  virtual void disableAutoMode() = 0;
  virtual void startInitSequence() = 0;
  virtual void clearInitFlags() = 0;

  virtual void startReconnectTimer() = 0;
  virtual void stopReconnectTimer() = 0;

  virtual void reportCommsLoss() = 0;
  virtual void alertOperator(AlertType type) = 0;
  virtual void logTransition(std::string_view info) = 0;
  virtual void resetFaultFlags() = 0;
};
}  // namespace robot_fsm
