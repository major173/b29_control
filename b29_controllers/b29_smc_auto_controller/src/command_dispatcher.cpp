// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/command_dispatcher.h>

namespace b29_smc_auto_controller
{
void CommandDispatcher::configure(const PositionJointHandles& position_joint_handles,
                                  const WheelJointHandles& wheel_joint_handles)
{
  position_joint_handles_ = position_joint_handles;
  wheel_joint_handles_ = wheel_joint_handles;
  hold_latched_ = false;
  configured_ = true;
}

bool CommandDispatcher::isConfigured() const
{
  return configured_;
}

void CommandDispatcher::setOutputMode(OutputMode mode)
{
  if (output_mode_ != mode)
  {
    hold_latched_ = false;
  }
  output_mode_ = mode;
}

CommandDispatcher::OutputMode CommandDispatcher::outputMode() const
{
  return output_mode_;
}

void CommandDispatcher::dispatch(const AutoControlCommand& command)
{
  if (!configured_)
  {
    return;
  }

  const bool hold_active = (output_mode_ == OutputMode::kSafeHold) || command.freeze_joints;
  if (!hold_active)
  {
    hold_latched_ = false;
  }

  for (std::size_t i = 0; i < wheel_joint_handles_.size(); ++i)
  {
    double target_speed = 0.0;
    if (output_mode_ == OutputMode::kNormal && !command.stop_all)
    {
      target_speed = (i == 0 ? command.left_wheel_speed : command.right_wheel_speed);
    }
    wheel_joint_handles_[i].setCommand(target_speed);
  }

  if (hold_active && !hold_latched_)
  {
    for (std::size_t i = 0; i < position_joint_handles_.size(); ++i)
    {
      held_joint_targets_[i] = position_joint_handles_[i].getPosition();
    }
    hold_latched_ = true;
  }

  for (std::size_t i = 0; i < position_joint_handles_.size(); ++i)
  {
    double target_position = command.joint_targets[i];
    if (hold_active)
    {
      target_position = held_joint_targets_[i];
    }
    position_joint_handles_[i].setCommand(target_position);
  }
}
}  // namespace b29_smc_auto_controller
