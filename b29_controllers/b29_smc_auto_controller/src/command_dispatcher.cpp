// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/command_dispatcher.h>

namespace b29_smc_auto_controller
{
void CommandDispatcher::configure(const PositionJointHandles& position_joint_handles,
                                  const WheelJointHandles& wheel_joint_handles)
{
  position_joint_handles_ = position_joint_handles;
  wheel_joint_handles_ = wheel_joint_handles;
  configured_ = true;
}

bool CommandDispatcher::isConfigured() const
{
  return configured_;
}

void CommandDispatcher::dispatch(const AutoControlCommand& command)
{
  if (!configured_)
  {
    return;
  }

  for (std::size_t i = 0; i < wheel_joint_handles_.size(); ++i)
  {
    const double target_speed = command.stop_all ? 0.0 : (i == 0 ? command.left_wheel_speed : command.right_wheel_speed);
    wheel_joint_handles_[i].setCommand(target_speed);
  }

  for (std::size_t i = 0; i < position_joint_handles_.size(); ++i)
  {
    const double target_position =
        command.freeze_joints ? position_joint_handles_[i].getPosition() : command.joint_targets[i];
    position_joint_handles_[i].setCommand(target_position);
  }
}
}  // namespace b29_smc_auto_controller
