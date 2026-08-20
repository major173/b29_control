// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>

#include <hardware_interface/joint_command_interface.h>

#include <b29_smc_auto_controller/auto_types.h>

namespace b29_smc_auto_controller
{
class CommandDispatcher
{
public:
  enum class OutputMode
  {
    kNormal,
    kSafeHold,
  };

  static constexpr std::size_t kPositionJointCount = 6;
  static constexpr std::size_t kWheelJointCount = 2;

  using PositionJointHandles = std::array<hardware_interface::JointHandle, kPositionJointCount>;
  using WheelJointHandles = std::array<hardware_interface::JointHandle, kWheelJointCount>;

  CommandDispatcher() = default;

  void configure(const PositionJointHandles& position_joint_handles, const WheelJointHandles& wheel_joint_handles);
  bool isConfigured() const;
  void setOutputMode(OutputMode mode);
  OutputMode outputMode() const;
  bool dispatch(const AutoControlCommand& command);

private:
  PositionJointHandles position_joint_handles_{};
  WheelJointHandles wheel_joint_handles_{};
  std::array<double, kPositionJointCount> held_joint_targets_{};
  OutputMode output_mode_{OutputMode::kNormal};
  bool hold_latched_{false};
  bool configured_{false};
};
}  // namespace b29_smc_auto_controller
