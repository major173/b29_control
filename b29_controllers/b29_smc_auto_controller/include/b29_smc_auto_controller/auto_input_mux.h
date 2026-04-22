// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/auto_types.h>

namespace b29_smc_auto_controller
{
class AutoInputMux
{
public:
  explicit AutoInputMux(const AutoInputMuxConfig& config = AutoInputMuxConfig{});

  void setJointState(const sensor_msgs::JointState& joint_state);
  void setBaseImu(const sensor_msgs::Imu& base_imu);
  void setSensorInput(const AutoSensorInput& sensor_input);
  void setControlRequest(const AutoControlRequest& control_request);
  void setDebugOverride(const AutoDebugOverride& debug_override);
  void IsImuOnline();

  AutoInputSnapshot buildSnapshot() const;

private:
  bool isPostureWithinThreshold() const;
  static ObstacleType toObstacleType(uint8_t obstacle_type);
  static ros::Time latestStamp(const ros::Time& lhs, const ros::Time& rhs);
  void applyDebugOverride(AutoInputSnapshot& snapshot) const;

  AutoInputMuxConfig config_{};
  sensor_msgs::JointState joint_state_{};
  sensor_msgs::Imu base_imu_{};
  AutoSensorInput sensor_input_{};
  AutoControlRequest control_request_{};
  AutoDebugOverride debug_override_{};
  bool has_joint_state_{false};
  bool has_base_imu_{false};
  bool has_sensor_input_{false};
  bool has_control_request_{false};
  bool has_debug_override_{false};
  bool IsImuOnline_{false};
  std::uint32_t imu_valid_streak_{0};
  std::uint32_t imu_invalid_streak_{0};
  sensor_msgs::Imu imu_prev_frame_{};
  bool has_imu_prev_frame_{false};
  std::uint32_t imu_stale_streak_{0};
};
}  // namespace b29_smc_auto_controller
