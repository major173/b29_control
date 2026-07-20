// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/auto_types.h>

#include <steering_engine/common/auto_state_interface.h>
#include <steering_engine/common/remote_control_interface.h>

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
  void setAutoState(const steering_engine_hw::AutoStateData& auto_state);
  void setRemoteControl(const steering_engine_hw::RemoteControlData& remote_control);

  AutoInputSnapshot buildSnapshot() const;

private:
  bool isPostureWithinThreshold() const;
  bool isBaseImuChange(const sensor_msgs::Imu& current, 
                       const sensor_msgs::Imu& previous) const;
  static ObstacleType toObstacleType(uint8_t obstacle_type);
  static ros::Time latestStamp(const ros::Time& lhs, const ros::Time& rhs);
  void applyDebugOverride(AutoInputSnapshot& snapshot) const;

  AutoInputMuxConfig config_{};
  sensor_msgs::JointState joint_state_{};
  sensor_msgs::Imu base_imu_{};
  AutoSensorInput sensor_input_{};
  AutoControlRequest control_request_{};
  AutoDebugOverride debug_override_{};
  steering_engine_hw::AutoStateData auto_state_{};
  steering_engine_hw::RemoteControlData remote_control_{};
  bool has_joint_state_{false};
  bool has_base_imu_{false};
  bool has_sensor_input_{false};
  bool has_auto_state_{false};
  bool has_remote_control_{false};
  bool has_control_request_{false};
  bool has_debug_override_{false};
};
}  // namespace b29_smc_auto_controller
