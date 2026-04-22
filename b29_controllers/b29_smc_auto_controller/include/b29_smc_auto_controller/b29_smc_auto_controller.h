// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>

#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/AutoStateTrace.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <b29_smc_auto_controller/auto_input_mux.h>
#include <b29_smc_auto_controller/command_dispatcher.h>
#include <b29_smc_auto_controller/robot_context.h>

namespace b29_smc_auto_controller
{
class B29SmcAutoController
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          hardware_interface::PositionJointInterface,
                                                          hardware_interface::VelocityJointInterface,
                                                          hardware_interface::ImuSensorInterface>
{
public:
  B29SmcAutoController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  static constexpr std::size_t kPositionJointCount = CommandDispatcher::kPositionJointCount;
  static constexpr std::size_t kWheelJointCount = CommandDispatcher::kWheelJointCount;

  bool initInterfaces(hardware_interface::RobotHW* robot_hw);
  bool loadParameters(ros::NodeHandle& controller_nh);
  void buildHandles();
  void sensorInputCallback(const AutoSensorInput::ConstPtr& msg);
  void debugOverrideCallback(const AutoDebugOverride::ConstPtr& msg);
  sensor_msgs::JointState buildJointStateMessage(const ros::Time& stamp) const;
  sensor_msgs::Imu buildBaseImuMessage(const ros::Time& stamp) const;

  hardware_interface::JointStateInterface* joint_state_interface_{nullptr};
  hardware_interface::PositionJointInterface* position_joint_interface_{nullptr};
  hardware_interface::VelocityJointInterface* velocity_joint_interface_{nullptr};
  hardware_interface::ImuSensorInterface* imu_sensor_interface_{nullptr};

  std::array<std::string, kPositionJointCount> position_joint_names_{{"left_first_leg_joint", "left_second_leg_joint",
                                                                       "left_rod_joint", "right_first_leg_joint",
                                                                       "right_second_leg_joint", "right_rod_joint"}};
  std::array<std::string, kWheelJointCount> wheel_joint_names_{{"left_friction_wheel_joint", "right_friction_wheel_joint"}};
  std::string base_imu_name_{"base_imu"};

  std::array<hardware_interface::JointStateHandle, kPositionJointCount> joint_state_handles_{};
  CommandDispatcher::PositionJointHandles position_joint_handles_{};
  CommandDispatcher::WheelJointHandles wheel_joint_handles_{};
  hardware_interface::ImuSensorHandle base_imu_handle_{};

  AutoInputMux input_mux_{};
  RobotContext robot_context_{};
  CommandDispatcher command_dispatcher_{};
  CommandDispatcher::OutputMode output_mode_{CommandDispatcher::OutputMode::kNormal};
  AutoSensorInput sensor_input_{};
  AutoDebugOverride debug_override_{};
  ros::Subscriber sensor_input_sub_;
  ros::Subscriber debug_override_sub_;
  ros::Publisher state_trace_pub_;
  bool initialized_{false};
  
};
}  // namespace b29_smc_auto_controller
