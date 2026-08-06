// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include <b29_smc_auto_controller/AutoDebugOverride.h>
#include <b29_smc_auto_controller/AutoSensorInput.h>
#include <b29_smc_auto_controller/CompletePlannerControl.h>
#include <b29_smc_auto_controller/PlannerControlState.h>
#include <b29_smc_auto_controller/PlannerJointCommand.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <b29_smc_auto_controller/auto_input_mux.h>
#include <b29_smc_auto_controller/command_dispatcher.h>
#include <b29_smc_auto_controller/disconnect_cable_process.h>
#include <b29_smc_auto_controller/obstacle_crossing_runtime.h>
#include <b29_smc_auto_controller/planner_control_coordinator.h>
#include <b29_smc_auto_controller/remote_control_session.h>
#include <b29_smc_auto_controller/robot_context.h>

#include <steering_engine/common/auto_state_interface.h>
#include <steering_engine/common/remote_control_interface.h>

namespace b29_smc_auto_controller
{
struct ControllerTraceState;

class B29SmcAutoController
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          hardware_interface::PositionJointInterface,
                                                          hardware_interface::VelocityJointInterface,
                                                          hardware_interface::ImuSensorInterface,
                                                          steering_engine_hw::AutoStateInterface,
                                                          steering_engine_hw::RemoteControlInterface>
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

  enum class InputSource
  {
    AutoStateInterface,
    SensorInputTopic,
  };

  struct CallbackInputs
  {
    AutoSensorInput sensor_input;
    AutoDebugOverride debug_override;
    std::uint64_t debug_override_sequence{0};
  };

  bool initInterfaces(hardware_interface::RobotHW* robot_hw);
  bool loadParameters(ros::NodeHandle& controller_nh);
  void buildHandles();
  CallbackInputs copyCallbackInputs() const;
  void populateInputMux(const AutoSensorInput& sensor_input);
  void refreshInputMux(const CallbackInputs& inputs);
  AutoInputSnapshot buildInputSnapshot(const ros::Time& time);
  void dispatchAndPublish(const ros::Time& time, const AutoControlCommand& command);
  void sensorInputCallback(const AutoSensorInput::ConstPtr& msg);
  void debugOverrideCallback(const AutoDebugOverride::ConstPtr& msg);
  void plannerJointCommandCallback(const PlannerJointCommand::ConstPtr& msg);
  bool plannerReleaseCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool completePlannerControlCallback(CompletePlannerControl::Request& request,
                                      CompletePlannerControl::Response& response);
  bool softwareEmergencyStopCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool manualResetCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  sensor_msgs::JointState buildJointStateMessage(const ros::Time& stamp) const;
  sensor_msgs::Imu buildBaseImuMessage(const ros::Time& stamp) const;
  AutoControlCommand buildEffectiveCommand(const ros::Time& time);
  void getCurrentJointStateToCommand(AutoControlCommand& effective);
  bool validateEffectiveCommand(AutoControlCommand& effective);

  PlannerControlCoordinator::Positions currentPlannerReferencePositions() const;

  bool isSafetyBlocked(const AutoControlCommand& effective) const;
  void resetObstacleCrossingState(const ros::Time& time = ros::Time{});
  void updateObstacleCrossingRuntime(const ros::Time& time);
  void updateSignedWheelTravel();
  void resetWheelTravelBaseline();
  DisconnectCableProcess::Outcome collectDisconnectOutcome();
  ObstacleCrossingRuntime::PlannerOutcome collectPlannerOutcome(const ros::Time& time);
  bool collectRemoteControlCompletion();
  void applyRuntimeActions(const ObstacleCrossingRuntime::Actions& actions, const ros::Time& time);
  void applyObstacleCrossingCommand(const ros::Time& time, AutoControlCommand& effective);
  void applyPlannerCommandIfAvailable(const ros::Time& time, AutoControlCommand& effective);
  void applyGravityCompensationCommand(AutoControlCommand& effective);
  ControllerTraceState captureControllerTraceState(const ros::Time& stamp) const;
  void publishControllerTrace(const ros::Time& stamp, const AutoControlCommand& command) const;
  void stopWheels(AutoControlCommand& effective, const std::string& reason) const;
  void seedTargetsFromLastCommand(AutoControlCommand& effective) const;
  void rememberTargets(const AutoControlCommand& effective);
  void writeAutoStateCommand(const AutoControlCommand& command);
  void publishPlannerControlState(const ros::Time& time);
  static void setBothGrippers(AutoControlCommand& effective, GripperState state);
  static std::size_t jointIndex(JointIndex joint);

  hardware_interface::JointStateInterface* joint_state_interface_{nullptr};
  hardware_interface::PositionJointInterface* position_joint_interface_{nullptr};
  hardware_interface::VelocityJointInterface* velocity_joint_interface_{nullptr};
  hardware_interface::ImuSensorInterface* imu_sensor_interface_{nullptr};
  steering_engine_hw::AutoStateInterface* auto_state_interface_{nullptr};
  steering_engine_hw::RemoteControlInterface* remote_control_interface_{nullptr};

  std::array<std::string, kPositionJointCount> position_joint_names_{{"left_first_leg_joint", "left_second_leg_joint",
                                                                       "left_gripper_joint", "right_first_leg_joint",
                                                                       "right_second_leg_joint", "right_gripper_joint"}};
  std::array<std::string, kWheelJointCount> wheel_joint_names_{{"left_friction_wheel_joint", "right_friction_wheel_joint"}};
  std::string base_imu_name_{"base_imu"};
  std::string auto_state_name_{"auto_state"};
  std::string remote_control_name_{"remote_control"};
  InputSource input_source_{InputSource::AutoStateInterface};
  bool debug_validation_enabled_{false};
  bool simulation_only_{false};

  ObstacleCrossingRuntime crossing_runtime_{};
  DisconnectCableProcess disconnect_cable_process_{};
  std::array<double, kPositionJointCount> last_effective_joint_targets_{};
  bool has_last_effective_joint_targets_{false};
  bool remote_control_completion_rising_edge_{false};
  std::array<double, kWheelJointCount> wheel_travel_baseline_positions_{};
  std::array<double, kWheelJointCount> wheel_travel_current_positions_{};
  double signed_wheel_travel_{0.0};
  bool wheel_travel_baseline_initialized_{false};

  std::array<hardware_interface::JointStateHandle, kPositionJointCount> joint_state_handles_{};
  CommandDispatcher::PositionJointHandles position_joint_handles_{};
  CommandDispatcher::WheelJointHandles wheel_joint_handles_{};
  hardware_interface::ImuSensorHandle base_imu_handle_{};
  steering_engine_hw::AutoStateHandle auto_state_handle_{};
  steering_engine_hw::RemoteControlHandle remote_control_handle_{};

  AutoInputMuxConfig input_mux_config_{};
  AutoInputMux input_mux_{};
  RobotContext robot_context_{};
  CommandDispatcher command_dispatcher_{};
  CommandDispatcher::OutputMode output_mode_{CommandDispatcher::OutputMode::kNormal};
  AutoSensorInput sensor_input_{};
  AutoDebugOverride debug_override_{};
  std::uint64_t debug_override_sequence_{0};
  ros::Subscriber sensor_input_sub_;
  ros::Subscriber debug_override_sub_;
  ros::Subscriber planner_joint_command_sub_;
  ros::Publisher state_trace_pub_;
  ros::Publisher planner_control_state_pub_;
  ros::ServiceServer planner_release_service_;
  ros::ServiceServer complete_planner_control_service_;
  ros::ServiceServer software_emergency_stop_service_;
  ros::ServiceServer manual_reset_service_;

  mutable std::mutex input_mutex_;
  AutoInputSnapshot last_input_snapshot_{};
  AutoDebugOverride applied_debug_override_{};
  std::atomic<bool> software_emergency_stop_latched_{false};
  std::atomic<bool> manual_reset_requested_{false};
  PlannerControlCoordinator::DispatchTicket planner_dispatch_ticket_{};
  bool command_dispatch_attempted_{false};
  bool command_dispatch_succeeded_{false};
  PlannerControlCoordinator planner_control_coordinator_{};
  RemoteControlSession remote_control_session_{};

  bool initialized_{false};
};
}  // namespace b29_smc_auto_controller
