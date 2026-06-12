// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

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
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>

#include <b29_smc_auto_controller/auto_input_mux.h>
#include <b29_smc_auto_controller/command_dispatcher.h>
#include <b29_smc_auto_controller/robot_context.h>

#include <steering_engine/common/auto_state_interface.h>

namespace b29_smc_auto_controller
{
enum class DisconnectCableStep
{
  Idle,
  Step1LoosenGripper,
  Step2WaitGripperRespond,
  Step3UpJoint,
  Step4MoveSecondJoint,
  Step5DownFirstJoint,
  Step6MoveSecondJoint,
  Step7CheckIfCableDisconnected,
  Step8ReturnToZero,
  Done,
  Failed,
};

enum GripperState
{
  OPEN = 1,
  CLOSED,
  HALFOPEN,
};

enum class JointIndex
{
  LeftFirstLeg = 0,
  LeftSecondLeg,
  LeftGripper,
  RightFirstLeg,
  RightSecondLeg,
  RightGripper,
};

enum class ObstacleCrossingStage
{
  Idle,
  CloseBothGrippers,
  Disconnecting,
  PlannerControl,
  Regrip,
  CompleteWaitObstacleClear,
  ManualIntervention,
};

enum class CrossingSide
{
  None,
  Left,
  Right,
};

enum class CrossingAction
{
  None,
  CloseBothGrippers,
  DisconnectCable,
  PlannerRegrip,
};

class B29SmcAutoController
  : public controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                          hardware_interface::PositionJointInterface,
                                                          hardware_interface::VelocityJointInterface,
                                                          hardware_interface::ImuSensorInterface,
                                                          steering_engine_hw::AutoStateInterface>
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

  struct PlannerJointPoint
  {
    double left_first{0.0};
    double left_second{0.0};
    double right_first{0.0};
    double right_second{0.0};
    ros::Time stamp{};
  };

  struct MotionSegment
  {
    DisconnectCableStep step{DisconnectCableStep::Idle};
    ros::Time start_time{};
    ros::Duration duration{};
    std::array<double, kPositionJointCount> start_targets{};
    std::array<double, kPositionJointCount> target_targets{};
    bool initialized{false};
    bool completed{false};
    ros::Time completed_time{};
  };

  struct RobotMotionConfig
  {
    double wait_for_planner_point_control_time{10.0};
    double wait_for_grip_respond_time{5.0};
    double disconnect_cable_step_motion_duration{3.0};
    double disconnect_cable_step_interval{0.5};
    double succeed_disconnect_cable_threshold{0.17};
    double disconnect_cable_second_joint_check_delta{0.20};
    int close_gripper_retry_limit{2};
    int disconnect_cable_retry_limit{2};
    int planner_regrip_retry_limit{2};
    double disconnect_cable_up_joint_position{0.5};
    double disconnect_cable_down_joint_position{0.5};
    double disconnect_cable_move_joint_position{0.5};
  };

  struct CrossingSideJoints
  {
    JointIndex gripper;
    JointIndex actuator_first_leg;
    JointIndex actuator_second_leg;
  };

  bool initInterfaces(hardware_interface::RobotHW* robot_hw);
  bool loadParameters(ros::NodeHandle& controller_nh);
  void buildHandles();
  void sensorInputCallback(const AutoSensorInput::ConstPtr& msg);
  void debugOverrideCallback(const AutoDebugOverride::ConstPtr& msg);
  void plannerInputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  bool plannerReleaseCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool softwareEmergencyStopCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool manualResetCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  sensor_msgs::JointState buildJointStateMessage(const ros::Time& stamp) const;
  sensor_msgs::Imu buildBaseImuMessage(const ros::Time& stamp) const;
  AutoControlCommand buildEffectiveCommand(const ros::Time& time);
  void getCurrentJointStateToCommand(AutoControlCommand& effective);

  bool validatePlannerPoint(const std::vector<double>& data, PlannerJointPoint& point, std::string& reason) const;
  bool latestPlannerPointIsFresh(const ros::Time& time, PlannerJointPoint& point);
  void clearPlannerPointState();

  bool isSafetyBlocked(const AutoControlCommand& effective) const;
  void resetObstacleCrossingState(const ros::Time& time = ros::Time{});
  void resetMotionSegment();
  void resetRetryCounts();
  void updateObstacleCrossingFsm(const ros::Time& time);
  void applyObstacleCrossingCommand(const ros::Time& time, AutoControlCommand& effective);
  void applyPlannerCommandIfAllowed(const ros::Time& time, AutoControlCommand& effective);
  void updateDisconnectCableStep(const ros::Time& time, AutoControlCommand& effective);
  void updatePlannerTakeover(const ros::Time& time);
  void updateRegrip(const ros::Time& time);
  void updateManualIntervention(const ros::Time& time);
  AutoStateTrace buildControllerTrace(const ros::Time& stamp, const AutoControlCommand& command) const;
  void applyControllerStateToTrace(const ros::Time& stamp, const AutoControlCommand& command, AutoStateTrace& trace) const;
  void enterManualIntervention(CrossingAction action, CrossingSide side, const ros::Time& time = ros::Time{});
  void enterPlannerControl(const ros::Time& time, CrossingSide side);
  void enterDisconnecting(CrossingSide side, const ros::Time& time = ros::Time{});
  void setObstacleCrossingStage(ObstacleCrossingStage stage, const ros::Time& time, const std::string& reason);
  void setDisconnectCableStep(DisconnectCableStep step, const ros::Time& time, const std::string& reason);
  bool applyMotionSegment(const ros::Time& time, AutoControlCommand& effective);
  void stopWheels(AutoControlCommand& effective, const std::string& reason) const;
  void seedTargetsFromLastCommand(AutoControlCommand& effective) const;
  void rememberTargets(const AutoControlCommand& effective);
  void clearRetryForAction(CrossingAction action, CrossingSide side);
  std::uint32_t currentRetryCount() const;
  CrossingSide firstCrossingSideFromCruiseSpeed() const;
  static CrossingSide oppositeCrossingSide(CrossingSide side);
  static CrossingSideJoints jointsForCrossingSide(CrossingSide side);
  static std::size_t jointIndex(JointIndex joint);
  bool plannerPointAvailable() const;
  bool plannerPointIsFreshForTrace(const ros::Time& time) const;

  hardware_interface::JointStateInterface* joint_state_interface_{nullptr};
  hardware_interface::PositionJointInterface* position_joint_interface_{nullptr};
  hardware_interface::VelocityJointInterface* velocity_joint_interface_{nullptr};
  hardware_interface::ImuSensorInterface* imu_sensor_interface_{nullptr};
  steering_engine_hw::AutoStateInterface* auto_state_interface_{nullptr};

  std::array<std::string, kPositionJointCount> position_joint_names_{{"left_first_leg_joint", "left_second_leg_joint",
                                                                       "left_gripper_joint", "right_first_leg_joint",
                                                                       "right_second_leg_joint", "right_gripper_joint"}};
  std::array<std::string, kWheelJointCount> wheel_joint_names_{{"left_friction_wheel_joint", "right_friction_wheel_joint"}};
  std::string base_imu_name_{"base_imu"};
  std::string auto_state_name_{"auto_state"};
  bool use_sensor_input_{false};
  bool use_auto_state_{true};
  bool debug_validation_enabled_{false};
  bool simulation_only_{false};
  bool planner_manual_release_enabled_{false};

  bool planner_input_enabled_{false};
  std::string planner_input_topic_{"planner_joint_point"};
  double planner_point_timeout_{1.0};
  double planner_point_max_delta_per_cycle_{0.10};
  ros::Time planner_take_control_time_{0.0};
  RobotMotionConfig robot_motion_config_{};

  DisconnectCableStep disconnect_cable_step_{DisconnectCableStep::Idle};
  ObstacleCrossingStage obstacle_crossing_stage_{ObstacleCrossingStage::Idle};
  CrossingSide crossing_side_{CrossingSide::None};
  CrossingSide first_crossing_side_{CrossingSide::Left};
  CrossingAction failed_action_{CrossingAction::None};
  MotionSegment motion_segment_{};
  ros::Time gripper_wait_start_time_{};
  ros::Time obstacle_crossing_stage_enter_time_{};
  ros::Time disconnect_step_enter_time_{};
  std::string obstacle_crossing_transition_reason_{};
  std::string disconnect_step_transition_reason_{};
  std::string last_failure_reason_{};
  double to_check_joint_pos_{0.0};
  double disconnect_check_displacement_{0.0};
  int close_grippers_retry_count_{0};
  int left_disconnect_retry_count_{0};
  int right_disconnect_retry_count_{0};
  int left_regrip_retry_count_{0};
  int right_regrip_retry_count_{0};
  std::array<double, kPositionJointCount> last_effective_joint_targets_{};
  bool has_last_effective_joint_targets_{false};

  std::array<hardware_interface::JointStateHandle, kPositionJointCount> joint_state_handles_{};
  CommandDispatcher::PositionJointHandles position_joint_handles_{};
  CommandDispatcher::WheelJointHandles wheel_joint_handles_{};
  hardware_interface::ImuSensorHandle base_imu_handle_{};
  steering_engine_hw::AutoStateHandle auto_state_handle_{};

  AutoInputMuxConfig input_mux_config_{};
  AutoInputMux input_mux_{};
  RobotContext robot_context_{};
  CommandDispatcher command_dispatcher_{};
  CommandDispatcher::OutputMode output_mode_{CommandDispatcher::OutputMode::kNormal};
  AutoSensorInput sensor_input_{};
  AutoDebugOverride debug_override_{};
  ros::Subscriber sensor_input_sub_;
  ros::Subscriber debug_override_sub_;
  ros::Subscriber planner_input_sub_;
  ros::Publisher state_trace_pub_;
  ros::ServiceServer planner_release_service_;
  ros::ServiceServer software_emergency_stop_service_;
  ros::ServiceServer manual_reset_service_;

  PlannerJointPoint latest_planner_point_{};
  PlannerJointPoint last_accepted_planner_point_{};
  bool has_latest_planner_point_{false};
  bool has_last_accepted_planner_point_{false};
  mutable std::mutex planner_point_mutex_;
  mutable std::mutex input_mutex_;
  AutoInputSnapshot last_input_snapshot_{};
  AutoDebugOverride applied_debug_override_{};
  std::atomic<bool> planner_release_requested_{false};
  std::atomic<bool> planner_control_active_for_service_{false};
  std::atomic<bool> software_emergency_stop_latched_{false};
  std::atomic<bool> manual_reset_requested_{false};
  bool planner_release_received_for_stage_{false};
  bool planner_override_applied_{false};
  bool command_dispatch_attempted_{false};
  bool command_dispatch_succeeded_{false};

  bool initialized_{false};
};
}  // namespace b29_smc_auto_controller
