#include "b29_smc_auto_controller/b29_smc_auto_controller.h"

#include <algorithm>
#include <cmath>

#include <b29_smc_auto_controller/output_mode.h>
#include <pluginlib/class_list_macros.hpp>

namespace b29_smc_auto_controller
{
namespace
{
double clampUnit(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

const char* stageName(ObstacleCrossingStage stage)
{
  switch (stage)
  {
    case ObstacleCrossingStage::Idle:
      return "Idle";
    case ObstacleCrossingStage::CloseBothGrippers:
      return "CloseBothGrippers";
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
      return "OpenGripperBeforeGravityCompensation";
    case ObstacleCrossingStage::EnableGravityCompensation:
      return "EnableGravityCompensation";
    case ObstacleCrossingStage::Disconnecting:
      return "Disconnecting";
    case ObstacleCrossingStage::PlannerControl:
      return "PlannerControl";
    case ObstacleCrossingStage::RemoteControl:
      return "RemoteControl";
    case ObstacleCrossingStage::Regrip:
      return "Regrip";
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      return "ReopenBeforeRemoteControl";
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
      return "CompleteWaitObstacleClear";
    case ObstacleCrossingStage::ManualIntervention:
      return "ManualIntervention";
  }
  return "Unknown";
}

const char* sideName(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return "Left";
    case CrossingSide::Right:
      return "Right";
    case CrossingSide::None:
      return "None";
  }
  return "Unknown";
}

const char* sideReasonName(CrossingSide side)
{
  return side == CrossingSide::Right ? "right" : "left";
}

const char* actionName(CrossingAction action)
{
  switch (action)
  {
    case CrossingAction::CloseBothGrippers:
      return "CloseBothGrippers";
    case CrossingAction::DisconnectCable:
      return "DisconnectCable";
    case CrossingAction::PlannerControl:
      return "PlannerControl";
    case CrossingAction::Regrip:
      return "Regrip";
    case CrossingAction::None:
      return "None";
  }
  return "Unknown";
}

const char* actionReasonName(CrossingAction action)
{
  switch (action)
  {
    case CrossingAction::CloseBothGrippers:
      return "close_both_grippers";
    case CrossingAction::DisconnectCable:
      return "disconnect_cable";
    case CrossingAction::PlannerControl:
      return "planner_control";
    case CrossingAction::Regrip:
      return "regrip";
    case CrossingAction::None:
      return "none";
  }
  return "unknown";
}

const char* disconnectStepName(DisconnectCableStep step)
{
  switch (step)
  {
    case DisconnectCableStep::Idle:
      return "Idle";
    case DisconnectCableStep::Step1LoosenGripper:
      return "Step1LoosenGripper";
    case DisconnectCableStep::Step2WaitGripperRespond:
      return "Step2WaitGripperRespond";
    case DisconnectCableStep::Step3UpFirstJoint:
      return "Step3UpFirstJoint";
    case DisconnectCableStep::Step4MoveSecondJoint:
      return "Step4MoveSecondJoint";
    case DisconnectCableStep::Step5DownFirstJoint:
      return "Step5DownFirstJoint";
    case DisconnectCableStep::Step6MoveSecondJoint:
      return "Step6MoveSecondJoint";
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
      return "Step7CheckIfCableDisconnected";
    case DisconnectCableStep::Step8ReturnToZero:
      return "Step8ReturnToZero";
    case DisconnectCableStep::Done:
      return "Done";
    case DisconnectCableStep::Failed:
      return "Failed";
  }
  return "Unknown";
}

const char* driveModeName(DriveMode mode)
{
  switch (mode)
  {
    case DriveMode::Forward:
      return "Forward";
    case DriveMode::Stop:
    default:
      return "Stop";
  }
}

double elapsedSec(const ros::Time& now, const ros::Time& start)
{
  return start.isZero() ? 0.0 : std::max(0.0, (now - start).toSec());
}

const char* disconnectStepExpectedCondition(DisconnectCableStep step)
{
  switch (step)
  {
    case DisconnectCableStep::Step1LoosenGripper:
      return "send_open_gripper_command";
    case DisconnectCableStep::Step2WaitGripperRespond:
      return "wait_for_gripper_response_timeout";
    case DisconnectCableStep::Step3UpFirstJoint:
    case DisconnectCableStep::Step4MoveSecondJoint:
    case DisconnectCableStep::Step5DownFirstJoint:
    case DisconnectCableStep::Step6MoveSecondJoint:
    case DisconnectCableStep::Step8ReturnToZero:
      return "wait_for_motion_segment_and_interval";
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
      return "abs_joint_feedback_displacement_reaches_threshold";
    case DisconnectCableStep::Done:
      return "disconnect_success";
    case DisconnectCableStep::Failed:
      return "retry_or_manual_intervention";
    case DisconnectCableStep::Idle:
    default:
      return "";
  }
}
}

bool B29SmcAutoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  ros::NodeHandle root_nh;
  return init(robot_hw, root_nh, controller_nh);
}

bool B29SmcAutoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& /*root_nh*/,
                                ros::NodeHandle& controller_nh)
{
  if (!loadParameters(controller_nh))
  {
    return false;
  }

  if (!initInterfaces(robot_hw))
  {
    return false;
  }

  buildHandles();
  command_dispatcher_.configure(position_joint_handles_, wheel_joint_handles_);
  command_dispatcher_.setOutputMode(output_mode_);
  robot_context_.setTraceOutputMode(output_mode_);
  sensor_input_sub_ =
      controller_nh.subscribe("sensor_input", 1, &B29SmcAutoController::sensorInputCallback, this);
  debug_override_sub_ =
      controller_nh.subscribe("debug_override", 1, &B29SmcAutoController::debugOverrideCallback, this);
  planner_control_state_pub_ =
      controller_nh.advertise<PlannerControlState>(planner_control_state_topic_, 1);
  if (planner_interface_mode_ == "production")
  {
    planner_joint_command_sub_ = controller_nh.subscribe(
        planner_joint_command_topic_, 1, &B29SmcAutoController::plannerJointCommandCallback, this);
    complete_planner_control_service_ = controller_nh.advertiseService(
        complete_planner_control_service_name_, &B29SmcAutoController::completePlannerControlCallback, this);
  }
  state_trace_pub_ = controller_nh.advertise<AutoStateTrace>("state_trace", 10);
  planner_release_service_ =
      controller_nh.advertiseService("planner_release", &B29SmcAutoController::plannerReleaseCallback, this);
  software_emergency_stop_service_ =
      controller_nh.advertiseService("software_emergency_stop", &B29SmcAutoController::softwareEmergencyStopCallback, this);
  manual_reset_service_ =
      controller_nh.advertiseService("manual_reset", &B29SmcAutoController::manualResetCallback, this);
  initialized_ = true;
  return true;
}

void B29SmcAutoController::starting(const ros::Time& time)
{
  if (!initialized_)
  {
    return;
  }

  robot_context_.start();
  writeAutoStateCommand(AutoControlCommand{});
  AutoSensorInput sensor_input;
  AutoDebugOverride debug_override;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    sensor_input_.header.stamp = time;
    sensor_input = sensor_input_;
    debug_override = debug_override_;
  }

  bool use_data_fault_ = (use_auto_state_ == use_sensor_input_);

  if(use_data_fault_)
  {
    const std::string state = (use_auto_state_ && use_sensor_input_) ? "true" : "false";
    ROS_WARN_THROTTLE(5.0, "Both use_auto_state and use_sensor_input are set to %s. The controller will use sensor input and auto "
             "state data interchangeably, treating missing data as faults.", state.c_str());
  }
  else if (use_auto_state_)
  {
    input_mux_.setAutoState(auto_state_handle_.getData());
    input_mux_.setRemoteControl(remote_control_handle_.getData());
  }
  else
  {
    input_mux_.setSensorInput(sensor_input);
  }

  applied_debug_override_ = debug_validation_enabled_ ? debug_override : AutoDebugOverride{};
  input_mux_.setDebugOverride(applied_debug_override_);
  command_dispatch_attempted_ = true;
  command_dispatch_succeeded_ = command_dispatcher_.dispatch(robot_context_.currentCommand());
  state_trace_pub_.publish(buildControllerTrace(time, robot_context_.currentCommand()));
  publishPlannerControlState(time);
}

void B29SmcAutoController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  if (!initialized_)
  {
    return;
  }

  AutoSensorInput sensor_input;
  AutoDebugOverride debug_override;
  std::uint64_t debug_override_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    sensor_input = sensor_input_;
    debug_override = debug_override_;
    debug_override_sequence = debug_override_sequence_;
  }

  bool use_data_fault_ = ((use_auto_state_ && use_sensor_input_) == true) || 
                         ((use_auto_state_ || use_sensor_input_) == false);

  if(use_data_fault_)
  {
    ROS_WARN_THROTTLE(5.0, "Both use_auto_state and use_sensor_input are set to %s. The controller will use sensor input and auto "
             "state data interchangeably, treating missing data as faults.",
             (use_auto_state_ && use_sensor_input_) ? "true" : "false");
  }
  else if (use_auto_state_)
  {
    input_mux_.setAutoState(auto_state_handle_.getData());
    input_mux_.setRemoteControl(remote_control_handle_.getData());
  }
  else
  {
    input_mux_.setSensorInput(sensor_input);
  }

  applied_debug_override_ = debug_validation_enabled_ ? debug_override : AutoDebugOverride{};
  applied_debug_override_sequence_ = debug_validation_enabled_ ? debug_override_sequence : 0;
  input_mux_.setDebugOverride(applied_debug_override_);
  if (debug_validation_enabled_ && applied_debug_override_.enabled)
  {
    steering_engine_hw::RemoteControlData remote_control;
    remote_control.header = applied_debug_override_.header;
    remote_control.sample_sequence = applied_debug_override_sequence_;
    remote_control.valid =
        (applied_debug_override_.field_mask & AutoDebugOverride::FIELD_REMOTE_CONTROL_INCREMENTS) != 0;
    if (remote_control.valid)
    {
      std::copy(applied_debug_override_.remote_control_joint_increments.begin(),
                applied_debug_override_.remote_control_joint_increments.end(),
                remote_control.joint_increments.begin());
    }
    const bool complete =
        (applied_debug_override_.field_mask & AutoDebugOverride::FIELD_REMOTE_CONTROL_COMPLETE) != 0 &&
        applied_debug_override_.remote_control_complete;
    if (applied_debug_override_sequence_ != 0 &&
        applied_debug_override_sequence_ != last_input_snapshot_.remote_control_sample_sequence)
    {
      if (!previous_debug_remote_control_complete_ && complete)
      {
        ++debug_remote_control_completion_rising_edge_sequence_;
      }
      previous_debug_remote_control_complete_ = complete;
    }
    remote_control.stage_complete = complete;
    remote_control.completion_rising_edge_sequence =
        debug_remote_control_completion_rising_edge_sequence_;
    input_mux_.setRemoteControl(remote_control);
  }
  input_mux_.setJointState(buildJointStateMessage(time));
  input_mux_.setBaseImu(buildBaseImuMessage(time));
  AutoInputSnapshot snapshot = input_mux_.buildSnapshot();
  snapshot.emergency_stop = snapshot.emergency_stop || software_emergency_stop_latched_.load();
  snapshot.manual_reset_requested = snapshot.manual_reset_requested || manual_reset_requested_.exchange(false);
  last_input_snapshot_ = snapshot;
  robot_context_.setInputSnapshot(snapshot);

  robot_context_.tick50Hz();

  const AutoControlCommand effective = buildEffectiveCommand(time);
  writeAutoStateCommand(effective);
  command_dispatch_attempted_ = true;
  command_dispatch_succeeded_ = command_dispatcher_.dispatch(effective);
  state_trace_pub_.publish(buildControllerTrace(time, effective));
  publishPlannerControlState(time);
}

void B29SmcAutoController::stopping(const ros::Time& time)
{
  if (!initialized_)
  {
    return;
  }

  planner_session_.stop(PlannerControlState::EXIT_CONTROLLER_STOPPED, false);
  AutoControlCommand safe_stop;
  resetObstacleCrossingState(time);
  writeAutoStateCommand(safe_stop);
  command_dispatch_attempted_ = true;
  command_dispatch_succeeded_ = command_dispatcher_.dispatch(safe_stop);
  state_trace_pub_.publish(buildControllerTrace(time, safe_stop));
  publishPlannerControlState(time);
}

bool B29SmcAutoController::initInterfaces(hardware_interface::RobotHW* robot_hw)
{
  joint_state_interface_ = robot_hw->get<hardware_interface::JointStateInterface>();
  position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
  velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
  imu_sensor_interface_ = robot_hw->get<hardware_interface::ImuSensorInterface>();
  auto_state_interface_ = robot_hw->get<steering_engine_hw::AutoStateInterface>();
  remote_control_interface_ = robot_hw->get<steering_engine_hw::RemoteControlInterface>();

  if (!joint_state_interface_ || !position_joint_interface_ || !velocity_joint_interface_ || !imu_sensor_interface_)
  {
    ROS_WARN_THROTTLE(5.0, "b29_smc_auto_controller requires JointStateInterface, PositionJointInterface, "
              "VelocityJointInterface, and ImuSensorInterface.");
    return false;
  }

  if (use_auto_state_ && (!auto_state_interface_ || !remote_control_interface_))
  {
    ROS_WARN_THROTTLE(5.0, "b29_smc_auto_controller requires AutoStateInterface and RemoteControlInterface "
                             "when use_auto_state=true.");
    return false;
  }

  return true;
}

bool B29SmcAutoController::loadParameters(ros::NodeHandle& controller_nh)
{
  controller_nh.param<std::string>("joint_names/left_first_leg_joint", position_joint_names_[0], position_joint_names_[0]);
  controller_nh.param<std::string>("joint_names/left_second_leg_joint", position_joint_names_[1], position_joint_names_[1]);
  controller_nh.param<std::string>("joint_names/l_gripper_left_drive_joint", position_joint_names_[2], position_joint_names_[2]);
  controller_nh.param<std::string>("joint_names/right_first_leg_joint", position_joint_names_[3], position_joint_names_[3]);
  controller_nh.param<std::string>("joint_names/right_second_leg_joint", position_joint_names_[4], position_joint_names_[4]);
  controller_nh.param<std::string>("joint_names/r_gripper_left_drive_joint", position_joint_names_[5], position_joint_names_[5]);
  controller_nh.param<std::string>("joint_names/left_friction_wheel_joint", wheel_joint_names_[0], wheel_joint_names_[0]);
  controller_nh.param<std::string>("joint_names/right_friction_wheel_joint", wheel_joint_names_[1], wheel_joint_names_[1]);
  controller_nh.param<std::string>("imu_names", base_imu_name_, base_imu_name_);
  controller_nh.param<std::string>("auto_state_name", auto_state_name_, auto_state_name_);
  controller_nh.param<std::string>("remote_control_name", remote_control_name_, remote_control_name_);
  controller_nh.param<bool>("use_sensor_input", use_sensor_input_, use_sensor_input_);
  controller_nh.param<bool>("use_auto_state", use_auto_state_, use_auto_state_);
  controller_nh.param<bool>("debug_validation/enabled", debug_validation_enabled_, debug_validation_enabled_);
  controller_nh.param<bool>("debug_validation/simulation_only", simulation_only_, simulation_only_);
  controller_nh.param<bool>("planner_control/manual_release_enabled", planner_manual_release_enabled_,
                            planner_manual_release_enabled_);
  controller_nh.param<std::string>("planner_interface/mode", planner_interface_mode_, planner_interface_mode_);
  controller_nh.param<std::string>("planner_interface/state_topic", planner_control_state_topic_,
                                   planner_control_state_topic_);
  controller_nh.param<std::string>("planner_interface/command_topic", planner_joint_command_topic_,
                                   planner_joint_command_topic_);
  controller_nh.param<std::string>("planner_interface/complete_service", complete_planner_control_service_name_,
                                   complete_planner_control_service_name_);
  controller_nh.param<double>("planner_interface/max_delta_per_command",
                              planner_session_config_.max_delta_per_command,
                              planner_session_config_.max_delta_per_command);
  controller_nh.param<double>("planner_interface/command_timeout", planner_session_config_.command_timeout,
                              planner_session_config_.command_timeout);
  controller_nh.param<double>("planner_interface/total_watchdog_timeout",
                              planner_session_config_.total_watchdog_timeout,
                              planner_session_config_.total_watchdog_timeout);
  controller_nh.param<double>("posture_roll_limit", input_mux_config_.max_abs_roll_rad,
                              input_mux_config_.max_abs_roll_rad);
  controller_nh.param<double>("posture_pitch_limit", input_mux_config_.max_abs_pitch_rad,
                              input_mux_config_.max_abs_pitch_rad);
  controller_nh.param<double>("robot_motion/debug_planner_control_wait_time",
                              robot_motion_config_.debug_planner_control_wait_time,
                              robot_motion_config_.debug_planner_control_wait_time);
  controller_nh.param<double>("robot_motion/wait_for_grip_respond_time",
                              robot_motion_config_.wait_for_grip_respond_time,
                              robot_motion_config_.wait_for_grip_respond_time);
  controller_nh.param<double>("robot_motion/gravity_compensation_enable_wait_time",
                              robot_motion_config_.gravity_compensation_enable_wait_time,
                              robot_motion_config_.gravity_compensation_enable_wait_time);
  controller_nh.param<double>("robot_motion/disconnect_cable_step_motion_duration",
                              robot_motion_config_.disconnect_cable_step_motion_duration,
                              robot_motion_config_.disconnect_cable_step_motion_duration);
  controller_nh.param<double>("robot_motion/disconnect_cable_step_interval",
                              robot_motion_config_.disconnect_cable_step_interval,
                              robot_motion_config_.disconnect_cable_step_interval);
  controller_nh.param<double>("robot_motion/succeed_disconnect_cable_threshold",
                              robot_motion_config_.succeed_disconnect_cable_threshold,
                              robot_motion_config_.succeed_disconnect_cable_threshold);
  controller_nh.param<double>("robot_motion/disconnect_cable_second_joint_check_delta",
                              robot_motion_config_.disconnect_cable_second_joint_check_delta,
                              robot_motion_config_.disconnect_cable_second_joint_check_delta);
  controller_nh.param<int>("robot_motion/close_gripper_retry_limit",
                           robot_motion_config_.close_gripper_retry_limit,
                           robot_motion_config_.close_gripper_retry_limit);
  controller_nh.param<int>("robot_motion/disconnect_cable_retry_limit",
                           robot_motion_config_.disconnect_cable_retry_limit,
                           robot_motion_config_.disconnect_cable_retry_limit);
  controller_nh.param<int>("robot_motion/regrip_retry_limit",
                           robot_motion_config_.regrip_retry_limit,
                           robot_motion_config_.regrip_retry_limit);
  controller_nh.param<double>("robot_motion/disconnect_cable_up_joint_position",
                              robot_motion_config_.disconnect_cable_up_joint_position,
                              robot_motion_config_.disconnect_cable_up_joint_position);
  controller_nh.param<double>("robot_motion/disconnect_cable_down_joint_position",
                              robot_motion_config_.disconnect_cable_down_joint_position,
                              robot_motion_config_.disconnect_cable_down_joint_position);
  controller_nh.param<double>("robot_motion/disconnect_cable_move_joint_position",
                              robot_motion_config_.disconnect_cable_move_joint_position,
                              robot_motion_config_.disconnect_cable_move_joint_position);
  controller_nh.param<double>("remote_control/max_increment_per_sample",
                              remote_control_config_.max_increment_per_sample,
                              remote_control_config_.max_increment_per_sample);
  std::string output_mode_name = toString(output_mode_);
  controller_nh.param<std::string>("output_mode", output_mode_name, output_mode_name);

  if (planner_interface_mode_ != "production" && planner_interface_mode_ != "debug")
  {
    ROS_ERROR("planner_interface/mode must be either 'production' or 'debug'.");
    return false;
  }
  if (planner_control_state_topic_.empty() || planner_joint_command_topic_.empty() ||
      complete_planner_control_service_name_.empty())
  {
    ROS_ERROR("planner_interface topic and service names must not be empty.");
    return false;
  }
  if (planner_interface_mode_ == "production" && planner_manual_release_enabled_)
  {
    ROS_ERROR("Production Planner interface cannot be enabled together with manual planner release.");
    return false;
  }
  if (!std::isfinite(planner_session_config_.max_delta_per_command) ||
      planner_session_config_.max_delta_per_command <= 0.0)
  {
    ROS_ERROR("planner_interface/max_delta_per_command must be finite and positive.");
    return false;
  }
  if (!std::isfinite(planner_session_config_.command_timeout) || planner_session_config_.command_timeout <= 0.0)
  {
    ROS_ERROR("planner_interface/command_timeout must be finite and positive.");
    return false;
  }
  if (!std::isfinite(planner_session_config_.total_watchdog_timeout) ||
      planner_session_config_.total_watchdog_timeout <= 0.0)
  {
    ROS_ERROR("planner_interface/total_watchdog_timeout must be finite and positive.");
    return false;
  }

  const auto is_positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };

  if (!is_positive(input_mux_config_.max_abs_roll_rad))
  {
    ROS_ERROR("posture_roll_limit must be positive.");
    return false;
  }
  if (!is_positive(input_mux_config_.max_abs_pitch_rad))
  {
    ROS_ERROR("posture_pitch_limit must be positive.");
    return false;
  }

  const auto is_non_negative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  const auto is_finite = [](double value) {
    return std::isfinite(value);
  };

  if (!is_non_negative(robot_motion_config_.debug_planner_control_wait_time))
  {
    ROS_ERROR("robot_motion/debug_planner_control_wait_time must be non-negative.");
    return false;
  }
  if (!is_non_negative(robot_motion_config_.wait_for_grip_respond_time))
  {
    ROS_ERROR("robot_motion/wait_for_grip_respond_time must be non-negative.");
    return false;
  }
  if (!is_non_negative(robot_motion_config_.gravity_compensation_enable_wait_time))
  {
    ROS_ERROR("robot_motion/gravity_compensation_enable_wait_time must be non-negative.");
    return false;
  }
  if (!is_positive(robot_motion_config_.disconnect_cable_step_motion_duration))
  {
    ROS_ERROR("robot_motion/disconnect_cable_step_motion_duration must be positive.");
    return false;
  }
  if (!is_non_negative(robot_motion_config_.disconnect_cable_step_interval))
  {
    ROS_ERROR("robot_motion/disconnect_cable_step_interval must be non-negative.");
    return false;
  }
  if (!is_positive(robot_motion_config_.succeed_disconnect_cable_threshold))
  {
    ROS_ERROR("robot_motion/succeed_disconnect_cable_threshold must be positive.");
    return false;
  }
  if (!is_finite(robot_motion_config_.disconnect_cable_second_joint_check_delta))
  {
    ROS_ERROR("robot_motion/disconnect_cable_second_joint_check_delta must be finite.");
    return false;
  }
  if (!is_finite(robot_motion_config_.disconnect_cable_up_joint_position) ||
      !is_finite(robot_motion_config_.disconnect_cable_down_joint_position) ||
      !is_finite(robot_motion_config_.disconnect_cable_move_joint_position))
  {
    ROS_ERROR("robot_motion disconnect cable joint positions must be finite.");
    return false;
  }
  if (robot_motion_config_.close_gripper_retry_limit < 1 ||
      robot_motion_config_.disconnect_cable_retry_limit < 1 ||
      robot_motion_config_.regrip_retry_limit < 1)
  {
    ROS_ERROR("robot_motion retry limits must be at least 1.");
    return false;
  }
  if (!is_positive(remote_control_config_.max_increment_per_sample))
  {
    ROS_ERROR("remote_control/max_increment_per_sample must be finite and positive.");
    return false;
  }

  try
  {
    output_mode_ = parseOutputMode(output_mode_name);
  }
  catch (const std::invalid_argument&)
  {
    ROS_ERROR_STREAM("Unsupported output_mode: " << output_mode_name);
    return false;
  }

  input_mux_ = AutoInputMux(input_mux_config_);
  planner_session_.configure(planner_session_config_);
  remote_control_session_.configure(remote_control_config_.max_increment_per_sample);
  return true;
}

void B29SmcAutoController::buildHandles()
{
  for (std::size_t i = 0; i < joint_state_handles_.size(); ++i)
  {
    joint_state_handles_[i] = joint_state_interface_->getHandle(position_joint_names_[i]);
    position_joint_handles_[i] = position_joint_interface_->getHandle(position_joint_names_[i]);
  }

  for (std::size_t i = 0; i < wheel_joint_handles_.size(); ++i)
  {
    wheel_joint_handles_[i] = velocity_joint_interface_->getHandle(wheel_joint_names_[i]);
  }

  base_imu_handle_ = imu_sensor_interface_->getHandle(base_imu_name_);
  if (use_auto_state_)
  {
    auto_state_handle_ = auto_state_interface_->getHandle(auto_state_name_);
    remote_control_handle_ = remote_control_interface_->getHandle(remote_control_name_);
  }
}

void B29SmcAutoController::sensorInputCallback(const AutoSensorInput::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  sensor_input_ = *msg;
}

void B29SmcAutoController::debugOverrideCallback(const AutoDebugOverride::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  if (!debug_validation_enabled_)
  {
    ROS_WARN_THROTTLE(5.0, "Ignoring debug_override because debug_validation/enabled=false.");
    return;
  }

  std::lock_guard<std::mutex> lock(input_mutex_);
  debug_override_ = *msg;
  ++debug_override_sequence_;
}

bool B29SmcAutoController::plannerReleaseCallback(std_srvs::Trigger::Request& /*request*/,
                                                  std_srvs::Trigger::Response& response)
{
  if (!planner_manual_release_enabled_)
  {
    response.success = false;
    response.message = "planner_control/manual_release_enabled=false";
    return true;
  }

  if (!planner_control_active_for_service_.load())
  {
    response.success = false;
    response.message = "PlannerControl is not active";
    return true;
  }

  planner_release_requested_.store(true);
  response.success = true;
  response.message = "Planner release accepted; transition will occur in the next control cycle";
  return true;
}

bool B29SmcAutoController::completePlannerControlCallback(CompletePlannerControl::Request& request,
                                                          CompletePlannerControl::Response& response)
{
  if (planner_interface_mode_ != "production")
  {
    response.accepted = false;
    response.message = "formal Planner completion is disabled in debug interface mode";
    return true;
  }

  const PlannerSession::CompletionResult result =
      planner_session_.requestCompletion(request.session_id, request.final_sequence);
  response.accepted = result.accepted;
  response.message = result.message;
  return true;
}

bool B29SmcAutoController::softwareEmergencyStopCallback(std_srvs::Trigger::Request& /*request*/,
                                                         std_srvs::Trigger::Response& response)
{
  software_emergency_stop_latched_.store(true);
  response.success = true;
  response.message = "Software emergency stop latched";
  return true;
}

bool B29SmcAutoController::manualResetCallback(std_srvs::Trigger::Request& /*request*/,
                                               std_srvs::Trigger::Response& response)
{
  software_emergency_stop_latched_.store(false);
  manual_reset_requested_.store(true);
  response.success = true;
  response.message = "Manual reset requested; active faults may keep the controller in SafeStop";
  return true;
}

void B29SmcAutoController::plannerJointCommandCallback(const PlannerJointCommand::ConstPtr& msg)
{
  if (!msg || planner_interface_mode_ != "production")
  {
    return;
  }

  const PlannerSession::CommandResult result = planner_session_.acceptCommand(*msg, ros::Time::now());
  if (!result.accepted)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Rejecting Planner command session=" << msg->session_id
                             << " sequence=" << msg->sequence << ": " << result.message);
  }
}

AutoControlCommand B29SmcAutoController::buildEffectiveCommand(const ros::Time& time)
{
  AutoControlCommand effective = robot_context_.currentCommand();
  planner_override_applied_ = false;
  remote_control_completion_rising_edge_ = false;
  getCurrentJointStateToCommand(effective);
  planner_session_.refreshReferenceIfUncommanded(currentPlannerReferencePositions(), time);

  if (isSafetyBlocked(effective))
  {
    planner_session_.stop(PlannerControlState::EXIT_SAFETY_REVOKED, false);
    resetObstacleCrossingState(time);
    applyGravityCompensationCommand(effective);
    return effective;
  }

  seedTargetsFromLastCommand(effective);
  updateObstacleCrossingFsm(time);
  applyObstacleCrossingCommand(time, effective);
  applyPlannerCommandIfAllowed(time, effective);
  applyGravityCompensationCommand(effective);
  rememberTargets(effective);

  return effective;
}

bool B29SmcAutoController::isSafetyBlocked(const AutoControlCommand& effective) const
{
  return robot_context_.isSafeStop() || robot_context_.isCommsLoss() || effective.freeze_joints;
}

void B29SmcAutoController::resetObstacleCrossingState(const ros::Time& time)
{
  planner_session_.stop(PlannerControlState::EXIT_SAFETY_REVOKED, false);
  setObstacleCrossingStage(ObstacleCrossingStage::Idle, time, "crossing_reset");
  crossing_side_ = CrossingSide::None;
  first_crossing_side_ = CrossingSide::Left;
  failed_action_ = CrossingAction::None;
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "crossing_reset");
  planner_take_control_time_ = ros::Time{};
  gripper_wait_start_time_ = ros::Time{};
  grip_confirmation_armed_ = false;
  to_check_joint_pos_ = 0.0;
  disconnect_check_displacement_ = 0.0;
  planner_release_requested_.store(false);
  planner_release_received_for_stage_ = false;
  has_last_effective_joint_targets_ = false;
  remote_control_session_.reset();
  remote_control_completion_rising_edge_ = false;
  resetMotionSegment();
  resetRetryCounts();
}

void B29SmcAutoController::resetMotionSegment()
{
  motion_segment_ = MotionSegment{};
}

void B29SmcAutoController::resetRetryCounts()
{
  close_grippers_retry_count_ = 0;
  left_disconnect_retry_count_ = 0;
  right_disconnect_retry_count_ = 0;
  left_regrip_retry_count_ = 0;
  right_regrip_retry_count_ = 0;
}

void B29SmcAutoController::updateObstacleCrossingFsm(const ros::Time& time)
{
  if (obstacle_crossing_stage_ == ObstacleCrossingStage::Idle)
  {
    if (robot_context_.isTraversing() && robot_context_.isObstacleWithinCrossObstaclesDistance())
    {
      resetRetryCounts();
      resetMotionSegment();
      failed_action_ = CrossingAction::None;
      last_failure_reason_.clear();
      first_crossing_side_ = firstCrossingSideFromCruiseSpeed();
      crossing_side_ = first_crossing_side_;
      setDisconnectCableStep(DisconnectCableStep::Idle, time, "new_obstacle_detected");
      setObstacleCrossingStage(ObstacleCrossingStage::CloseBothGrippers, time, "obstacle_within_crossing_distance");
      gripper_wait_start_time_ = time;
      grip_confirmation_armed_ = !robot_context_.isGripConfirmed();
    }
    return;
  }

  switch (obstacle_crossing_stage_)
  {
    case ObstacleCrossingStage::CloseBothGrippers:
      if (!robot_context_.isGripConfirmed())
      {
        grip_confirmation_armed_ = true;
      }
      if (grip_confirmation_armed_ && robot_context_.isGripConfirmed())
      {
        close_grippers_retry_count_ = 0;
        grip_confirmation_armed_ = false;
        enterOpenGripperBeforeGravityCompensation(first_crossing_side_, time);
      }
      else if ((time - gripper_wait_start_time_) >=
               ros::Duration(robot_motion_config_.wait_for_grip_respond_time))
      {
        ++close_grippers_retry_count_;
        if (close_grippers_retry_count_ >= robot_motion_config_.close_gripper_retry_limit)
        {
          last_failure_reason_ = "close_both_grippers_retry_limit_reached";
          enterManualIntervention(CrossingAction::CloseBothGrippers, first_crossing_side_, time);
        }
        else
        {
          gripper_wait_start_time_ = time;
        }
      }
      break;
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
      if ((time - gripper_wait_start_time_) >=
          ros::Duration(robot_motion_config_.wait_for_grip_respond_time))
      {
        enterEnableGravityCompensation(crossing_side_, time);
      }
      break;
    case ObstacleCrossingStage::EnableGravityCompensation:
      if ((time - obstacle_crossing_stage_enter_time_) >=
          ros::Duration(robot_motion_config_.gravity_compensation_enable_wait_time))
      {
        enterDisconnecting(crossing_side_, time);
      }
      break;
    case ObstacleCrossingStage::PlannerControl:
      updatePlannerTakeover(time);
      break;
    case ObstacleCrossingStage::RemoteControl:
      updateRemoteControl(time);
      break;
    case ObstacleCrossingStage::Regrip:
      updateRegrip(time);
      break;
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      updateReopenBeforeRemoteControl(time);
      break;
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
      if (!robot_context_.isObstacleDetected())
      {
        resetObstacleCrossingState(time);
      }
      break;
    case ObstacleCrossingStage::ManualIntervention:
      updateManualIntervention(time);
      break;
    case ObstacleCrossingStage::Disconnecting:
    case ObstacleCrossingStage::Idle:
    default:
      break;
  }
}

void B29SmcAutoController::applyObstacleCrossingCommand(const ros::Time& time, AutoControlCommand& effective)
{
  switch (obstacle_crossing_stage_)
  {
    case ObstacleCrossingStage::Idle:
      if (robot_context_.isTraversing())
      {
        effective.joint_targets[jointIndex(JointIndex::LeftGripper)]  = GripperState::HALFOPEN;
        effective.joint_targets[jointIndex(JointIndex::RightGripper)] = GripperState::HALFOPEN;
      }
      break;
    case ObstacleCrossingStage::CloseBothGrippers:
      stopWheels(effective, "crossing_close_grippers");
      effective.joint_targets[jointIndex(JointIndex::LeftGripper)]  = GripperState::CLOSED;
      effective.joint_targets[jointIndex(JointIndex::RightGripper)] = GripperState::CLOSED;
      break;
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
    {
      const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
      stopWheels(effective, std::string("crossing_") + sideReasonName(crossing_side_) +
                                "_open_gripper_before_gravity_compensation");
      effective.joint_targets[jointIndex(JointIndex::LeftGripper)] = GripperState::CLOSED;
      effective.joint_targets[jointIndex(JointIndex::RightGripper)] = GripperState::CLOSED;
      effective.joint_targets[jointIndex(joints.gripper)] = GripperState::OPEN;
      break;
    }
    case ObstacleCrossingStage::EnableGravityCompensation:
    {
      const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
      stopWheels(effective, std::string("crossing_") + sideReasonName(crossing_side_) +
                                "_enable_gravity_compensation");
      effective.joint_targets[jointIndex(JointIndex::LeftGripper)] = GripperState::CLOSED;
      effective.joint_targets[jointIndex(JointIndex::RightGripper)] = GripperState::CLOSED;
      effective.joint_targets[jointIndex(joints.gripper)] = GripperState::OPEN;
      break;
    }
    case ObstacleCrossingStage::Disconnecting:
      stopWheels(effective, std::string("disconnect_") + sideReasonName(crossing_side_) + "_active");
      updateDisconnectCableStep(time, effective);
      break;
    case ObstacleCrossingStage::PlannerControl:
      stopWheels(effective, std::string("planner_") + sideReasonName(crossing_side_) + "_wait");
      break;
    case ObstacleCrossingStage::RemoteControl:
    {
      const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
      stopWheels(effective, std::string("remote_control_") + sideReasonName(crossing_side_) + "_active");
      const RemoteControlSession::Positions& targets = remote_control_session_.targets();
      effective.joint_targets[jointIndex(JointIndex::LeftFirstLeg)] = targets[0];
      effective.joint_targets[jointIndex(JointIndex::LeftSecondLeg)] = targets[1];
      effective.joint_targets[jointIndex(JointIndex::RightFirstLeg)] = targets[2];
      effective.joint_targets[jointIndex(JointIndex::RightSecondLeg)] = targets[3];
      effective.joint_targets[jointIndex(joints.gripper)] = GripperState::OPEN;
      break;
    }
    case ObstacleCrossingStage::Regrip:
    {
      const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
      stopWheels(effective, std::string("regrip_") + sideReasonName(crossing_side_) + "_wait");
      effective.joint_targets[jointIndex(joints.gripper)] = GripperState::CLOSED;
      break;
    }
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
    {
      const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
      stopWheels(effective, std::string("regrip_") + sideReasonName(crossing_side_) + "_reopen_before_remote_control");
      effective.joint_targets[jointIndex(joints.gripper)] = GripperState::OPEN;
      break;
    }
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
    {
      const double cruise_speed = robot_context_.getCruiseSpeed();
      effective.drive_mode = DriveMode::Forward;
      effective.left_wheel_speed = cruise_speed;
      effective.right_wheel_speed = cruise_speed;
      effective.stop_all = false;
      effective.freeze_joints = false;
      effective.joint_targets[jointIndex(JointIndex::LeftGripper)]  = GripperState::HALFOPEN;
      effective.joint_targets[jointIndex(JointIndex::RightGripper)] = GripperState::HALFOPEN;
      effective.command_reason = "crossing_complete_wait_obstacle_clear";
      break;
    }
    case ObstacleCrossingStage::ManualIntervention:
      stopWheels(effective, std::string("manual_intervention_") + actionReasonName(failed_action_));
      break;
    default:
      break;
  }
}

void B29SmcAutoController::applyPlannerCommandIfAllowed(const ros::Time& time, AutoControlCommand& effective)
{
  if (obstacle_crossing_stage_ != ObstacleCrossingStage::PlannerControl ||
      planner_interface_mode_  != "production")
  {
    return;
  }

  PlannerSession::Positions positions;
  if (!planner_session_.latestCommandIsFresh(time, positions))
  {
    return;
  }
  effective.joint_targets[jointIndex(JointIndex::LeftFirstLeg)]   = positions[0];
  effective.joint_targets[jointIndex(JointIndex::LeftSecondLeg)]  = positions[1];
  effective.joint_targets[jointIndex(JointIndex::RightFirstLeg)]  = positions[2];
  effective.joint_targets[jointIndex(JointIndex::RightSecondLeg)] = positions[3];
  effective.command_reason = std::string("planner_") + sideReasonName(crossing_side_) + "_control";
  planner_override_applied_ = true;
}

void B29SmcAutoController::applyGravityCompensationCommand(AutoControlCommand& effective) const
{
  const bool manual_side_crossing =
      obstacle_crossing_stage_ == ObstacleCrossingStage::ManualIntervention &&
      failed_action_ != CrossingAction::None &&
      failed_action_ != CrossingAction::CloseBothGrippers;
  const bool side_crossing_active =
      crossing_side_ != CrossingSide::None &&
      (obstacle_crossing_stage_ == ObstacleCrossingStage::EnableGravityCompensation ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::Disconnecting ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::PlannerControl ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::RemoteControl ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::Regrip ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::ReopenBeforeRemoteControl ||
       manual_side_crossing);
  if (!side_crossing_active)
  {
    effective.gravity_compensation_mode = GravityCompensationMode::Off;
    return;
  }

  const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
  if (joints.actuator_first_leg == JointIndex::LeftFirstLeg)
  {
    effective.gravity_compensation_mode = GravityCompensationMode::LeftFirstLeg;
  }
  else if (joints.actuator_first_leg == JointIndex::RightFirstLeg)
  {
    effective.gravity_compensation_mode = GravityCompensationMode::RightFirstLeg;
  }
  else
  {
    effective.gravity_compensation_mode = GravityCompensationMode::Off;
  }
}

void B29SmcAutoController::updateDisconnectCableStep(const ros::Time& time, AutoControlCommand& effective)
{
  if (crossing_side_ == CrossingSide::None)
  {
    return;
  }

  const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
  effective.joint_targets[jointIndex(joints.gripper)] = GripperState::OPEN;

  if (disconnect_cable_step_ == DisconnectCableStep::Idle)
  {
    setDisconnectCableStep(DisconnectCableStep::Step1LoosenGripper, time, "disconnect_started");
  }

  const std::string prefix = std::string("disconnect_") + sideReasonName(crossing_side_) + "_";

  switch (disconnect_cable_step_)
  {
    case DisconnectCableStep::Step1LoosenGripper:
      effective.command_reason = prefix + "step1_loosen_gripper";
      gripper_wait_start_time_ = time;
      setDisconnectCableStep(DisconnectCableStep::Step2WaitGripperRespond, time, "open_gripper_command_sent");
      break;
    case DisconnectCableStep::Step2WaitGripperRespond:
      effective.command_reason = prefix + "step2_wait_gripper";
      if ((time - gripper_wait_start_time_) >= ros::Duration(robot_motion_config_.wait_for_grip_respond_time))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step3UpFirstJoint, time, "gripper_response_wait_elapsed");
      }
      break;
    case DisconnectCableStep::Step3UpFirstJoint:
      effective.command_reason = prefix + "step3_up_first_joint";
      if (applyMotionSegment(time, effective))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step4MoveSecondJoint, time, "step3_motion_completed");
      }
      break;
    case DisconnectCableStep::Step4MoveSecondJoint:
      effective.command_reason = prefix + "step4_move_second_joint";
      if (applyMotionSegment(time, effective))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step5DownFirstJoint, time, "step4_motion_completed");
      }
      break;
    case DisconnectCableStep::Step5DownFirstJoint:
      effective.command_reason = prefix + "step5_down_first_joint";
      if (applyMotionSegment(time, effective))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step6MoveSecondJoint, time, "step5_motion_completed");
      }
      break;
    case DisconnectCableStep::Step6MoveSecondJoint:
      effective.command_reason = prefix + "step6_move_second_joint";
      if (applyMotionSegment(time, effective))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step7CheckIfCableDisconnected, time, "step6_motion_completed");
      }
      break;
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
    {
      effective.command_reason = prefix + "step7_check_disconnect";
      const double current_pos = joint_state_handles_[jointIndex(joints.actuator_second_leg)].getPosition();
      disconnect_check_displacement_ = std::abs(current_pos - to_check_joint_pos_);
      if (disconnect_check_displacement_ >= robot_motion_config_.succeed_disconnect_cable_threshold)
      {
        setDisconnectCableStep(DisconnectCableStep::Done, time, "disconnect_displacement_threshold_reached");
        if (crossing_side_ == CrossingSide::Left)
        {
          left_disconnect_retry_count_ = 0;
        }
        else
        {
          right_disconnect_retry_count_ = 0;
        }
        enterPlannerControl(time, crossing_side_);
      }
      else
      {
        resetMotionSegment();
        last_failure_reason_ = prefix + "step7_displacement_below_threshold";
        setDisconnectCableStep(DisconnectCableStep::Step8ReturnToZero, time, "disconnect_displacement_below_threshold");
      }
      break;
    }
    case DisconnectCableStep::Step8ReturnToZero:
      effective.command_reason = prefix + "step8_return_to_zero";
      if (applyMotionSegment(time, effective))
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Failed, time, "return_to_zero_completed");
      }
      break;
    case DisconnectCableStep::Failed:
    {
      int& retry_count = crossing_side_ == CrossingSide::Left ? left_disconnect_retry_count_ : right_disconnect_retry_count_;
      ++retry_count;
      if (retry_count >= robot_motion_config_.disconnect_cable_retry_limit)
      {
        last_failure_reason_ = prefix + "retry_limit_reached";
        enterManualIntervention(CrossingAction::DisconnectCable, crossing_side_, time);
      }
      else
      {
        resetMotionSegment();
        setDisconnectCableStep(DisconnectCableStep::Step1LoosenGripper, time, "disconnect_retry");
      }
      break;
    }
    case DisconnectCableStep::Done:
    case DisconnectCableStep::Idle:
    default:
      break;
  }
}

void B29SmcAutoController::updatePlannerTakeover(const ros::Time& time)
{
  if (planner_interface_mode_ == "production")
  {
    uint32_t completed_session = 0;
    uint32_t final_sequence = 0;
    if (planner_session_.consumeCompletionRequest(completed_session, final_sequence))
    {
      planner_session_.stop(PlannerControlState::EXIT_COMPLETED, true);
      enterRemoteControl(time, crossing_side_);
      return;
    }

    if (planner_session_.totalWatchdogExpired(time))
    {
      planner_session_.stop(PlannerControlState::EXIT_TOTAL_WATCHDOG_TIMEOUT, false);
      last_failure_reason_ = std::string("planner_") + sideReasonName(crossing_side_) +
                             "_total_watchdog_timeout";
      enterManualIntervention(CrossingAction::PlannerControl, crossing_side_, time);
    }
    return;
  }

  if (planner_manual_release_enabled_)
  {
    if (!planner_release_requested_.exchange(false))
    {
      return;
    }
    planner_release_received_for_stage_ = true;
    enterRemoteControl(time, crossing_side_, "planner_manual_release_received");
    return;
  }

  if ((time - planner_take_control_time_) <
      ros::Duration(robot_motion_config_.debug_planner_control_wait_time))
  {
    return;
  }

  enterRemoteControl(time, crossing_side_, "planner_control_timeout_elapsed");
}

void B29SmcAutoController::updateRemoteControl(const ros::Time& time)
{
  RemoteControlSession::Input input;
  input.increments = last_input_snapshot_.remote_control_joint_increments;
  input.valid = last_input_snapshot_.remote_control_input_valid;
  input.sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
  input.completion_rising_edge_sequence =
      last_input_snapshot_.remote_control_completion_rising_edge_sequence;
  const RemoteControlSession::UpdateResult result = remote_control_session_.update(input);
  if (result.completion_rising_edge)
  {
    remote_control_completion_rising_edge_ = true;
    setObstacleCrossingStage(ObstacleCrossingStage::Regrip, time,
                             "remote_control_completion_rising_edge");
    gripper_wait_start_time_ = time;
    grip_confirmation_armed_ = !robot_context_.isGripConfirmed();
    return;
  }
  if (result.sample_consumed && !result.increments_applied &&
      last_input_snapshot_.remote_control_input_valid)
  {
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite remote control joint increment sample");
  }
}

void B29SmcAutoController::updateRegrip(const ros::Time& time)
{
  if (!robot_context_.isGripConfirmed())
  {
    grip_confirmation_armed_ = true;
  }
  if (grip_confirmation_armed_ && robot_context_.isGripConfirmed())
  {
    grip_confirmation_armed_ = false;
    if (crossing_side_ == CrossingSide::Left)
    {
      left_regrip_retry_count_ = 0;
    }
    else
    {
      right_regrip_retry_count_ = 0;
    }

    if (crossing_side_ == first_crossing_side_)
    {
      enterOpenGripperBeforeGravityCompensation(oppositeCrossingSide(crossing_side_), time);
    }
    else
    {
      setObstacleCrossingStage(ObstacleCrossingStage::CompleteWaitObstacleClear, time, "both_sides_completed");
      crossing_side_ = CrossingSide::None;
      setDisconnectCableStep(DisconnectCableStep::Idle, time, "both_sides_completed");
      resetMotionSegment();
    }
    return;
  }

  if ((time - gripper_wait_start_time_) < ros::Duration(robot_motion_config_.wait_for_grip_respond_time))
  {
    return;
  }

  int& retry_count = crossing_side_ == CrossingSide::Left ? left_regrip_retry_count_ : right_regrip_retry_count_;
  ++retry_count;
  if (retry_count >= robot_motion_config_.regrip_retry_limit)
  {
    last_failure_reason_ = std::string("regrip_") + sideReasonName(crossing_side_) + "_retry_limit_reached";
    enterManualIntervention(CrossingAction::Regrip, crossing_side_, time);
    return;
  }

  setObstacleCrossingStage(ObstacleCrossingStage::ReopenBeforeRemoteControl, time,
                           "regrip_retry_reopen_before_remote_control");
  gripper_wait_start_time_ = time;
}

void B29SmcAutoController::updateReopenBeforeRemoteControl(const ros::Time& time)
{
  if ((time - gripper_wait_start_time_) < ros::Duration(robot_motion_config_.wait_for_grip_respond_time))
  {
    return;
  }

  enterRemoteControl(time, crossing_side_, "regrip_retry_reopen_completed");
}

void B29SmcAutoController::updateManualIntervention(const ros::Time& time)
{
  if (!robot_context_.isGripConfirmed())
  {
    return;
  }

  const CrossingAction action = failed_action_;
  const CrossingSide side = crossing_side_;
  clearRetryForAction(action, side);
  failed_action_ = CrossingAction::None;
  resetMotionSegment();
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "manual_intervention_confirmed");

  if (action == CrossingAction::CloseBothGrippers)
  {
    enterOpenGripperBeforeGravityCompensation(side == CrossingSide::None ? first_crossing_side_ : side, time);
  }
  else if (action == CrossingAction::DisconnectCable)
  {
    enterPlannerControl(time, side);
  }
  else if (action == CrossingAction::PlannerControl && side != CrossingSide::None)
  {
    enterRemoteControl(time, side, "manual_intervention_planner_confirmed");
  }
  else if (action == CrossingAction::Regrip && side == first_crossing_side_)
  {
    enterOpenGripperBeforeGravityCompensation(oppositeCrossingSide(side), time);
  }
  else if (action == CrossingAction::Regrip && side != CrossingSide::None)
  {
    setObstacleCrossingStage(ObstacleCrossingStage::CompleteWaitObstacleClear, time, "manual_intervention_regrip_confirmed");
    crossing_side_ = CrossingSide::None;
  }
  else
  {
    resetObstacleCrossingState(time);
  }
}

AutoStateTrace B29SmcAutoController::buildControllerTrace(const ros::Time& stamp,
                                                          const AutoControlCommand& command) const
{
  AutoStateTrace trace = robot_context_.buildTraceMessage(stamp);
  applyCommandToTrace(command, trace);
  applyControllerStateToTrace(stamp, command, trace);
  return trace;
}

void B29SmcAutoController::applyControllerStateToTrace(const ros::Time& stamp, const AutoControlCommand& command,
                                                       AutoStateTrace& trace) const
{
  trace.obstacle_crossing_active = obstacle_crossing_stage_ != ObstacleCrossingStage::Idle;
  trace.obstacle_crossing_stage = stageName(obstacle_crossing_stage_);
  trace.obstacle_crossing_side = sideName(crossing_side_);
  trace.first_crossing_side = sideName(first_crossing_side_);
  trace.obstacle_crossing_stage_enter_time = obstacle_crossing_stage_enter_time_;
  trace.obstacle_crossing_stage_elapsed_sec = elapsedSec(stamp, obstacle_crossing_stage_enter_time_);
  trace.obstacle_crossing_transition_reason = obstacle_crossing_transition_reason_;
  trace.disconnect_step = disconnectStepName(disconnect_cable_step_);
  trace.disconnect_step_enter_time = disconnect_step_enter_time_;
  trace.disconnect_step_elapsed_sec = elapsedSec(stamp, disconnect_step_enter_time_);
  trace.disconnect_step_transition_reason = disconnect_step_transition_reason_;
  trace.disconnect_step_expected_condition = disconnectStepExpectedCondition(disconnect_cable_step_);
  trace.disconnect_check_displacement = disconnect_check_displacement_;
  trace.to_check_joint_pos = to_check_joint_pos_;
  trace.succeed_disconnect_cable_threshold = robot_motion_config_.succeed_disconnect_cable_threshold;
  trace.manual_intervention_active = obstacle_crossing_stage_ == ObstacleCrossingStage::ManualIntervention;
  trace.manual_intervention_reason =
      trace.manual_intervention_active ? std::string("manual_intervention_") + actionReasonName(failed_action_) : "";
  trace.failed_action = actionName(failed_action_);
  trace.retry_count = currentRetryCount();
  trace.close_grippers_retry_count = close_grippers_retry_count_;
  trace.left_disconnect_retry_count = left_disconnect_retry_count_;
  trace.right_disconnect_retry_count = right_disconnect_retry_count_;
  trace.left_regrip_retry_count = left_regrip_retry_count_;
  trace.right_regrip_retry_count = right_regrip_retry_count_;
  trace.close_gripper_retry_limit = robot_motion_config_.close_gripper_retry_limit;
  trace.disconnect_cable_retry_limit = robot_motion_config_.disconnect_cable_retry_limit;
  trace.regrip_retry_limit = robot_motion_config_.regrip_retry_limit;
  trace.waiting_for_grip_confirmed =
      obstacle_crossing_stage_ == ObstacleCrossingStage::CloseBothGrippers ||
      obstacle_crossing_stage_ == ObstacleCrossingStage::Regrip;
  trace.grip_wait_elapsed_sec = trace.waiting_for_grip_confirmed ? elapsedSec(stamp, gripper_wait_start_time_) : 0.0;
  trace.grip_wait_timed_out =
      trace.waiting_for_grip_confirmed &&
      trace.grip_wait_elapsed_sec >= robot_motion_config_.wait_for_grip_respond_time;
  trace.last_failure_reason = last_failure_reason_;

  trace.planner_control_active = obstacle_crossing_stage_ == ObstacleCrossingStage::PlannerControl;
  trace.planner_manual_release_enabled = planner_manual_release_enabled_;
  trace.planner_release_received = planner_release_received_for_stage_ || planner_release_requested_.load();
  trace.planner_control_wait_elapsed_sec =
      trace.planner_control_active ? elapsedSec(stamp, planner_take_control_time_) : 0.0;
  trace.planner_point_available = plannerCommandAvailable();
  trace.planner_point_fresh = plannerCommandIsFreshForTrace(stamp);
  trace.planner_override_applied = planner_override_applied_;
  trace.remote_control_active = obstacle_crossing_stage_ == ObstacleCrossingStage::RemoteControl;
  const RemoteControlSession::Positions& raw_increments = remote_control_session_.rawIncrements();
  const RemoteControlSession::Positions& applied_increments = remote_control_session_.appliedIncrements();
  const RemoteControlSession::Positions& targets = remote_control_session_.targets();
  for (std::size_t index = 0; index < targets.size(); ++index)
  {
    trace.remote_control_raw_increments[index] = raw_increments[index];
    trace.remote_control_applied_increments[index] = applied_increments[index];
    trace.remote_control_joint_targets[index] = targets[index];
  }
  trace.remote_control_input_valid = last_input_snapshot_.remote_control_input_valid;
  trace.remote_control_sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
  trace.remote_control_complete = last_input_snapshot_.remote_control_complete;
  trace.remote_control_completion_rising_edge = remote_control_completion_rising_edge_;

  trace.base_command_reason = robot_context_.currentCommand().command_reason;
  trace.drive_mode = driveModeName(command.drive_mode);
  for (std::size_t i = 0; i < command.joint_targets.size(); ++i)
  {
    trace.joint_targets[i] = command.joint_targets[i];
  }
  trace.left_gripper_target = command.joint_targets[jointIndex(JointIndex::LeftGripper)];
  trace.right_gripper_target = command.joint_targets[jointIndex(JointIndex::RightGripper)];
  trace.command_dispatch_attempted = command_dispatch_attempted_;
  trace.command_dispatch_succeeded = command_dispatch_succeeded_;

  trace.current_side_gripper = "";
  trace.current_side_gripper_target = 0.0;
  if (crossing_side_ != CrossingSide::None)
  {
    const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
    trace.current_side_gripper = crossing_side_ == CrossingSide::Left ? "left" : "right";
    trace.current_side_gripper_target = command.joint_targets[jointIndex(joints.gripper)];
  }

  trace.debug_validation_enabled = debug_validation_enabled_;
  trace.simulation_only = simulation_only_;
  trace.debug_override_active = debug_validation_enabled_ && applied_debug_override_.enabled;
  trace.debug_obstacle_detected = applied_debug_override_.obstacle_detected;
  trace.debug_obstacle_distance = applied_debug_override_.range_to_obstacle;
  trace.software_emergency_stop_latched = software_emergency_stop_latched_.load();
  trace.manual_reset_requested = last_input_snapshot_.manual_reset_requested;
  trace.lower_alive = last_input_snapshot_.lower_alive;
  trace.imu_ready = last_input_snapshot_.imu_ready;
  trace.posture_ready = last_input_snapshot_.posture_ready;
  trace.grip_confirmed = last_input_snapshot_.grip_confirmed;
  trace.joint_fault = last_input_snapshot_.joint_fault;
  trace.grip_fault = last_input_snapshot_.grip_fault;
  trace.obstacle_detected = last_input_snapshot_.obstacle_detected;
  trace.range_to_obstacle = last_input_snapshot_.range_to_obstacle;
}

void B29SmcAutoController::enterManualIntervention(CrossingAction action, CrossingSide side, const ros::Time& time)
{
  planner_session_.stop(PlannerControlState::EXIT_MANUAL_INTERVENTION, false);
  setObstacleCrossingStage(ObstacleCrossingStage::ManualIntervention, time, "retry_limit_reached");
  crossing_side_ = side;
  failed_action_ = action;
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "manual_intervention_required");
  resetMotionSegment();
}

void B29SmcAutoController::enterPlannerControl(const ros::Time& time, CrossingSide side, const std::string& reason)
{
  crossing_side_ = side;
  setObstacleCrossingStage(ObstacleCrossingStage::PlannerControl, time, reason);
  planner_take_control_time_ = time;
  planner_release_requested_.store(false);
  planner_release_received_for_stage_ = false;
  if (planner_interface_mode_ == "production")
  {
    planner_session_.start(plannerCrossingSide(side), currentPlannerReferencePositions(), time);
  }
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "planner_control_started");
  resetMotionSegment();
}

void B29SmcAutoController::enterRemoteControl(const ros::Time& time, CrossingSide side,
                                              const std::string& reason)
{
  crossing_side_ = side;
  setObstacleCrossingStage(ObstacleCrossingStage::RemoteControl, time, reason);
  RemoteControlSession::Input input;
  input.increments = last_input_snapshot_.remote_control_joint_increments;
  input.valid = last_input_snapshot_.remote_control_input_valid;
  input.sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
  input.completion_rising_edge_sequence =
      last_input_snapshot_.remote_control_completion_rising_edge_sequence;
  remote_control_session_.start(currentPlannerReferencePositions(), input);
  remote_control_completion_rising_edge_ = false;
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "remote_control_started");
  resetMotionSegment();
}

void B29SmcAutoController::enterOpenGripperBeforeGravityCompensation(CrossingSide side,
                                                                     const ros::Time& time)
{
  crossing_side_ = side;
  setObstacleCrossingStage(ObstacleCrossingStage::OpenGripperBeforeGravityCompensation, time,
                           "grippers_closed_before_opening_crossing_side");
  gripper_wait_start_time_ = time;
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "waiting_open_gripper_before_gravity_compensation");
  resetMotionSegment();
}

void B29SmcAutoController::enterEnableGravityCompensation(CrossingSide side, const ros::Time& time)
{
  crossing_side_ = side;
  setObstacleCrossingStage(ObstacleCrossingStage::EnableGravityCompensation, time,
                           "crossing_side_gripper_open_wait_completed");
  setDisconnectCableStep(DisconnectCableStep::Idle, time, "waiting_gravity_compensation_command");
  resetMotionSegment();
}

void B29SmcAutoController::enterDisconnecting(CrossingSide side, const ros::Time& time)
{
  setObstacleCrossingStage(ObstacleCrossingStage::Disconnecting, time, "start_disconnect_side");
  crossing_side_ = side;
  setDisconnectCableStep(DisconnectCableStep::Step3UpFirstJoint, time,
                         "gripper_opened_and_gravity_compensation_enabled");
  resetMotionSegment();
}

void B29SmcAutoController::setObstacleCrossingStage(ObstacleCrossingStage stage, const ros::Time& time,
                                                    const std::string& reason)
{
  obstacle_crossing_stage_ = stage;
  obstacle_crossing_stage_enter_time_ = time.isZero() ? ros::Time::now() : time;
  obstacle_crossing_transition_reason_ = reason;
  planner_control_active_for_service_.store(stage == ObstacleCrossingStage::PlannerControl);
}

void B29SmcAutoController::setDisconnectCableStep(DisconnectCableStep step, const ros::Time& time,
                                                  const std::string& reason)
{
  disconnect_cable_step_ = step;
  disconnect_step_enter_time_ = time.isZero() ? ros::Time::now() : time;
  disconnect_step_transition_reason_ = reason;
}

bool B29SmcAutoController::applyMotionSegment(const ros::Time& time, AutoControlCommand& effective)
{
  if (!motion_segment_.initialized || motion_segment_.step != disconnect_cable_step_)
  {
    motion_segment_.step = disconnect_cable_step_;
    motion_segment_.start_time = time;
    motion_segment_.duration = ros::Duration(robot_motion_config_.disconnect_cable_step_motion_duration);
    motion_segment_.start_targets = effective.joint_targets;
    motion_segment_.target_targets = effective.joint_targets;
    motion_segment_.completed = false;
    motion_segment_.completed_time = ros::Time{};
    motion_segment_.initialized = true;

    const CrossingSideJoints joints = jointsForCrossingSide(crossing_side_);
    const double first_leg_target_scale =
        joints.actuator_first_leg == JointIndex::LeftFirstLeg ? -1.0 : 1.0;
    if (disconnect_cable_step_ == DisconnectCableStep::Step3UpFirstJoint)
    {
      motion_segment_.target_targets[jointIndex(joints.actuator_first_leg)] =
          first_leg_target_scale * robot_motion_config_.disconnect_cable_up_joint_position;
    }
    else if (disconnect_cable_step_ == DisconnectCableStep::Step4MoveSecondJoint)
    {
      motion_segment_.target_targets[jointIndex(joints.actuator_second_leg)] =
          robot_motion_config_.disconnect_cable_move_joint_position;
    }
    else if (disconnect_cable_step_ == DisconnectCableStep::Step5DownFirstJoint)
    {
      motion_segment_.target_targets[jointIndex(joints.actuator_first_leg)] =
          first_leg_target_scale * robot_motion_config_.disconnect_cable_down_joint_position;
    }
    else if (disconnect_cable_step_ == DisconnectCableStep::Step6MoveSecondJoint)
    {
      const std::size_t second_joint = jointIndex(joints.actuator_second_leg);
      to_check_joint_pos_ = joint_state_handles_[second_joint].getPosition();
      motion_segment_.target_targets[second_joint] =
          motion_segment_.start_targets[second_joint] +
          robot_motion_config_.disconnect_cable_second_joint_check_delta;
    }
    else if (disconnect_cable_step_ == DisconnectCableStep::Step8ReturnToZero)
    {
      motion_segment_.target_targets[jointIndex(joints.actuator_first_leg)] = 0.0;
      motion_segment_.target_targets[jointIndex(joints.actuator_second_leg)] = 0.0;
    }
  }

  const double duration = motion_segment_.duration.toSec();
  const double alpha = duration <= 0.0 ? 1.0 : clampUnit((time - motion_segment_.start_time).toSec() / duration);
  for (std::size_t i = 0; i < effective.joint_targets.size(); ++i)
  {
    const double start = motion_segment_.start_targets[i];
    const double target = motion_segment_.target_targets[i];
    effective.joint_targets[i] = start + (target - start) * alpha;
  }

  if (alpha < 1.0)
  {
    return false;
  }

  if (!motion_segment_.completed)
  {
    motion_segment_.completed = true;
    motion_segment_.completed_time = time;
  }

  return (time - motion_segment_.completed_time) >=
         ros::Duration(robot_motion_config_.disconnect_cable_step_interval);
}

void B29SmcAutoController::stopWheels(AutoControlCommand& effective, const std::string& reason) const
{
  effective.drive_mode = DriveMode::Stop;
  effective.left_wheel_speed = 0.0;
  effective.right_wheel_speed = 0.0;
  effective.stop_all = true;
  effective.freeze_joints = false;
  effective.command_reason = reason;
}

void B29SmcAutoController::seedTargetsFromLastCommand(AutoControlCommand& effective) const
{
  const bool cruise_soft_hold =
      robot_context_.isTraversing() &&
      (obstacle_crossing_stage_ == ObstacleCrossingStage::Idle ||
       obstacle_crossing_stage_ == ObstacleCrossingStage::CompleteWaitObstacleClear);
  if (cruise_soft_hold)
  {
    return;
  }

  if (has_last_effective_joint_targets_ &&
      obstacle_crossing_stage_ != ObstacleCrossingStage::ManualIntervention)
  {
    effective.joint_targets = last_effective_joint_targets_;
  }
}

void B29SmcAutoController::rememberTargets(const AutoControlCommand& effective)
{
  last_effective_joint_targets_ = effective.joint_targets;
  has_last_effective_joint_targets_ = true;
}

void B29SmcAutoController::writeAutoStateCommand(const AutoControlCommand& command)
{
  if (!auto_state_handle_.valid())
  {
    return;
  }
  auto_state_handle_.setGravityCompensationMode(static_cast<std::uint8_t>(command.gravity_compensation_mode));
}

void B29SmcAutoController::publishPlannerControlState(const ros::Time& time)
{
  planner_control_state_pub_.publish(planner_session_.buildState(time));
}

void B29SmcAutoController::clearRetryForAction(CrossingAction action, CrossingSide side)
{
  if (action == CrossingAction::CloseBothGrippers)
  {
    close_grippers_retry_count_ = 0;
  }
  else if (action == CrossingAction::DisconnectCable && side == CrossingSide::Left)
  {
    left_disconnect_retry_count_ = 0;
  }
  else if (action == CrossingAction::DisconnectCable && side == CrossingSide::Right)
  {
    right_disconnect_retry_count_ = 0;
  }
  else if (action == CrossingAction::Regrip && side == CrossingSide::Left)
  {
    left_regrip_retry_count_ = 0;
  }
  else if (action == CrossingAction::Regrip && side == CrossingSide::Right)
  {
    right_regrip_retry_count_ = 0;
  }
}

std::uint32_t B29SmcAutoController::currentRetryCount() const
{
  const CrossingAction action =
      obstacle_crossing_stage_ == ObstacleCrossingStage::ManualIntervention ? failed_action_ :
      obstacle_crossing_stage_ == ObstacleCrossingStage::CloseBothGrippers ? CrossingAction::CloseBothGrippers :
      obstacle_crossing_stage_ == ObstacleCrossingStage::Disconnecting ? CrossingAction::DisconnectCable :
      obstacle_crossing_stage_ == ObstacleCrossingStage::Regrip ? CrossingAction::Regrip :
      obstacle_crossing_stage_ == ObstacleCrossingStage::ReopenBeforeRemoteControl ? CrossingAction::Regrip :
      CrossingAction::None;
  const CrossingSide side = crossing_side_;

  if (action == CrossingAction::CloseBothGrippers)
  {
    return static_cast<std::uint32_t>(close_grippers_retry_count_);
  }
  if (action == CrossingAction::DisconnectCable && side == CrossingSide::Left)
  {
    return static_cast<std::uint32_t>(left_disconnect_retry_count_);
  }
  if (action == CrossingAction::DisconnectCable && side == CrossingSide::Right)
  {
    return static_cast<std::uint32_t>(right_disconnect_retry_count_);
  }
  if (action == CrossingAction::Regrip && side == CrossingSide::Left)
  {
    return static_cast<std::uint32_t>(left_regrip_retry_count_);
  }
  if (action == CrossingAction::Regrip && side == CrossingSide::Right)
  {
    return static_cast<std::uint32_t>(right_regrip_retry_count_);
  }
  return 0;
}

CrossingSide B29SmcAutoController::firstCrossingSideFromCruiseSpeed() const
{
  return robot_context_.getCruiseSpeed() < 0.0 ? CrossingSide::Right : CrossingSide::Left;
}

CrossingSide B29SmcAutoController::oppositeCrossingSide(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return CrossingSide::Right;
    case CrossingSide::Right:
      return CrossingSide::Left;
    case CrossingSide::None:
      return CrossingSide::None;
    default:
      return CrossingSide::None;
  }
}

B29SmcAutoController::CrossingSideJoints B29SmcAutoController::jointsForCrossingSide(CrossingSide side)
{
  struct CrossingSideJointEntry
  {
    CrossingSide side;
    CrossingSideJoints joints;
  };

  static constexpr CrossingSideJointEntry kCrossingSideJointTable[] = {
    {CrossingSide::Left,  {JointIndex::LeftGripper,  JointIndex::RightFirstLeg, JointIndex::RightSecondLeg}},
    {CrossingSide::Right, {JointIndex::RightGripper, JointIndex::LeftFirstLeg,  JointIndex::LeftSecondLeg}},
  };

  for (const CrossingSideJointEntry& entry : kCrossingSideJointTable)
  {
    if (entry.side == side)
    {
      return entry.joints;
    }
  }

  return kCrossingSideJointTable[0].joints;
}

uint8_t B29SmcAutoController::plannerCrossingSide(CrossingSide side)
{
  switch (side)
  {
    case CrossingSide::Left:
      return PlannerControlState::CROSSING_SIDE_LEFT;
    case CrossingSide::Right:
      return PlannerControlState::CROSSING_SIDE_RIGHT;
    case CrossingSide::None:
    default:
      return PlannerControlState::CROSSING_SIDE_NONE;
  }
}

std::size_t B29SmcAutoController::jointIndex(JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}

PlannerSession::Positions B29SmcAutoController::currentPlannerReferencePositions() const
{
  return PlannerSession::Positions{{
      joint_state_handles_[jointIndex(JointIndex::LeftFirstLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::LeftSecondLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::RightFirstLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::RightSecondLeg)].getPosition(),
  }};
}

bool B29SmcAutoController::plannerCommandAvailable() const
{
  const PlannerControlState state = planner_session_.buildState(ros::Time{});
  return planner_interface_mode_ == "production" && state.active && state.has_accepted_command;
}

bool B29SmcAutoController::plannerCommandIsFreshForTrace(const ros::Time& time) const
{
  PlannerSession::Positions positions;
  return planner_interface_mode_ == "production" && planner_session_.latestCommandIsFresh(time, positions);
}

void B29SmcAutoController::getCurrentJointStateToCommand(AutoControlCommand& effective)
{
  for (std::size_t i = 0; i < joint_state_handles_.size(); ++i)
  {
    effective.joint_targets[i] = joint_state_handles_[i].getPosition();
  }
}

sensor_msgs::JointState B29SmcAutoController::buildJointStateMessage(const ros::Time& stamp) const
{
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name.reserve(joint_state_handles_.size());
  joint_state.position.reserve(joint_state_handles_.size());
  joint_state.velocity.reserve(joint_state_handles_.size());
  joint_state.effort.reserve(joint_state_handles_.size());

  for (const auto& handle : joint_state_handles_)
  {
    joint_state.name.push_back(handle.getName());
    joint_state.position.push_back(handle.getPosition());
    joint_state.velocity.push_back(handle.getVelocity());
    joint_state.effort.push_back(handle.getEffort());
  }

  return joint_state;
}

sensor_msgs::Imu B29SmcAutoController::buildBaseImuMessage(const ros::Time& stamp) const
{
  sensor_msgs::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = base_imu_name_;

  const double* orientation = base_imu_handle_.getOrientation();
  imu.orientation.x = orientation[0];
  imu.orientation.y = orientation[1];
  imu.orientation.z = orientation[2];
  imu.orientation.w = orientation[3];

  const double* angular_velocity = base_imu_handle_.getAngularVelocity();
  imu.angular_velocity.x = angular_velocity[0];
  imu.angular_velocity.y = angular_velocity[1];
  imu.angular_velocity.z = angular_velocity[2];

  const double* linear_acceleration = base_imu_handle_.getLinearAcceleration();
  imu.linear_acceleration.x = linear_acceleration[0];
  imu.linear_acceleration.y = linear_acceleration[1];
  imu.linear_acceleration.z = linear_acceleration[2];

  return imu;
}
}  // namespace b29_smc_auto_controller

PLUGINLIB_EXPORT_CLASS(b29_smc_auto_controller::B29SmcAutoController, controller_interface::ControllerBase)
