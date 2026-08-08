#include "b29_smc_auto_controller/b29_smc_auto_controller.h"

#include <algorithm>
#include <cmath>

#include <b29_smc_auto_controller/controller_trace_builder.h>
#include <b29_smc_auto_controller/output_mode.h>
#include <pluginlib/class_list_macros.hpp>

namespace b29_smc_auto_controller
{
namespace
{
double gripperTarget(GripperState state)
{
  return static_cast<double>(static_cast<uint8_t>(state));
}
}

bool B29SmcAutoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
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
  if (input_source_ == InputSource::SensorInputTopic)
  {
    sensor_input_sub_ =
        controller_nh.subscribe("sensor_input", 1, &B29SmcAutoController::sensorInputCallback, this);
  }
  if (debug_validation_enabled_)
  {
    debug_override_sub_ =
        controller_nh.subscribe("debug_override", 1, &B29SmcAutoController::debugOverrideCallback, this);
  }
  planner_control_state_pub_ = controller_nh.advertise<PlannerControlState>("planner_control_state", 1);
  if (planner_control_coordinator_.isNormalMode())
  {
    start_disconnect_sub_ = controller_nh.subscribe(
        "start_disconnect", 1, &B29SmcAutoController::startDisconnectCallback, this);
    start_flip_sub_ = controller_nh.subscribe(
        "start_flip", 1, &B29SmcAutoController::startFlipCallback, this);
    planner_joint_command_sub_ = controller_nh.subscribe(
        "planner_joint_command", 1, &B29SmcAutoController::plannerJointCommandCallback, this);
    complete_planner_control_service_ = controller_nh.advertiseService(
        "complete_planner_control", &B29SmcAutoController::completePlannerControlCallback, this);
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

bool B29SmcAutoController::init(hardware_interface::RobotHW* /*robot_hw*/, ros::NodeHandle& /*root_nh*/,
                                ros::NodeHandle& /*controller_nh*/)
{
  return true;
}

void B29SmcAutoController::starting(const ros::Time& time)
{
  if (!initialized_)
  {
    return;
  }

  robot_context_.start();
  resetWheelTravelBaseline();
  writeAutoStateCommand(AutoControlCommand{});
  CallbackInputs inputs = copyCallbackInputs();
  inputs.sensor_input.header.stamp = time;
  refreshInputMux(inputs);
  dispatchAndPublish(time, robot_context_.currentCommand());
}

void B29SmcAutoController::update(const ros::Time& time, const ros::Duration& period)
{
  if (!initialized_)
  {
    return;
  }

  refreshInputMux(copyCallbackInputs());
  const AutoInputSnapshot snapshot = buildInputSnapshot(time);
  robot_context_.setInputSnapshot(snapshot);
  processStartDisconnectRequest(snapshot);
  processStartFlipRequest(snapshot);

  robot_context_.tick(period);

  const AutoControlCommand effective = buildEffectiveCommand(time);
  writeAutoStateCommand(effective);
  dispatchAndPublish(time, effective);
}

void B29SmcAutoController::stopping(const ros::Time& time)
{
  if (!initialized_)
  {
    return;
  }

  AutoControlCommand safe_stop;
  resetObstacleCrossingState(time);
  planner_control_coordinator_.leave(PlannerControlCoordinator::ExitCause::ControllerStopped);
  applyGravityCompensationCommand(safe_stop);
  writeAutoStateCommand(safe_stop);
  dispatchAndPublish(time, safe_stop);
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
    ROS_WARN("b29_smc_auto_controller requires JointStateInterface, PositionJointInterface, "
             "VelocityJointInterface, and ImuSensorInterface.");
    return false;
  }

  if (!remote_control_interface_)
  {
    ROS_WARN("b29_smc_auto_controller requires RemoteControlInterface.");
    return false;
  }

  if (input_source_ == InputSource::AutoStateInterface && !auto_state_interface_)
  {
    ROS_WARN("b29_smc_auto_controller requires AutoStateInterface when use_auto_state=true.");
    return false;
  }

  return true;
}

bool B29SmcAutoController::loadParameters(ros::NodeHandle& controller_nh)
{
  ObstacleCrossingRuntime::Config crossing_config;
  DisconnectCableProcess::Config disconnect_config;
  PlannerControlCoordinator::Config planner_config;
  RemoteControlSession::Config remote_control_config;
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
  bool use_sensor_input = false;
  bool use_auto_state = true;
  controller_nh.param<bool>("use_sensor_input", use_sensor_input, use_sensor_input);
  controller_nh.param<bool>("use_auto_state", use_auto_state, use_auto_state);
  controller_nh.param<bool>("debug_validation/enabled", debug_validation_enabled_, debug_validation_enabled_);
  controller_nh.param<bool>("debug_validation/simulation_only", simulation_only_, simulation_only_);
  controller_nh.param<bool>("temporary_allow_start_without_grip_confirmed",
                            temporary_allow_start_without_grip_confirmed_,
                            temporary_allow_start_without_grip_confirmed_);
  std::string planner_interface_mode_name{"normal"};
  controller_nh.param<std::string>("planner_interface/mode", planner_interface_mode_name,
                                   planner_interface_mode_name);
  controller_nh.param<double>("planner_interface/max_delta_per_command",
                              planner_config.session.max_delta_per_command,
                              planner_config.session.max_delta_per_command);
  controller_nh.param<double>("planner_interface/command_timeout", planner_config.session.command_timeout,
                              planner_config.session.command_timeout);
  controller_nh.param<double>("planner_interface/total_watchdog_timeout",
                              planner_config.session.total_watchdog_timeout,
                              planner_config.session.total_watchdog_timeout);
  controller_nh.param<double>("posture_roll_limit", input_mux_config_.max_abs_roll_rad,
                              input_mux_config_.max_abs_roll_rad);
  controller_nh.param<double>("posture_pitch_limit", input_mux_config_.max_abs_pitch_rad,
                              input_mux_config_.max_abs_pitch_rad);
  controller_nh.param<double>("obstacle_crossing/wait_for_grip_respond_time",
                              crossing_config.wait_for_grip_respond_time,
                              crossing_config.wait_for_grip_respond_time);
  controller_nh.param<double>("obstacle_crossing/disconnect_cable_step_duration",
                              disconnect_config.step_duration,
                              disconnect_config.step_duration);
  controller_nh.param<double>("obstacle_crossing/disconnect_cable_step_interval",
                              disconnect_config.step_interval,
                              disconnect_config.step_interval);
  controller_nh.param<int>("obstacle_crossing/retry_limit", crossing_config.retry_limit,
                           crossing_config.retry_limit);
  controller_nh.param<double>("obstacle_crossing/disconnect_cable_first_joint_up_position",
                              disconnect_config.step3_first_joint_target,
                              disconnect_config.step3_first_joint_target);
  controller_nh.param<double>("obstacle_crossing/disconnect_cable_first_joint_down_position",
                              disconnect_config.step5_first_joint_target,
                              disconnect_config.step5_first_joint_target);
  controller_nh.param<double>("obstacle_crossing/disconnect_settle_velocity_threshold",
                              disconnect_config.settle_velocity_threshold,
                              disconnect_config.settle_velocity_threshold);
  controller_nh.param<double>("obstacle_crossing/disconnect_settle_duration",
                              disconnect_config.settle_duration,
                              disconnect_config.settle_duration);
  controller_nh.param<double>("remote_control/max_increment_per_sample",
                              remote_control_config.max_increment_per_sample,
                              remote_control_config.max_increment_per_sample);
  controller_nh.param<double>("remote_control/increment_deadband",
                              remote_control_config.increment_deadband,
                              remote_control_config.increment_deadband);
  controller_nh.param<double>("remote_control/increment_scale",
                              remote_control_config.increment_scale,
                              remote_control_config.increment_scale);
  controller_nh.param<double>("remote_control/joint_direction_signs/left_first",
                              remote_control_config.joint_direction_signs[0],
                              remote_control_config.joint_direction_signs[0]);
  controller_nh.param<double>("remote_control/joint_direction_signs/left_second",
                              remote_control_config.joint_direction_signs[1],
                              remote_control_config.joint_direction_signs[1]);
  controller_nh.param<double>("remote_control/joint_direction_signs/right_first",
                              remote_control_config.joint_direction_signs[2],
                              remote_control_config.joint_direction_signs[2]);
  controller_nh.param<double>("remote_control/joint_direction_signs/right_second",
                              remote_control_config.joint_direction_signs[3],
                              remote_control_config.joint_direction_signs[3]);
  std::string output_mode_name = toString(output_mode_);
  controller_nh.param<std::string>("output_mode", output_mode_name, output_mode_name);

  if (use_auto_state == use_sensor_input)
  {
    ROS_ERROR("Exactly one of use_auto_state and use_sensor_input must be true.");
    return false;
  }
  else if (use_auto_state)
  {
    input_source_ = InputSource::AutoStateInterface;
  }
  else
  {
    input_source_ = InputSource::SensorInputTopic;
  }

  if (planner_interface_mode_name == "normal")
  {
    planner_config.mode = PlannerControlCoordinator::Mode::Normal;
  }
  else if (planner_interface_mode_name == "debug")
  {
    planner_config.mode = PlannerControlCoordinator::Mode::Debug;
  }
  else
  {
    ROS_ERROR("planner_interface/mode must be either 'normal' or 'debug'.");
    return false;
  }
  disconnect_config.wait_for_grip_respond_time = crossing_config.wait_for_grip_respond_time;

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
  planner_control_coordinator_.configure(planner_config);
  crossing_runtime_.configure(crossing_config);
  disconnect_cable_process_.configure(disconnect_config);
  remote_control_session_.configure(remote_control_config);
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
  remote_control_handle_ = remote_control_interface_->getHandle(remote_control_name_);
  if (input_source_ == InputSource::AutoStateInterface)
  {
    auto_state_handle_ = auto_state_interface_->getHandle(auto_state_name_);
  }
}

B29SmcAutoController::CallbackInputs B29SmcAutoController::copyCallbackInputs() const
{
  std::lock_guard<std::mutex> lock(input_mutex_);

  CallbackInputs inputs;
  inputs.sensor_input = sensor_input_;
  inputs.debug_override = debug_override_;
  inputs.debug_override_sequence = debug_override_sequence_;
  return inputs;
}

void B29SmcAutoController::populateInputMux(const AutoSensorInput& sensor_input)
{
  input_mux_.setRemoteControl(remote_control_handle_.getData());
  switch (input_source_)
  {
    case InputSource::AutoStateInterface:
      input_mux_.setAutoState(auto_state_handle_.getData());
      break;
    case InputSource::SensorInputTopic:
      input_mux_.setSensorInput(sensor_input);
      break;
  }
}

void B29SmcAutoController::refreshInputMux(const CallbackInputs& inputs)
{
  populateInputMux(inputs.sensor_input);
  if (debug_validation_enabled_)
  {
    applied_debug_override_ = inputs.debug_override;
  }
  else
  {
    applied_debug_override_ = AutoDebugOverride{};
  }
  input_mux_.setDebugOverride(applied_debug_override_);
}

AutoInputSnapshot B29SmcAutoController::buildInputSnapshot(const ros::Time& time)
{
  input_mux_.setJointState(buildJointStateMessage(time));
  input_mux_.setBaseImu(buildBaseImuMessage(time));

  AutoInputSnapshot snapshot = input_mux_.buildSnapshot();
  snapshot.emergency_stop = snapshot.emergency_stop || software_emergency_stop_latched_.load();
  snapshot.manual_reset_requested = snapshot.manual_reset_requested || manual_reset_requested_.exchange(false);
  last_input_snapshot_ = snapshot;
  return snapshot;
}

void B29SmcAutoController::dispatchAndPublish(const ros::Time& time, const AutoControlCommand& command)
{
  command_dispatch_attempted_ = true;
  command_dispatch_succeeded_ = command_dispatcher_.dispatch(command);
  const bool physically_dispatched = command_dispatch_succeeded_ &&
      command_dispatcher_.outputMode() == CommandDispatcher::OutputMode::kNormal && !command.freeze_joints;
  planner_control_coordinator_.reportDispatch(planner_dispatch_ticket_, physically_dispatched);
  publishControllerTrace(time, command);
  publishPlannerControlState(time);
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

void B29SmcAutoController::startDisconnectCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  start_disconnect_requested_.store(true);
}

void B29SmcAutoController::startFlipCallback(const std_msgs::Empty::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  start_flip_requested_.store(true);
}

void B29SmcAutoController::processStartDisconnectRequest(const AutoInputSnapshot& snapshot)
{
  if (!start_disconnect_requested_.exchange(false))
  {
    return;
  }

  std::string rejection_reason;
  const std::string outer_state = robot_context_.currentStateName();
  if (crossing_runtime_.stage() != ObstacleCrossingStage::Idle)
  {
    rejection_reason = "start_disconnect_rejected_crossing_busy";
  }
  else if (outer_state != "Idle" && outer_state != "Traversing")
  {
    rejection_reason = "start_disconnect_rejected_outer_state_" + outer_state;
  }
  else if (snapshot.emergency_stop)
  {
    rejection_reason = "start_disconnect_rejected_emergency_stop";
  }
  else if (!snapshot.lower_alive)
  {
    rejection_reason = "start_disconnect_rejected_lower_not_alive";
  }
  else if (!snapshot.imu_ready)
  {
    rejection_reason = "start_disconnect_rejected_imu_not_ready";
  }
  else if (!snapshot.grip_confirmed && !temporary_allow_start_without_grip_confirmed_)
  {
    rejection_reason = "start_disconnect_rejected_grip_not_confirmed";
  }
  else if (snapshot.joint_fault || snapshot.grip_fault)
  {
    rejection_reason = "start_disconnect_rejected_hardware_fault";
  }

  if (!rejection_reason.empty())
  {
    crossing_runtime_.reportFailure(rejection_reason);
    ROS_WARN_STREAM("Rejecting start_disconnect request: " << rejection_reason);
    return;
  }

  if (!snapshot.grip_confirmed && temporary_allow_start_without_grip_confirmed_)
  {
    ROS_WARN("Accepting start_disconnect without grip_confirmed because the temporary startup bypass is enabled");
  }

  if (outer_state == "Idle")
  {
    robot_context_.requestAutoStart();
  }
  start_disconnect_event_pending_ = true;
  ROS_INFO("Accepted start_disconnect request; crossing side will follow signed wheel travel.");
}

void B29SmcAutoController::processStartFlipRequest(const AutoInputSnapshot& snapshot)
{
  if (!start_flip_requested_.exchange(false))
  {
    return;
  }

  std::string rejection_reason;
  if (crossing_runtime_.stage() != ObstacleCrossingStage::DisconnectDoneWaitFlip)
  {
    rejection_reason = "start_flip_rejected_not_waiting_after_disconnect";
  }
  else if (snapshot.emergency_stop)
  {
    rejection_reason = "start_flip_rejected_emergency_stop";
  }
  else if (!snapshot.lower_alive)
  {
    rejection_reason = "start_flip_rejected_lower_not_alive";
  }
  else if (!snapshot.imu_ready)
  {
    rejection_reason = "start_flip_rejected_imu_not_ready";
  }
  else if (snapshot.joint_fault || snapshot.grip_fault)
  {
    rejection_reason = "start_flip_rejected_hardware_fault";
  }

  if (!rejection_reason.empty())
  {
    crossing_runtime_.reportFailure(rejection_reason);
    ROS_WARN_STREAM("Rejecting start_flip request: " << rejection_reason);
    return;
  }

  start_flip_event_pending_ = true;
  ROS_INFO("Accepted start_flip request; entering PlannerControl for one MoveIt flip.");
}

bool B29SmcAutoController::plannerReleaseCallback(std_srvs::Trigger::Request& /*request*/,
                                                  std_srvs::Trigger::Response& response)
{
  const PlannerControlCoordinator::ManualReleaseResult result =
      planner_control_coordinator_.requestManualRelease();
  response.success = result.accepted;
  response.message = result.message;
  return true;
}

bool B29SmcAutoController::completePlannerControlCallback(CompletePlannerControl::Request& request,
                                                          CompletePlannerControl::Response& response)
{
  const PlannerSession::CompletionResult result =
      planner_control_coordinator_.requestCompletion(request.session_id, request.final_sequence);
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
  if (!msg)
  {
    return;
  }

  const PlannerSession::CommandResult result = planner_control_coordinator_.acceptCommand(*msg, ros::Time::now());
  if (!result.accepted)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Rejecting Planner command session=" << msg->session_id
                             << " sequence=" << msg->sequence << ": " << result.message);
  }
}

AutoControlCommand B29SmcAutoController::buildEffectiveCommand(const ros::Time& time)
{
  AutoControlCommand effective = robot_context_.currentCommand();
  planner_control_coordinator_.beginCycle();
  planner_dispatch_ticket_ = PlannerControlCoordinator::DispatchTicket{};
  remote_control_completion_rising_edge_ = false;
  updateSignedWheelTravel();
  getCurrentJointStateToCommand(effective);
  planner_control_coordinator_.refreshReferenceIfUncommanded(currentPlannerReferencePositions(), time);

  if (isSafetyBlocked(effective))
  {
    resetObstacleCrossingState(time);
    applyGravityCompensationCommand(effective);
    validateEffectiveCommand(effective);
    return effective;
  }

  seedTargetsFromLastCommand(effective);
  updateObstacleCrossingRuntime(time);
  applyObstacleCrossingCommand(time, effective);
  applyPlannerCommandIfAvailable(time, effective);
  applyGravityCompensationCommand(effective);
  validateEffectiveCommand(effective);
  rememberTargets(effective);

  return effective;
}

bool B29SmcAutoController::validateEffectiveCommand(AutoControlCommand& effective)
{
  const bool wheels_finite = std::isfinite(effective.left_wheel_speed) &&
                            std::isfinite(effective.right_wheel_speed);
  const bool targets_finite = std::all_of(
      effective.joint_targets.begin(), effective.joint_targets.end(),
      [](double target) { return std::isfinite(target); });
  if (wheels_finite && targets_finite)
  {
    return true;
  }

  getCurrentJointStateToCommand(effective);
  for (double& target : effective.joint_targets)
  {
    if (!std::isfinite(target))
    {
      target = 0.0;
    }
  }
  effective.drive_mode = DriveMode::Stop;
  effective.left_wheel_speed = 0.0;
  effective.right_wheel_speed = 0.0;
  effective.stop_all = true;
  effective.freeze_joints = true;
  effective.command_reason = "invalid_effective_command";
  crossing_runtime_.reportFailure("non_finite_effective_command");
  applyGravityCompensationCommand(effective);
  ROS_ERROR_THROTTLE(1.0, "Rejecting non-finite effective command and freezing joint targets");
  return false;
}

bool B29SmcAutoController::isSafetyBlocked(const AutoControlCommand& effective) const
{
  return robot_context_.isSafeStop() || robot_context_.isCommsLoss() || effective.freeze_joints;
}

void B29SmcAutoController::resetObstacleCrossingState(const ros::Time& time)
{
  has_last_effective_joint_targets_ = false;
  remote_control_completion_rising_edge_ = false;
  const ObstacleCrossingRuntime::Actions actions =
      crossing_runtime_.reset(time, robot_context_.isGripConfirmed(), "crossing_reset");
  applyRuntimeActions(actions, time);
}

DisconnectCableProcess::Outcome B29SmcAutoController::collectDisconnectOutcome()
{
  if (crossing_runtime_.stage() != ObstacleCrossingStage::Disconnecting)
  {
    return DisconnectCableProcess::Outcome::None;
  }
  return disconnect_cable_process_.consumeOutcome();
}

ObstacleCrossingRuntime::PlannerOutcome B29SmcAutoController::collectPlannerOutcome(const ros::Time& time)
{
  if (crossing_runtime_.stage() != ObstacleCrossingStage::PlannerControl)
  {
    return ObstacleCrossingRuntime::PlannerOutcome::None;
  }

  const PlannerControlCoordinator::Outcome outcome = planner_control_coordinator_.pollOutcome(time);
  switch (outcome)
  {
    case PlannerControlCoordinator::Outcome::Completed:
      return ObstacleCrossingRuntime::PlannerOutcome::Completed;
    case PlannerControlCoordinator::Outcome::ManualReleased:
      return ObstacleCrossingRuntime::PlannerOutcome::ManualReleased;
    case PlannerControlCoordinator::Outcome::TotalWatchdogExpired:
      return ObstacleCrossingRuntime::PlannerOutcome::TotalWatchdogExpired;
    case PlannerControlCoordinator::Outcome::None:
      break;
  }
  return ObstacleCrossingRuntime::PlannerOutcome::None;
}

bool B29SmcAutoController::collectRemoteControlCompletion()
{
  if (crossing_runtime_.stage() != ObstacleCrossingStage::RemoteControl)
  {
    return false;
  }

  RemoteControlSession::Input input;
  input.increments = last_input_snapshot_.remote_control_joint_increments;
  input.increments_valid = last_input_snapshot_.remote_control_increments_valid;
  input.sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
  input.completion_rising_edge_sequence =
      last_input_snapshot_.remote_control_completion_rising_edge_sequence;
  const RemoteControlSession::UpdateResult result = remote_control_session_.update(input);
  if (result.completion_rising_edge)
  {
    remote_control_completion_rising_edge_ = true;
    return true;
  }
  if (result.sample_consumed && !result.increments_applied && input.increments_valid)
  {
    ROS_WARN_THROTTLE(1.0, "Ignoring non-finite remote control joint increment sample");
  }
  return false;
}

void B29SmcAutoController::applyRuntimeActions(const ObstacleCrossingRuntime::Actions& actions,
                                                const ros::Time& time)
{
  switch (actions.planner_action)
  {
    case ObstacleCrossingRuntime::PlannerAction::Start:
      planner_control_coordinator_.enter(crossing_runtime_.crossingSide(),
                                         currentPlannerReferencePositions(), time);
      break;
    case ObstacleCrossingRuntime::PlannerAction::RevokeForSafety:
      planner_control_coordinator_.leave(PlannerControlCoordinator::ExitCause::SafetyRevoked);
      break;
    case ObstacleCrossingRuntime::PlannerAction::RevokeForManualIntervention:
      planner_control_coordinator_.leave(PlannerControlCoordinator::ExitCause::ManualIntervention);
      break;
    case ObstacleCrossingRuntime::PlannerAction::None:
      break;
  }
  if (actions.reset_disconnect_process)
  {
    disconnect_cable_process_.reset(time, "crossing_runtime_reset");
  }
  if (actions.start_disconnect_process)
  {
    const CrossingSideProfile* profile = crossing_runtime_.currentProfile();
    if (profile != nullptr)
    {
      disconnect_cable_process_.start(time, *profile, "gripper_opened_and_gravity_compensation_enabled");
    }
    else
    {
      crossing_runtime_.reportFailure("disconnect_invalid_crossing_side");
    }
  }
  if (actions.reset_remote_control_session)
  {
    remote_control_session_.reset();
    remote_control_completion_rising_edge_ = false;
  }
  if (actions.reset_wheel_travel)
  {
    resetWheelTravelBaseline();
  }
  if (actions.start_remote_control_session)
  {
    RemoteControlSession::Input input;
    input.increments = last_input_snapshot_.remote_control_joint_increments;
    input.increments_valid = last_input_snapshot_.remote_control_increments_valid;
    input.sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
    input.completion_rising_edge_sequence =
        last_input_snapshot_.remote_control_completion_rising_edge_sequence;
    remote_control_session_.start(currentPlannerReferencePositions(), input);
    remote_control_completion_rising_edge_ = false;
    ROS_INFO_STREAM("MoveIt flip completed on side "
                    << crossingSideReasonName(crossing_runtime_.crossingSide())
                    << "; lower-level RemoteControl handoff is active");
  }
  if (actions.remote_control_completed_regrip_entered)
  {
    ROS_INFO_STREAM("Lower-level RemoteControl completion received on side "
                    << crossingSideReasonName(crossing_runtime_.crossingSide())
                    << "; commanding the open gripper to close");
  }
  if (actions.disconnect_failed_hold_entered)
  {
    ROS_ERROR_STREAM("Cable disconnect validation failed on side "
                     << crossingSideReasonName(crossing_runtime_.crossingSide())
                     << "; holding current joint positions with no automatic retry");
  }
  if (actions.disconnect_succeeded_wait_flip_entered)
  {
    ROS_INFO_STREAM("Cable disconnect succeeded on side "
                    << crossingSideReasonName(crossing_runtime_.crossingSide())
                    << "; holding position and waiting for the automatic start_flip trigger");
  }
}

void B29SmcAutoController::updateObstacleCrossingRuntime(const ros::Time& time)
{
  ObstacleCrossingRuntime::Inputs inputs;
  inputs.traversing = robot_context_.isTraversing();
  inputs.obstacle_crossing_trigger = last_input_snapshot_.obstacle_crossing_trigger;
  inputs.obstacle_trigger_edge_sequences_valid =
      last_input_snapshot_.obstacle_trigger_edge_sequences_valid;
  inputs.obstacle_trigger_rising_edge_sequence =
      last_input_snapshot_.obstacle_trigger_rising_edge_sequence;
  inputs.obstacle_trigger_falling_edge_sequence =
      last_input_snapshot_.obstacle_trigger_falling_edge_sequence;
  inputs.signed_wheel_travel = signed_wheel_travel_;
  inputs.grip_confirmed = robot_context_.isGripConfirmed();

  ObstacleCrossingRuntime::Events events;
  events.start_disconnect_requested = start_disconnect_event_pending_;
  start_disconnect_event_pending_ = false;
  events.start_flip_requested = start_flip_event_pending_;
  start_flip_event_pending_ = false;
  events.disconnect = collectDisconnectOutcome();
  events.planner = collectPlannerOutcome(time);
  events.remote_control_completion_rising_edge = collectRemoteControlCompletion();

  const ObstacleCrossingRuntime::Actions actions = crossing_runtime_.update(time, inputs, events);
  applyRuntimeActions(actions, time);
}


void B29SmcAutoController::applyObstacleCrossingCommand(const ros::Time& time,
                                                         AutoControlCommand& effective)
{
  const ObstacleCrossingStage stage  = crossing_runtime_.stage();
  const CrossingSideProfile* profile = crossing_runtime_.currentProfile();
  switch (stage)
  {
    case ObstacleCrossingStage::Idle:
      if (robot_context_.isTraversing())
      {
        setBothGrippers(effective, GripperState::HalfOpen);
      }
      return;
    case ObstacleCrossingStage::CloseBothGrippers:
      stopWheels(effective, "crossing_close_grippers");
      setBothGrippers(effective, GripperState::Closed);
      return;
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
    {
      effective.freeze_joints = false;
      setBothGrippers(effective, GripperState::HalfOpen);
      effective.command_reason = "crossing_complete_wait_obstacle_clear";
      return;
    }
    case ObstacleCrossingStage::ManualIntervention:
      stopWheels(effective, crossing_runtime_.manualInterventionCommandReason());
      return;
    default:
      break;
  }

  if (profile == nullptr)
  {
    stopWheels(effective, "crossing_invalid_side");
    effective.freeze_joints = true;
    return;
  }

  const std::size_t gripper   = jointIndex(profile->gripper);
  const std::string side_name = crossingSideReasonName(profile->side);
  switch (stage)
  {
    case ObstacleCrossingStage::OpenGripperBeforeGravityCompensation:
      stopWheels(effective, "crossing_" + side_name + "_open_gripper_before_gravity_compensation");
      setBothGrippers(effective, GripperState::Closed);
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    case ObstacleCrossingStage::EnableGravityCompensation:
      stopWheels(effective, "crossing_" + side_name + "_enable_gravity_compensation");
      setBothGrippers(effective, GripperState::Closed);
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    case ObstacleCrossingStage::Disconnecting:
    {
      stopWheels(effective, "disconnect_" + side_name + "_active");
      DisconnectCableProcess::JointTargets feedback{};
      DisconnectCableProcess::JointTargets velocities{};
      for (std::size_t index = 0; index < feedback.size(); ++index)
      {
        feedback[index] = joint_state_handles_[index].getPosition();
        velocities[index] = joint_state_handles_[index].getVelocity();
      }
      const DisconnectCableProcess::Result result =
          disconnect_cable_process_.update(time, effective.joint_targets, feedback, velocities);
      if (!result.valid)
      {
        effective.freeze_joints = true;
        effective.command_reason = "disconnect_invalid_process_state";
        crossing_runtime_.reportFailure("disconnect_invalid_process_state");
        return;
      }
      effective.joint_targets = result.joint_targets;
      effective.command_reason = result.command_reason;
      return;
    }
    case ObstacleCrossingStage::DisconnectDoneWaitFlip:
      stopWheels(effective, "disconnect_" + side_name + "_done_wait_flip");
      setBothGrippers(effective, GripperState::Closed);
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    case ObstacleCrossingStage::PlannerControl:
      stopWheels(effective, "planner_" + side_name + "_wait");
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    case ObstacleCrossingStage::RemoteControl:
    {
      stopWheels(effective, "remote_control_" + side_name + "_active");
      const RemoteControlSession::Positions& targets = remote_control_session_.targets();
      effective.joint_targets[jointIndex(JointIndex::LeftFirstLeg)]   = targets[0];
      effective.joint_targets[jointIndex(JointIndex::LeftSecondLeg)]  = targets[1];
      effective.joint_targets[jointIndex(JointIndex::RightFirstLeg)]  = targets[2];
      effective.joint_targets[jointIndex(JointIndex::RightSecondLeg)] = targets[3];
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    }
    case ObstacleCrossingStage::Regrip:
      stopWheels(effective, "regrip_" + side_name + "_wait");
      effective.joint_targets[gripper] = gripperTarget(GripperState::Closed);
      return;
    case ObstacleCrossingStage::ReopenBeforeRemoteControl:
      stopWheels(effective, "regrip_" + side_name + "_reopen_before_remote_control");
      effective.joint_targets[gripper] = gripperTarget(GripperState::Open);
      return;
    case ObstacleCrossingStage::Idle:
    case ObstacleCrossingStage::CloseBothGrippers:
    case ObstacleCrossingStage::CompleteWaitObstacleClear:
    case ObstacleCrossingStage::ManualIntervention:
    default:
      return;
  }
}

void B29SmcAutoController::applyPlannerCommandIfAvailable(const ros::Time& time,
                                                           AutoControlCommand& effective)
{
  if (crossing_runtime_.stage() != ObstacleCrossingStage::PlannerControl)
  {
    return;
  }

  planner_dispatch_ticket_ = planner_control_coordinator_.commandCandidate(time);
  if (!planner_dispatch_ticket_.available)
  {
    return;
  }
  const PlannerControlCoordinator::Positions& positions = planner_dispatch_ticket_.positions;
  effective.joint_targets[jointIndex(JointIndex::LeftFirstLeg)]   = positions[0];
  effective.joint_targets[jointIndex(JointIndex::LeftSecondLeg)]  = positions[1];
  effective.joint_targets[jointIndex(JointIndex::RightFirstLeg)]  = positions[2];
  effective.joint_targets[jointIndex(JointIndex::RightSecondLeg)] = positions[3];
  effective.command_reason =
      std::string("planner_") + crossingSideReasonName(crossing_runtime_.crossingSide()) + "_control";
}

void B29SmcAutoController::applyGravityCompensationCommand(AutoControlCommand& effective)
{
  effective.gravity_compensation_mode = crossing_runtime_.gravityCompensationMode();
}

void B29SmcAutoController::publishControllerTrace(const ros::Time& stamp,
                                                   const AutoControlCommand& command) const
{
  const RobotContextTraceState robot_state = robot_context_.traceState();
  const ControllerTraceState controller_state = captureControllerTraceState(stamp);
  state_trace_pub_.publish(ControllerTraceBuilder::build(stamp, command, robot_state, controller_state));
}

ControllerTraceState B29SmcAutoController::captureControllerTraceState(const ros::Time& stamp) const
{
  ControllerTraceState state;
  const ObstacleCrossingRuntime::TraceState crossing_state = crossing_runtime_.traceState();
  const DisconnectCableProcess::TraceState disconnect_state = disconnect_cable_process_.traceState();
  state.obstacle_crossing_stage = crossing_state.stage;
  state.crossing_side = crossing_state.crossing_side;
  state.first_crossing_side = crossing_state.first_crossing_side;
  state.obstacle_crossing_stage_enter_time = crossing_state.stage_enter_time;
  state.obstacle_crossing_transition_reason = crossing_state.transition_reason;
  state.disconnect_step = disconnect_state.step;
  state.disconnect_step_enter_time = disconnect_state.step_enter_time;
  state.disconnect_step_transition_reason = disconnect_state.transition_reason;
  state.disconnect_max_abs_pose_joint_velocity = disconnect_state.max_abs_pose_joint_velocity;
  state.disconnect_settle_velocity_threshold = disconnect_state.settle_velocity_threshold;
  state.disconnect_velocity_within_threshold = disconnect_state.velocity_within_threshold;
  if (!disconnect_state.velocity_stable_since.isZero())
  {
    state.disconnect_velocity_stable_elapsed_sec =
        std::max(0.0, (stamp - disconnect_state.velocity_stable_since).toSec());
  }
  state.disconnect_settle_duration = disconnect_state.settle_duration;
  state.failed_operation = crossing_state.failed_operation;
  state.retry_count = crossing_state.retry_count;
  state.close_grippers_retry_count = crossing_state.close_grippers_retry_count;
  state.left_disconnect_retry_count = crossing_state.left_disconnect_retry_count;
  state.right_disconnect_retry_count = crossing_state.right_disconnect_retry_count;
  state.left_regrip_retry_count = crossing_state.left_regrip_retry_count;
  state.right_regrip_retry_count = crossing_state.right_regrip_retry_count;
  state.retry_limit = crossing_state.retry_limit;
  state.gripper_wait_start_time = crossing_state.gripper_wait_start_time;
  state.wait_for_grip_respond_time = crossing_state.wait_for_grip_respond_time;
  state.last_failure_reason = crossing_state.last_failure_reason;
  state.obstacle_crossing_trigger = crossing_state.obstacle_crossing_trigger;
  state.obstacle_trigger_rising_edge = crossing_state.obstacle_trigger_rising_edge;
  state.obstacle_trigger_falling_edge = crossing_state.obstacle_trigger_falling_edge;
  state.obstacle_trigger_rising_edge_sequence = crossing_state.obstacle_trigger_rising_edge_sequence;
  state.obstacle_trigger_falling_edge_sequence = crossing_state.obstacle_trigger_falling_edge_sequence;
  state.left_wheel_travel_baseline_position = wheel_travel_baseline_positions_[0];
  state.right_wheel_travel_baseline_position = wheel_travel_baseline_positions_[1];
  state.left_wheel_travel_current_position = wheel_travel_current_positions_[0];
  state.right_wheel_travel_current_position = wheel_travel_current_positions_[1];
  state.signed_wheel_travel = signed_wheel_travel_;
  state.wheel_travel_baseline_initialized = wheel_travel_baseline_initialized_;
  const PlannerControlCoordinator::TraceState planner_state =
      planner_control_coordinator_.traceState(stamp);
  state.planner_manual_release_enabled = planner_state.manual_release_enabled;
  state.planner_release_received = planner_state.release_received;
  state.planner_point_available = planner_state.point_available;
  state.planner_point_fresh = planner_state.point_fresh;
  state.planner_override_applied = planner_state.override_applied;
  const RemoteControlSession::Positions& raw_increments = remote_control_session_.rawIncrements();
  const RemoteControlSession::Positions& applied_increments = remote_control_session_.appliedIncrements();
  const RemoteControlSession::Positions& targets = remote_control_session_.targets();
  for (std::size_t index = 0; index < targets.size(); ++index)
  {
    state.remote_control_raw_increments[index] = raw_increments[index];
    state.remote_control_applied_increments[index] = applied_increments[index];
    state.remote_control_joint_targets[index] = targets[index];
  }
  state.remote_control_increments_valid = last_input_snapshot_.remote_control_increments_valid;
  state.remote_control_sample_sequence = last_input_snapshot_.remote_control_sample_sequence;
  state.remote_control_complete = last_input_snapshot_.remote_control_complete;
  state.remote_control_completion_rising_edge = remote_control_completion_rising_edge_;
  state.command_dispatch_attempted = command_dispatch_attempted_;
  state.command_dispatch_succeeded = command_dispatch_succeeded_;
  state.debug_validation_enabled = debug_validation_enabled_;
  state.simulation_only = simulation_only_;
  state.temporary_allow_start_without_grip_confirmed =
      temporary_allow_start_without_grip_confirmed_;
  state.debug_override_active = debug_validation_enabled_ && applied_debug_override_.enabled;
  state.debug_obstacle_crossing_trigger = applied_debug_override_.obstacle_crossing_trigger;
  state.software_emergency_stop_latched = software_emergency_stop_latched_.load();
  state.manual_reset_requested = last_input_snapshot_.manual_reset_requested;
  state.lower_alive = last_input_snapshot_.lower_alive;
  state.imu_ready = last_input_snapshot_.imu_ready;
  state.posture_ready = last_input_snapshot_.posture_ready;
  state.grip_confirmed = last_input_snapshot_.grip_confirmed;
  state.joint_fault = last_input_snapshot_.joint_fault;
  state.grip_fault = last_input_snapshot_.grip_fault;
  state.cruise_drive_request_raw = last_input_snapshot_.cruise_drive_request_raw;
  state.cruise_drive_request_valid = last_input_snapshot_.cruise_drive_request_valid;
  state.auto_start = last_input_snapshot_.auto_start_requested;
  state.manual_reset = last_input_snapshot_.manual_reset_requested;
  state.auto_start_rising_edge_sequence = last_input_snapshot_.auto_start_rising_edge_sequence;
  state.manual_reset_rising_edge_sequence = last_input_snapshot_.manual_reset_rising_edge_sequence;
  return state;
}

void B29SmcAutoController::updateSignedWheelTravel()
{
  for (std::size_t index = 0; index < wheel_joint_handles_.size(); ++index)
  {
    const double position = wheel_joint_handles_[index].getPosition();
    if (!std::isfinite(position))
    {
      return;
    }
    wheel_travel_current_positions_[index] = position;
  }

  if (!wheel_travel_baseline_initialized_)
  {
    wheel_travel_baseline_positions_ = wheel_travel_current_positions_;
    signed_wheel_travel_ = 0.0;
    wheel_travel_baseline_initialized_ = true;
    return;
  }

  const double left_travel = wheel_travel_current_positions_[0] - wheel_travel_baseline_positions_[0];
  const double right_travel = wheel_travel_current_positions_[1] - wheel_travel_baseline_positions_[1];
  signed_wheel_travel_ = (left_travel + right_travel) * 0.5;
}

void B29SmcAutoController::resetWheelTravelBaseline()
{
  wheel_travel_baseline_initialized_ = false;
  signed_wheel_travel_ = 0.0;
  updateSignedWheelTravel();
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

void B29SmcAutoController::setBothGrippers(AutoControlCommand& effective, GripperState state)
{
  const double target = gripperTarget(state);
  effective.joint_targets[jointIndex(JointIndex::LeftGripper)] = target;
  effective.joint_targets[jointIndex(JointIndex::RightGripper)] = target;
}

void B29SmcAutoController::seedTargetsFromLastCommand(AutoControlCommand& effective) const
{
  const ObstacleCrossingStage stage = crossing_runtime_.stage();
  const bool cruise_soft_hold = robot_context_.isTraversing() &&
      (stage == ObstacleCrossingStage::Idle || stage == ObstacleCrossingStage::CompleteWaitObstacleClear);
  if (!cruise_soft_hold && has_last_effective_joint_targets_ && crossing_runtime_.stageUsesLastEffectiveTargets())
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
  if (auto_state_handle_.valid())
  {
    auto_state_handle_.setGravityCompensationMode(static_cast<std::uint8_t>(command.gravity_compensation_mode));
  }
}

void B29SmcAutoController::publishPlannerControlState(const ros::Time& time)
{
  planner_control_state_pub_.publish(planner_control_coordinator_.buildState(time));
}

std::size_t B29SmcAutoController::jointIndex(JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}

PlannerControlCoordinator::Positions B29SmcAutoController::currentPlannerReferencePositions() const
{
  return PlannerControlCoordinator::Positions{{
      joint_state_handles_[jointIndex(JointIndex::LeftFirstLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::LeftSecondLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::RightFirstLeg)].getPosition(),
      joint_state_handles_[jointIndex(JointIndex::RightSecondLeg)].getPosition(),
  }};
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
