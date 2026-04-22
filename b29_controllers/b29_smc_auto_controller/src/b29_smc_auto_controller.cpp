#include "b29_smc_auto_controller/b29_smc_auto_controller.h"

#include <b29_smc_auto_controller/output_mode.h>
#include <pluginlib/class_list_macros.hpp>

namespace b29_smc_auto_controller
{
bool B29SmcAutoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  ros::NodeHandle root_nh;
  return init(robot_hw, root_nh, controller_nh);
}

bool B29SmcAutoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& /*root_nh*/,
                                ros::NodeHandle& controller_nh)
{
  if (!initInterfaces(robot_hw))
  {
    return false;
  }

  if (!loadParameters(controller_nh))
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
  state_trace_pub_ = controller_nh.advertise<AutoStateTrace>("state_trace", 10);
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
  sensor_input_.header.stamp = time;
  input_mux_.setSensorInput(sensor_input_);
  input_mux_.setDebugOverride(debug_override_);
  command_dispatcher_.dispatch(robot_context_.currentCommand());
  state_trace_pub_.publish(robot_context_.buildTraceMessage(time));
}

void B29SmcAutoController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  if (!initialized_)
  {
    return;
  }

  sensor_input_.header.stamp = time;
  input_mux_.setSensorInput(sensor_input_);
  input_mux_.setDebugOverride(debug_override_);
  input_mux_.setJointState(buildJointStateMessage(time));
  input_mux_.setBaseImu(buildBaseImuMessage(time));

  robot_context_.setInputSnapshot(input_mux_.buildSnapshot());
  robot_context_.tick50Hz();
  command_dispatcher_.dispatch(robot_context_.currentCommand());
  state_trace_pub_.publish(robot_context_.buildTraceMessage(time));
}

void B29SmcAutoController::stopping(const ros::Time& time)
{
  if (!initialized_)
  {
    return;
  }

  AutoControlCommand safe_stop;
  command_dispatcher_.dispatch(safe_stop);
  AutoStateTrace trace = robot_context_.buildTraceMessage(time);
  applyCommandToTrace(safe_stop, trace);
  state_trace_pub_.publish(trace);
}

bool B29SmcAutoController::initInterfaces(hardware_interface::RobotHW* robot_hw)
{
  joint_state_interface_ = robot_hw->get<hardware_interface::JointStateInterface>();
  position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
  velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
  imu_sensor_interface_ = robot_hw->get<hardware_interface::ImuSensorInterface>();

  return joint_state_interface_ && position_joint_interface_ && velocity_joint_interface_ && imu_sensor_interface_;
}

bool B29SmcAutoController::loadParameters(ros::NodeHandle& controller_nh)
{
  controller_nh.param<std::string>("joint_names/left_first_leg_joint", position_joint_names_[0], position_joint_names_[0]);
  controller_nh.param<std::string>("joint_names/left_second_leg_joint", position_joint_names_[1], position_joint_names_[1]);
  controller_nh.param<std::string>("joint_names/left_rod_joint", position_joint_names_[2], position_joint_names_[2]);
  controller_nh.param<std::string>("joint_names/right_first_leg_joint", position_joint_names_[3], position_joint_names_[3]);
  controller_nh.param<std::string>("joint_names/right_second_leg_joint", position_joint_names_[4], position_joint_names_[4]);
  controller_nh.param<std::string>("joint_names/right_rod_joint", position_joint_names_[5], position_joint_names_[5]);
  controller_nh.param<std::string>("joint_names/left_friction_wheel_joint", wheel_joint_names_[0], wheel_joint_names_[0]);
  controller_nh.param<std::string>("joint_names/right_friction_wheel_joint", wheel_joint_names_[1], wheel_joint_names_[1]);
  controller_nh.param<std::string>("imu_names", base_imu_name_, base_imu_name_);
  std::string output_mode_name = toString(output_mode_);
  controller_nh.param<std::string>("output_mode", output_mode_name, output_mode_name);

  try
  {
    output_mode_ = parseOutputMode(output_mode_name);
  }
  catch (const std::invalid_argument&)
  {
    ROS_ERROR_STREAM("Unsupported output_mode: " << output_mode_name);
    return false;
  }

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
}

void B29SmcAutoController::sensorInputCallback(const AutoSensorInput::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  sensor_input_ = *msg;
}

void B29SmcAutoController::debugOverrideCallback(const AutoDebugOverride::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  debug_override_ = *msg;
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
