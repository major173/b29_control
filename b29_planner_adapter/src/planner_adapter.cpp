// SPDX-License-Identifier: BSD-3-Clause

#include <b29_planner_adapter/planner_adapter.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include <boost/bind/bind.hpp>

namespace b29_planner_adapter
{
namespace
{
constexpr double kTimeEpsilon = 1e-9;
const std::array<std::string, kPlannerJointCount> kJointKeys{{
    "left_first", "left_second", "right_first", "right_second"}};
const std::array<std::string, kPlannerJointCount> kFixedOutputOrder{{
    "left_first_leg_joint", "left_second_leg_joint", "right_first_leg_joint", "right_second_leg_joint"}};

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

PlannerAdapter::PlannerAdapter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
  : node_handle_(std::move(node_handle)), private_node_handle_(std::move(private_node_handle))
{
}

bool PlannerAdapter::init()
{
  if (!loadParameters())
  {
    return false;
  }

  trajectory_processor_ = std::make_unique<TrajectoryProcessor>(joint_specs_);
  planner_state_subscriber_ = node_handle_.subscribe(
      config_.planner_state_topic, 1, &PlannerAdapter::plannerStateCallback, this);
  joint_state_subscriber_ = node_handle_.subscribe(
      config_.joint_states_topic, 1, &PlannerAdapter::jointStateCallback, this);
  planner_command_publisher_ = node_handle_.advertise<b29_smc_auto_controller::PlannerJointCommand>(
      config_.planner_command_topic, 1);
  completion_client_ = node_handle_.serviceClient<b29_smc_auto_controller::CompletePlannerControl>(
      config_.completion_service, false);
  emergency_stop_client_ = node_handle_.serviceClient<std_srvs::Trigger>(
      config_.emergency_stop_service, false);
  action_server_ = std::make_unique<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
      node_handle_, config_.action_name,
      boost::bind(&PlannerAdapter::executeTrajectory, this, boost::placeholders::_1), false);
  action_server_->start();

  ROS_INFO_STREAM("B29 Planner adapter ready: action=" << config_.action_name
                  << " interface_mode=" << config_.interface_mode);
  return true;
}

bool PlannerAdapter::loadParameters()
{
  private_node_handle_.param<std::string>("interface_mode", config_.interface_mode, config_.interface_mode);
  private_node_handle_.param<std::string>("action_name", config_.action_name, config_.action_name);
  private_node_handle_.param<std::string>("planner_control_state_topic", config_.planner_state_topic,
                                         config_.planner_state_topic);
  private_node_handle_.param<std::string>("planner_joint_command_topic", config_.planner_command_topic,
                                         config_.planner_command_topic);
  private_node_handle_.param<std::string>("complete_planner_control_service", config_.completion_service,
                                         config_.completion_service);
  private_node_handle_.param<std::string>("software_emergency_stop_service", config_.emergency_stop_service,
                                         config_.emergency_stop_service);
  private_node_handle_.param<std::string>("joint_states_topic", config_.joint_states_topic,
                                         config_.joint_states_topic);
  private_node_handle_.param<double>("publish_rate", config_.publish_rate, config_.publish_rate);
  private_node_handle_.param<double>("time_scale", config_.time_scale, config_.time_scale);
  private_node_handle_.param<double>("max_output_delta", config_.max_output_delta, config_.max_output_delta);
  private_node_handle_.param<double>("expected_smc_max_delta_per_command", config_.expected_smc_max_delta,
                                    config_.expected_smc_max_delta);
  private_node_handle_.param<bool>("require_goal_velocity", config_.require_goal_velocity,
                                  config_.require_goal_velocity);
  private_node_handle_.param<double>("settle_time", config_.settle_time, config_.settle_time);
  private_node_handle_.param<double>("timeouts/joint_state", config_.joint_state_timeout,
                                    config_.joint_state_timeout);
  private_node_handle_.param<double>("timeouts/planner_state", config_.planner_state_timeout,
                                    config_.planner_state_timeout);
  private_node_handle_.param<double>("timeouts/command_ack", config_.command_ack_timeout,
                                    config_.command_ack_timeout);
  private_node_handle_.param<double>("timeouts/action", config_.action_timeout, config_.action_timeout);
  private_node_handle_.param<double>("timeouts/completion_service", config_.completion_service_timeout,
                                    config_.completion_service_timeout);
  private_node_handle_.param<double>("timeouts/completion_transition", config_.completion_transition_timeout,
                                    config_.completion_transition_timeout);
  private_node_handle_.param<double>("completion/retry_interval", config_.completion_retry_interval,
                                    config_.completion_retry_interval);
  private_node_handle_.param<int>("completion/retry_count", config_.completion_retry_count,
                                 config_.completion_retry_count);
  private_node_handle_.param<int>("safety/direction_mismatch_samples", config_.direction_mismatch_samples,
                                 config_.direction_mismatch_samples);
  private_node_handle_.param<double>("timeouts/emergency_stop_service",
                                    config_.emergency_stop_service_timeout,
                                    config_.emergency_stop_service_timeout);

  if (config_.interface_mode != "production" && config_.interface_mode != "debug")
  {
    ROS_ERROR("interface_mode must be either 'production' or 'debug'.");
    return false;
  }
  if (config_.action_name.empty() || config_.planner_state_topic.empty() ||
      config_.planner_command_topic.empty() || config_.completion_service.empty() ||
      config_.emergency_stop_service.empty() || config_.joint_states_topic.empty())
  {
    ROS_ERROR("Action, topic and service names must not be empty.");
    return false;
  }
  if (!finitePositive(config_.publish_rate) || !finitePositive(config_.time_scale) ||
      !finitePositive(config_.max_output_delta) || !finitePositive(config_.expected_smc_max_delta) ||
      !finitePositive(config_.settle_time) || !finitePositive(config_.joint_state_timeout) ||
      !finitePositive(config_.planner_state_timeout) || !finitePositive(config_.command_ack_timeout) ||
      !finitePositive(config_.action_timeout) || !finitePositive(config_.completion_service_timeout) ||
      !finitePositive(config_.completion_retry_interval) ||
      !finitePositive(config_.completion_transition_timeout) || config_.completion_retry_count < 1)
  {
    ROS_ERROR("Planner adapter timing, rate, delta and retry parameters must be finite and positive.");
    return false;
  }
  if (config_.max_output_delta > config_.expected_smc_max_delta + kTimeEpsilon)
  {
    ROS_ERROR("max_output_delta must not exceed expected_smc_max_delta_per_command.");
    return false;
  }
  if (!finitePositive(config_.emergency_stop_service_timeout) ||
      config_.direction_mismatch_samples < 1)
  {
    ROS_ERROR("Emergency-stop timeout and direction mismatch sample count must be positive.");
    return false;
  }
  if (!loadJointVector("tolerances/start", config_.start_tolerance, true) ||
      !loadJointVector("tolerances/path", config_.path_tolerance, true) ||
      !loadJointVector("tolerances/goal_position", config_.goal_position_tolerance, true) ||
      !loadJointVector("tolerances/goal_velocity", config_.goal_velocity_tolerance, true) ||
      !loadJointVector("safety/max_total_displacement", config_.max_total_displacement, false) ||
      !loadJointVector("safety/direction_target_threshold", config_.direction_target_threshold, false) ||
      !loadJointVector("safety/direction_feedback_threshold", config_.direction_feedback_threshold, false))
  {
    return false;
  }
  return loadJointSpecs(joint_specs_);
}

bool PlannerAdapter::loadJointSpecs(std::array<JointSpec, kPlannerJointCount>& specs)
{
  std::vector<std::string> output_order;
  if (!private_node_handle_.getParam("joint_mapping/output_order", output_order) ||
      output_order.size() != kPlannerJointCount ||
      !std::equal(output_order.begin(), output_order.end(), kFixedOutputOrder.begin()))
  {
    ROS_ERROR("joint_mapping/output_order must match the fixed PlannerJointCommand order.");
    return false;
  }

  std::set<std::string> all_aliases;
  for (std::size_t index = 0; index < kPlannerJointCount; ++index)
  {
    JointSpec spec;
    const std::string prefix = "joint_mapping/" + kJointKeys[index];
    private_node_handle_.param<std::string>(prefix + "/output_name", spec.output_name,
                                           kFixedOutputOrder[index]);
    if (spec.output_name != kFixedOutputOrder[index])
    {
      ROS_ERROR_STREAM(prefix << "/output_name must be " << kFixedOutputOrder[index]);
      return false;
    }
    if (!private_node_handle_.getParam(prefix + "/accepted_input_names", spec.accepted_input_names) ||
        spec.accepted_input_names.empty())
    {
      ROS_ERROR_STREAM(prefix << "/accepted_input_names must contain at least one name");
      return false;
    }
    private_node_handle_.param<bool>(prefix + "/continuous", spec.continuous, false);
    private_node_handle_.param<double>(prefix + "/min_position", spec.min_position, -12.566370614359172);
    private_node_handle_.param<double>(prefix + "/max_position", spec.max_position, 12.566370614359172);
    if (!spec.continuous && (!std::isfinite(spec.min_position) || !std::isfinite(spec.max_position) ||
                             spec.min_position >= spec.max_position))
    {
      ROS_ERROR_STREAM(prefix << " position range is invalid");
      return false;
    }
    for (const std::string& alias : spec.accepted_input_names)
    {
      if (alias.empty() || !all_aliases.insert(alias).second)
      {
        ROS_ERROR_STREAM("Joint aliases must be non-empty and unique: " << alias);
        return false;
      }
    }
    specs[index] = spec;
  }
  return true;
}

bool PlannerAdapter::loadJointVector(const std::string& name, JointVector& values, bool allow_zero) const
{
  std::vector<double> parameter;
  if (private_node_handle_.getParam(name, parameter))
  {
    if (parameter.size() != kPlannerJointCount)
    {
      ROS_ERROR_STREAM(name << " must contain exactly four values");
      return false;
    }
    std::copy(parameter.begin(), parameter.end(), values.begin());
  }
  for (double value : values)
  {
    if (!std::isfinite(value) || (allow_zero ? value < 0.0 : value <= 0.0))
    {
      ROS_ERROR_STREAM(name << " values must be finite and " << (allow_zero ? "non-negative" : "positive"));
      return false;
    }
  }
  return true;
}

void PlannerAdapter::plannerStateCallback(
    const b29_smc_auto_controller::PlannerControlState::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(planner_state_mutex_);
  planner_state_snapshot_.state = *msg;
  planner_state_snapshot_.received_time = ros::WallTime::now();
  planner_state_snapshot_.available = true;
}

void PlannerAdapter::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  JointStateSnapshot snapshot;
  snapshot.has_velocities = true;
  for (std::size_t output_index = 0; output_index < kPlannerJointCount; ++output_index)
  {
    const std::string& output_name = joint_specs_[output_index].output_name;
    const auto name_iterator = std::find(msg->name.begin(), msg->name.end(), output_name);
    if (name_iterator == msg->name.end())
    {
      return;
    }
    const std::size_t message_index = static_cast<std::size_t>(std::distance(msg->name.begin(), name_iterator));
    if (message_index >= msg->position.size() || !std::isfinite(msg->position[message_index]))
    {
      return;
    }
    snapshot.positions[output_index] = msg->position[message_index];
    if (message_index >= msg->velocity.size() || !std::isfinite(msg->velocity[message_index]))
    {
      snapshot.has_velocities = false;
    }
    else
    {
      snapshot.velocities[output_index] = msg->velocity[message_index];
    }
  }
  snapshot.received_time = ros::WallTime::now();
  snapshot.available = true;
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  joint_state_snapshot_ = snapshot;
}

void PlannerAdapter::executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  if (!goal)
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, "received a null trajectory goal");
    return;
  }

  b29_smc_auto_controller::PlannerControlState planner_state;
  std::string error;
  if (!getFreshPlannerState(planner_state, error) || !planner_state.active || !planner_state.accepting_commands)
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                error.empty() ? "PlannerControl is not accepting commands" : error);
    return;
  }
  if (planner_state.has_accepted_command)
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                "active PlannerControl session already contains accepted commands");
    return;
  }
  if (!finitePositive(planner_state.max_delta_per_command) ||
      config_.max_output_delta > planner_state.max_delta_per_command + kTimeEpsilon)
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                "adapter max_output_delta exceeds the active SMC session limit");
    return;
  }

  JointStateSnapshot initial_joint_state;
  if (!getFreshJointState(initial_joint_state, error))
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, error);
    return;
  }
  const uint32_t session_id = planner_state.session_id;
  if (!waitForFirstCommandReference(session_id, initial_joint_state, planner_state, error))
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, error);
    return;
  }

  std::array<std::size_t, kPlannerJointCount> goal_index_for_output{};
  if (!buildGoalIndexMap(goal->trajectory.joint_names, goal_index_for_output, error))
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS, error);
    return;
  }

  NormalizedTrajectory trajectory;
  if (!trajectory_processor_->normalize(goal->trajectory, trajectory, error))
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, error);
    return;
  }
  trajectory_processor_->alignContinuousJoints(initial_joint_state.positions, trajectory);
  if (trajectory.points.front().time_from_start <= kTimeEpsilon)
  {
    bool stale_start = false;
    for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
    {
      if (std::abs(trajectory_processor_->jointError(
              joint_index, trajectory.points.front().positions[joint_index],
              initial_joint_state.positions[joint_index])) > config_.start_tolerance[joint_index])
      {
        stale_start = true;
      }
    }
    if (stale_start)
    {
      abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                  "trajectory start point exceeds configured start tolerance");
      return;
    }
  }
  trajectory_processor_->anchorStartToReference(initial_joint_state.positions, trajectory);

  if (!trajectory_processor_->validateTotalDisplacement(
          initial_joint_state.positions, trajectory, config_.max_total_displacement, error))
  {
    // Nothing has been published yet.  Rejecting here must not latch the
    // machine-wide software emergency stop.
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, error);
    return;
  }

  const double effective_max_delta = std::min(config_.max_output_delta, planner_state.max_delta_per_command);
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    if (std::abs(trajectory.points.front().positions[joint_index] -
                 initial_joint_state.positions[joint_index]) > effective_max_delta)
    {
      abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                  "first output command exceeds the active SMC delta limit");
      return;
    }
  }

  std::vector<JointVector> samples;
  double applied_time_scale = config_.time_scale;
  if (!trajectory_processor_->buildSamples(trajectory, config_.publish_rate, config_.time_scale,
                                           effective_max_delta, samples, applied_time_scale, error))
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL, error);
    return;
  }
  const double scaled_duration = trajectory.points.back().time_from_start * applied_time_scale;
  if (scaled_duration + config_.settle_time + config_.completion_transition_timeout > config_.action_timeout)
  {
    abortAction(control_msgs::FollowJointTrajectoryResult::INVALID_GOAL,
                "scaled trajectory cannot complete within the configured action timeout");
    return;
  }

  const ros::WallTime action_deadline = ros::WallTime::now() + ros::WallDuration(config_.action_timeout);
  DirectionSafetyState direction_state;
  direction_state.previous_actual = initial_joint_state.positions;
  direction_state.initialized = true;
  uint32_t rejection_count = planner_state.rejection_count;
  uint32_t sequence = 0;
  for (const JointVector& positions : samples)
  {
    ++sequence;
    if (!sendCommandAndAwait(session_id, sequence, positions, rejection_count,
                             action_deadline, error))
    {
      if (action_server_->isActive())
      {
        if (sequence == 1 && firstCommandWasExplicitlyRejected(session_id, sequence))
        {
          abortAction(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED,
                      error + "; no Planner command was accepted, software emergency stop was not latched");
        }
        else
        {
          emergencyAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
        }
      }
      return;
    }

    JointStateSnapshot actual;
    if (!getFreshJointState(actual, error))
    {
      emergencyAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return;
    }
    if (!validatePathTolerance(positions, actual, error))
    {
      recoverableAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return;
    }
    if (!validateMotionDirection(positions, actual, direction_state, error))
    {
      emergencyAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return;
    }
    publishFeedback(goal->trajectory.joint_names, goal_index_for_output, positions, actual);
  }

  const JointVector& final_positions = samples.back();
  if (!waitForFinalSettle(session_id, sequence, final_positions, goal->trajectory.joint_names,
                          goal_index_for_output, action_deadline, direction_state, error))
  {
    if (action_server_->isActive())
    {
      recoverableAbort(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED, error);
    }
    return;
  }
  if (!requestCompletion(session_id, sequence, error) ||
      !waitForCompletionState(session_id, action_deadline, error))
  {
    if (action_server_->isActive())
    {
      recoverableAbort(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED, error);
    }
    return;
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  result.error_string = "trajectory completed and SMC confirmed normal PlannerControl exit";
  action_server_->setSucceeded(result, result.error_string);
}

bool PlannerAdapter::getFreshPlannerState(b29_smc_auto_controller::PlannerControlState& state,
                                          std::string& error) const
{
  std::lock_guard<std::mutex> lock(planner_state_mutex_);
  if (!planner_state_snapshot_.available)
  {
    error = "PlannerControlState has not been received";
    return false;
  }
  if ((ros::WallTime::now() - planner_state_snapshot_.received_time).toSec() > config_.planner_state_timeout)
  {
    error = "PlannerControlState timed out";
    return false;
  }
  state = planner_state_snapshot_.state;
  return true;
}

bool PlannerAdapter::getFreshJointState(JointStateSnapshot& state, std::string& error) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  if (!joint_state_snapshot_.available)
  {
    error = "joint_states does not contain all four B29 leg joints";
    return false;
  }
  if ((ros::WallTime::now() - joint_state_snapshot_.received_time).toSec() > config_.joint_state_timeout)
  {
    error = "joint_states feedback timed out";
    return false;
  }
  state = joint_state_snapshot_;
  return true;
}

bool PlannerAdapter::waitForFirstCommandReference(
    uint32_t session_id, const JointStateSnapshot& initial_joint_state,
    b29_smc_auto_controller::PlannerControlState& state, std::string& error) const
{
  const double wait_seconds = std::max(config_.planner_state_timeout, 2.0 / config_.publish_rate);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(wait_seconds);
  ros::WallRate rate(config_.publish_rate);
  double largest_delta = std::numeric_limits<double>::infinity();

  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    if (!getFreshPlannerState(state, error))
    {
      return false;
    }
    if (state.session_id != session_id || !state.active || !state.accepting_commands ||
        state.has_accepted_command)
    {
      error = "PlannerControl session changed while aligning the first-command reference";
      return false;
    }
    if (!finitePositive(state.max_delta_per_command))
    {
      error = "PlannerControl published an invalid first-command delta limit";
      return false;
    }

    bool reference_valid = !state.reference_stamp.isZero();
    largest_delta = 0.0;
    for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
    {
      const double reference = state.reference_positions[joint_index];
      reference_valid = reference_valid && std::isfinite(reference);
      if (reference_valid)
      {
        largest_delta = std::max(
            largest_delta,
            std::abs(initial_joint_state.positions[joint_index] - reference));
      }
    }
    if (reference_valid && largest_delta <= state.max_delta_per_command + kTimeEpsilon)
    {
      return true;
    }
    rate.sleep();
  }

  std::ostringstream stream;
  stream << "live trajectory start did not align with the SMC first-command reference before timeout"
         << "; largest_delta=" << largest_delta
         << " limit=" << state.max_delta_per_command;
  error = stream.str();
  return false;
}

bool PlannerAdapter::firstCommandWasExplicitlyRejected(uint32_t session_id,
                                                       uint32_t sequence) const
{
  b29_smc_auto_controller::PlannerControlState state;
  std::string ignored_error;
  return getFreshPlannerState(state, ignored_error) && state.session_id == session_id &&
         state.active && state.accepting_commands && !state.has_accepted_command &&
         state.last_rejected_sequence == sequence &&
         state.reject_reason != b29_smc_auto_controller::PlannerControlState::REJECT_NONE;
}

bool PlannerAdapter::buildGoalIndexMap(
    const std::vector<std::string>& goal_names,
    std::array<std::size_t, kPlannerJointCount>& goal_index_for_output, std::string& error) const
{
  if (goal_names.size() != kPlannerJointCount ||
      std::set<std::string>(goal_names.begin(), goal_names.end()).size() != kPlannerJointCount)
  {
    error = "trajectory must contain exactly four unique joint names";
    return false;
  }
  std::set<std::size_t> matched_indices;
  for (std::size_t output_index = 0; output_index < kPlannerJointCount; ++output_index)
  {
    bool found = false;
    for (std::size_t goal_index = 0; goal_index < goal_names.size(); ++goal_index)
    {
      const std::vector<std::string>& aliases = joint_specs_[output_index].accepted_input_names;
      if (std::find(aliases.begin(), aliases.end(), goal_names[goal_index]) == aliases.end())
      {
        continue;
      }
      if (found || !matched_indices.insert(goal_index).second)
      {
        error = "trajectory joint names are ambiguous";
        return false;
      }
      goal_index_for_output[output_index] = goal_index;
      found = true;
    }
    if (!found)
    {
      error = "trajectory is missing required joint " + joint_specs_[output_index].output_name;
      return false;
    }
  }
  return true;
}

bool PlannerAdapter::sendCommandAndAwait(uint32_t session_id, uint32_t sequence,
                                         const JointVector& positions,
                                         uint32_t& rejection_count,
                                         const ros::WallTime& action_deadline, std::string& error)
{
  const ros::WallTime ack_deadline = ros::WallTime::now() + ros::WallDuration(config_.command_ack_timeout);
  ros::WallRate rate(config_.publish_rate);
  while (ros::ok())
  {
    if (preemptRequested(error))
    {
      return false;
    }
    if (ros::WallTime::now() >= action_deadline)
    {
      error = "action timeout while waiting for command acknowledgement";
      return false;
    }

    b29_smc_auto_controller::PlannerControlState state;
    if (!validateSession(session_id, state, error))
    {
      return false;
    }
    if (state.has_accepted_command && state.last_accepted_sequence == sequence)
    {
      rememberAcceptedCommand(session_id, sequence, positions);
      return true;
    }
    if (state.has_accepted_command && state.last_accepted_sequence > sequence)
    {
      error = "SMC acknowledged a sequence newer than the command being sent";
      return false;
    }
    if (state.rejection_count != rejection_count &&
        state.last_rejected_sequence == sequence &&
        state.reject_reason != b29_smc_auto_controller::PlannerControlState::REJECT_NONE)
    {
      rejection_count = state.rejection_count;
      error = "SMC rejected Planner command with reason code " + std::to_string(state.reject_reason);
      return false;
    }
    if (ros::WallTime::now() >= ack_deadline)
    {
      error = "SMC command acknowledgement timed out";
      return false;
    }

    b29_smc_auto_controller::PlannerJointCommand command;
    command.header.stamp = ros::Time::now();
    command.session_id = session_id;
    command.sequence = sequence;
    std::copy(positions.begin(), positions.end(), command.positions.begin());
    planner_command_publisher_.publish(command);
    rate.sleep();
  }
  error = "ROS shutdown while waiting for command acknowledgement";
  return false;
}

bool PlannerAdapter::validateSession(uint32_t session_id,
                                     b29_smc_auto_controller::PlannerControlState& state,
                                     std::string& error) const
{
  if (!getFreshPlannerState(state, error))
  {
    return false;
  }
  if (state.session_id != session_id)
  {
    error = "PlannerControl session changed during trajectory execution";
    return false;
  }
  if (!state.active || !state.accepting_commands)
  {
    error = "PlannerControl authority was revoked during trajectory execution";
    return false;
  }
  return true;
}

bool PlannerAdapter::validatePathTolerance(const JointVector& desired,
                                           const JointStateSnapshot& actual, std::string& error) const
{
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    if (config_.path_tolerance[joint_index] > 0.0 &&
        std::abs(trajectory_processor_->jointError(joint_index, desired[joint_index],
                                                   actual.positions[joint_index])) >
            config_.path_tolerance[joint_index])
    {
      error = "joint feedback exceeded configured path tolerance for " +
              joint_specs_[joint_index].output_name;
      return false;
    }
  }
  return true;
}

bool PlannerAdapter::validateMotionDirection(
    const JointVector& desired, const JointStateSnapshot& actual,
    DirectionSafetyState& state, std::string& error) const
{
  if (!state.initialized)
  {
    state.previous_actual = actual.positions;
    state.initialized = true;
    return true;
  }

  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    const bool opposite = trajectory_processor_->isOppositeMotionEvidence(
        joint_index, desired[joint_index], state.previous_actual[joint_index],
        actual.positions[joint_index], config_.direction_target_threshold[joint_index],
        config_.direction_feedback_threshold[joint_index]);
    const double feedback_step = trajectory_processor_->jointError(
        joint_index, actual.positions[joint_index], state.previous_actual[joint_index]);
    if (opposite)
    {
      ++state.mismatch_counts[joint_index];
    }
    else if (std::abs(feedback_step) >= config_.direction_feedback_threshold[joint_index])
    {
      // A measurable step that is not away from the target breaks the
      // consecutive-divergence streak.  Stationary/noisy samples do not.
      state.mismatch_counts[joint_index] = 0;
    }

    if (state.mismatch_counts[joint_index] >= config_.direction_mismatch_samples)
    {
      std::ostringstream stream;
      stream << "joint feedback moved away from the commanded target for "
             << joint_specs_[joint_index].output_name
             << ": desired=" << desired[joint_index]
             << " previous_actual=" << state.previous_actual[joint_index]
             << " actual=" << actual.positions[joint_index]
             << " target_error="
             << trajectory_processor_->jointError(joint_index, desired[joint_index],
                                                   state.previous_actual[joint_index])
             << " feedback_step=" << feedback_step
             << " consecutive_samples=" << state.mismatch_counts[joint_index];
      error = stream.str();
      state.previous_actual = actual.positions;
      return false;
    }
  }
  state.previous_actual = actual.positions;
  return true;
}

bool PlannerAdapter::goalWithinTolerance(const JointVector& desired,
                                         const JointStateSnapshot& actual) const
{
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    if (std::abs(trajectory_processor_->jointError(joint_index, desired[joint_index],
                                                   actual.positions[joint_index])) >
        config_.goal_position_tolerance[joint_index])
    {
      return false;
    }
    if (config_.require_goal_velocity &&
        (!actual.has_velocities ||
         std::abs(actual.velocities[joint_index]) > config_.goal_velocity_tolerance[joint_index]))
    {
      return false;
    }
  }
  return true;
}

void PlannerAdapter::publishFeedback(
    const std::vector<std::string>& goal_joint_names,
    const std::array<std::size_t, kPlannerJointCount>& goal_index_for_output,
    const JointVector& desired, const JointStateSnapshot& actual)
{
  control_msgs::FollowJointTrajectoryFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.joint_names = goal_joint_names;
  feedback.desired.positions.resize(kPlannerJointCount);
  feedback.actual.positions.resize(kPlannerJointCount);
  feedback.error.positions.resize(kPlannerJointCount);
  for (std::size_t output_index = 0; output_index < kPlannerJointCount; ++output_index)
  {
    const std::size_t goal_index = goal_index_for_output[output_index];
    feedback.desired.positions[goal_index] = desired[output_index];
    feedback.actual.positions[goal_index] = actual.positions[output_index];
    feedback.error.positions[goal_index] = trajectory_processor_->jointError(
        output_index, desired[output_index], actual.positions[output_index]);
  }
  action_server_->publishFeedback(feedback);
}

bool PlannerAdapter::waitForFinalSettle(
    uint32_t session_id, uint32_t final_sequence, const JointVector& final_positions,
    const std::vector<std::string>& goal_joint_names,
    const std::array<std::size_t, kPlannerJointCount>& goal_index_for_output,
    const ros::WallTime& action_deadline, DirectionSafetyState& direction_state,
    std::string& error)
{
  ros::WallTime settle_start;
  ros::WallRate rate(config_.publish_rate);
  while (ros::ok() && ros::WallTime::now() < action_deadline)
  {
    if (preemptRequested(error))
    {
      return false;
    }
    b29_smc_auto_controller::PlannerControlState state;
    if (!validateSession(session_id, state, error))
    {
      return false;
    }

    b29_smc_auto_controller::PlannerJointCommand command;
    command.header.stamp = ros::Time::now();
    command.session_id = session_id;
    command.sequence = final_sequence;
    std::copy(final_positions.begin(), final_positions.end(), command.positions.begin());
    planner_command_publisher_.publish(command);

    JointStateSnapshot actual;
    if (!getFreshJointState(actual, error))
    {
      emergencyAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return false;
    }
    if (!validatePathTolerance(final_positions, actual, error))
    {
      recoverableAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return false;
    }
    if (!validateMotionDirection(final_positions, actual, direction_state, error))
    {
      emergencyAbort(control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED, error);
      return false;
    }
    if (config_.require_goal_velocity && !actual.has_velocities)
    {
      error = "joint_states velocities are required for final settling but are unavailable";
      recoverableAbort(control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED, error);
      return false;
    }
    publishFeedback(goal_joint_names, goal_index_for_output, final_positions, actual);
    if (goalWithinTolerance(final_positions, actual))
    {
      if (settle_start.isZero())
      {
        settle_start = ros::WallTime::now();
      }
      if ((ros::WallTime::now() - settle_start).toSec() >= config_.settle_time)
      {
        return true;
      }
    }
    else
    {
      settle_start = ros::WallTime{};
    }
    rate.sleep();
  }
  error = "final joint tolerance did not remain satisfied before the action timeout";
  return false;
}

bool PlannerAdapter::requestCompletion(uint32_t session_id, uint32_t final_sequence,
                                       std::string& error)
{
  for (int attempt = 0; attempt < config_.completion_retry_count; ++attempt)
  {
    if (preemptRequested(error))
    {
      return false;
    }
    const ros::WallTime existence_deadline =
        ros::WallTime::now() + ros::WallDuration(config_.completion_service_timeout);
    while (ros::ok() && !completion_client_.exists() && ros::WallTime::now() < existence_deadline)
    {
      ros::WallDuration(0.02).sleep();
    }
    if (completion_client_.exists())
    {
      b29_smc_auto_controller::CompletePlannerControl service;
      service.request.session_id = session_id;
      service.request.final_sequence = final_sequence;
      if (completion_client_.call(service) && service.response.accepted)
      {
        return true;
      }
      error = service.response.message.empty() ? "CompletePlannerControl service call failed" :
                                                service.response.message;
    }
    else
    {
      error = "CompletePlannerControl service is unavailable";
    }
    if (attempt + 1 < config_.completion_retry_count)
    {
      ros::WallDuration(config_.completion_retry_interval).sleep();
    }
  }
  return false;
}

bool PlannerAdapter::waitForCompletionState(uint32_t session_id,
                                            const ros::WallTime& action_deadline,
                                            std::string& error)
{
  const ros::WallTime transition_deadline = std::min(
      action_deadline, ros::WallTime::now() + ros::WallDuration(config_.completion_transition_timeout));
  ros::WallRate rate(config_.publish_rate);
  while (ros::ok() && ros::WallTime::now() < transition_deadline)
  {
    if (preemptRequested(error))
    {
      return false;
    }
    b29_smc_auto_controller::PlannerControlState state;
    if (!getFreshPlannerState(state, error))
    {
      return false;
    }
    if (state.last_completed_session_id == session_id &&
        state.exit_reason == b29_smc_auto_controller::PlannerControlState::EXIT_COMPLETED)
    {
      return true;
    }
    if (state.session_id != session_id || !state.active)
    {
      error = "SMC exited PlannerControl without confirming normal completion";
      return false;
    }
    rate.sleep();
  }
  error = "timed out waiting for SMC to publish the normal PlannerControl exit";
  return false;
}

bool PlannerAdapter::preemptRequested(std::string& error)
{
  if (action_server_->isPreemptRequested())
  {
    std::string hold_detail;
    const bool hold_sent = publishCurrentPositionHold(hold_detail);
    std::string stop_detail;
    const bool stop_latched = hold_sent ? false : triggerEmergencyStop(stop_detail);
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    if (hold_sent)
    {
      result.error_string = "trajectory execution preempted; current-position hold accepted: " + hold_detail;
    }
    else if (stop_latched)
    {
      result.error_string = "trajectory execution preempted; hold failed, software emergency stop latched: " +
                            stop_detail;
    }
    else
    {
      result.error_string = "trajectory execution preempted; CRITICAL: hold and software emergency stop failed: " +
                            hold_detail + "; " + stop_detail;
    }
    action_server_->setPreempted(result, result.error_string);
    error = result.error_string;
    return true;
  }
  if (!ros::ok())
  {
    error = "ROS shutdown during trajectory execution";
    return true;
  }
  return false;
}

bool PlannerAdapter::publishCurrentPositionHold(std::string& detail)
{
  b29_smc_auto_controller::PlannerControlState state;
  JointStateSnapshot actual;
  if (!getFreshPlannerState(state, detail) || !getFreshJointState(actual, detail))
  {
    return false;
  }
  if (!state.active || !state.accepting_commands || !state.has_accepted_command)
  {
    detail = "PlannerControl cannot accept a current-position hold";
    return false;
  }

  if (!last_accepted_positions_available_ ||
      last_accepted_session_id_ != state.session_id ||
      last_accepted_sequence_ != state.last_accepted_sequence)
  {
    detail = "adapter does not have the last SMC-accepted position needed for a bounded hold";
    return false;
  }

  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(config_.command_ack_timeout);
  JointVector bounded = last_accepted_positions_;
  uint32_t hold_sequence = state.last_accepted_sequence;
  std::size_t step_count = 0;
  while (ros::ok())
  {
    bool reached = true;
    JointVector next = bounded;
    for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
    {
      const double error = trajectory_processor_->jointError(
          joint_index, actual.positions[joint_index], bounded[joint_index]);
      if (std::abs(error) > config_.max_output_delta)
      {
        next[joint_index] += std::copysign(config_.max_output_delta, error);
        reached = false;
      }
      else
      {
        next[joint_index] += error;
      }
    }

    // Even when all axes are already within one step, publish the exact
    // encoder snapshot once.  The first iteration therefore always emits a
    // hold command instead of silently claiming success.
    ++hold_sequence;
    ++step_count;
    if (!publishHoldStepAndAwait(state.session_id, hold_sequence, next, deadline, detail))
    {
      return false;
    }
    bounded = next;
    if (reached)
    {
      detail = "SMC accepted bounded current-position hold through sequence " +
               std::to_string(hold_sequence) + " in " + std::to_string(step_count) + " step(s)";
      return true;
    }
    if (ros::WallTime::now() >= deadline)
    {
      detail = "bounded current-position hold exceeded its acknowledgement deadline";
      return false;
    }
  }
  detail = "ROS shutdown while applying bounded current-position hold";
  return false;
}

bool PlannerAdapter::publishHoldStepAndAwait(uint32_t session_id, uint32_t sequence,
                                             const JointVector& positions,
                                             const ros::WallTime& deadline,
                                             std::string& detail)
{
  ros::WallRate rate(config_.publish_rate);
  while (ros::ok() && ros::WallTime::now() < deadline)
  {
    b29_smc_auto_controller::PlannerJointCommand command;
    command.header.stamp = ros::Time::now();
    command.session_id = session_id;
    command.sequence = sequence;
    std::copy(positions.begin(), positions.end(), command.positions.begin());
    planner_command_publisher_.publish(command);
    rate.sleep();

    b29_smc_auto_controller::PlannerControlState latest;
    if (!getFreshPlannerState(latest, detail))
    {
      return false;
    }
    if (latest.session_id != session_id || !latest.active || !latest.accepting_commands)
    {
      detail = "PlannerControl exited while applying bounded current-position hold";
      return false;
    }
    if (latest.has_accepted_command && latest.last_accepted_sequence == sequence)
    {
      rememberAcceptedCommand(session_id, sequence, positions);
      return true;
    }
    if (latest.last_rejected_sequence == sequence &&
        latest.reject_reason != b29_smc_auto_controller::PlannerControlState::REJECT_NONE)
    {
      detail = "SMC rejected bounded current-position hold sequence " +
               std::to_string(sequence) + " with reason code " +
               std::to_string(latest.reject_reason);
      return false;
    }
  }
  detail = "SMC did not acknowledge bounded current-position hold sequence " +
           std::to_string(sequence);
  return false;
}

void PlannerAdapter::rememberAcceptedCommand(uint32_t session_id, uint32_t sequence,
                                             const JointVector& positions)
{
  last_accepted_session_id_ = session_id;
  last_accepted_sequence_ = sequence;
  last_accepted_positions_ = positions;
  last_accepted_positions_available_ = true;
}

bool PlannerAdapter::triggerEmergencyStop(std::string& detail)
{
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(config_.emergency_stop_service_timeout);
  while (ros::ok() && !emergency_stop_client_.exists() && ros::WallTime::now() < deadline)
  {
    ros::WallDuration(0.01).sleep();
  }
  if (!emergency_stop_client_.exists())
  {
    detail = "software emergency-stop service is unavailable";
    ROS_FATAL_STREAM(detail);
    return false;
  }

  std_srvs::Trigger service;
  if (!emergency_stop_client_.call(service))
  {
    detail = "software emergency-stop service call failed";
    ROS_FATAL_STREAM(detail);
    return false;
  }
  detail = service.response.message;
  if (!service.response.success)
  {
    ROS_FATAL_STREAM("software emergency stop was rejected: " << detail);
    return false;
  }
  ROS_ERROR_STREAM("software emergency stop latched by Planner adapter: " << detail);
  return true;
}

void PlannerAdapter::emergencyAbort(int32_t error_code, const std::string& message)
{
  std::string stop_detail;
  const bool stop_latched = triggerEmergencyStop(stop_detail);
  const std::string combined = message +
      (stop_latched ? "; software emergency stop latched: " :
                      "; CRITICAL: software emergency stop failed: ") +
      stop_detail;
  abortAction(error_code, combined);
}

void PlannerAdapter::recoverableAbort(int32_t error_code, const std::string& message)
{
  std::string hold_detail;
  if (publishCurrentPositionHold(hold_detail))
  {
    abortAction(error_code, message + "; current-position hold accepted: " + hold_detail);
    return;
  }
  emergencyAbort(error_code, message + "; current-position hold failed: " + hold_detail);
}

void PlannerAdapter::abortAction(int32_t error_code, const std::string& message)
{
  if (!action_server_ || !action_server_->isActive())
  {
    return;
  }
  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = error_code;
  result.error_string = message;
  action_server_->setAborted(result, message);
}
}  // namespace b29_planner_adapter
