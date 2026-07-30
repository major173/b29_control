// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/auto_input_mux.h>

#include <algorithm>
#include <cmath>

namespace b29_smc_auto_controller
{
namespace
{
double clampUnit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}
}  // namespace

AutoInputMux::AutoInputMux(const AutoInputMuxConfig& config) : config_(config)
{
}

void AutoInputMux::setJointState(const sensor_msgs::JointState& joint_state)
{
  joint_state_ = joint_state;
  has_joint_state_ = true;
}

void AutoInputMux::setBaseImu(const sensor_msgs::Imu& base_imu)
{
  base_imu_ = base_imu;
  has_base_imu_ = true;
}

void AutoInputMux::setSensorInput(const AutoSensorInput& sensor_input)
{
  sensor_input_ = sensor_input;
  has_sensor_input_ = true;
}

void AutoInputMux::setControlRequest(const AutoControlRequest& control_request)
{
  control_request_ = control_request;
  has_control_request_ = true;
}

void AutoInputMux::setDebugOverride(const AutoDebugOverride& debug_override)
{
  debug_override_ = debug_override;
  has_debug_override_ = true;
}

void AutoInputMux::setAutoState(const steering_engine_hw::AutoStateData& auto_state)
{
  auto_state_ = auto_state;
  has_auto_state_ = true;
}

AutoInputSnapshot AutoInputMux::buildSnapshot() const
{
  AutoInputSnapshot snapshot;

  if (has_sensor_input_)
  {
    snapshot.lower_alive = sensor_input_.lower_alive;
    snapshot.imu_ready = sensor_input_.imu_ready;
    snapshot.grip_confirmed = sensor_input_.grip_confirmed;
    snapshot.joint_fault = sensor_input_.joint_fault;
    snapshot.grip_fault = sensor_input_.grip_fault;
    snapshot.obstacle_detected = sensor_input_.obstacle_detected;
    snapshot.obstacle_type = toObstacleType(sensor_input_.obstacle_type);
    snapshot.classification_stable = sensor_input_.classification_stable;
    snapshot.range_to_obstacle = sensor_input_.range_to_obstacle;
    snapshot.at_crossing_position = sensor_input_.at_crossing_position;
    snapshot.post_check_passed = sensor_input_.post_check_passed;
    snapshot.post_check_failed = sensor_input_.post_check_failed;
    snapshot.auto_run_pause = false;
    snapshot.stamp = sensor_input_.header.stamp;
  }

  if (has_auto_state_)
  {
    snapshot.lower_alive = auto_state_.lower_alive;
    snapshot.imu_ready = auto_state_.imu_ready;
    snapshot.grip_confirmed = auto_state_.grip_confirmed;
    snapshot.joint_fault = auto_state_.joint_fault;
    snapshot.grip_fault = auto_state_.grip_fault;
    snapshot.obstacle_detected = auto_state_.obstacle_detected;
    snapshot.obstacle_type = toObstacleType(static_cast<uint8_t>(auto_state_.obstacle_type));
    snapshot.classification_stable = auto_state_.classification_stable;
    snapshot.range_to_obstacle = auto_state_.range_to_obstacle;
    snapshot.at_crossing_position = auto_state_.at_crossing_position;
    snapshot.post_check_passed = auto_state_.post_check_passed;
    snapshot.post_check_failed = auto_state_.post_check_failed;
    snapshot.auto_run_pause = false;
    snapshot.stamp = auto_state_.header.stamp;
  }

  if (has_control_request_)
  {
    snapshot.auto_start_requested = control_request_.auto_start_requested;
    snapshot.manual_reset_requested = control_request_.manual_reset_requested;
    snapshot.emergency_stop = control_request_.emergency_stop;
    snapshot.stamp = latestStamp(snapshot.stamp, control_request_.stamp);
  }

  if (has_joint_state_)
  {
    snapshot.stamp = latestStamp(snapshot.stamp, joint_state_.header.stamp);
  }

  const bool base_imu_ready = has_base_imu_ && isBaseImuReady();
  if (has_base_imu_)
  {
    snapshot.posture_ready = isPostureWithinThreshold();
    snapshot.stamp = latestStamp(snapshot.stamp, base_imu_.header.stamp);
  }
  snapshot.imu_ready = (has_auto_state_ || has_sensor_input_)
                           ? snapshot.imu_ready && base_imu_ready
                           : base_imu_ready;

  applyDebugOverride(snapshot);
  return snapshot;
}

bool AutoInputMux::isPostureWithinThreshold() const
{
  if (!has_base_imu_)
  {
    return false;
  }

  const double x = base_imu_.orientation.x;
  const double y = base_imu_.orientation.y;
  const double z = base_imu_.orientation.z;
  const double w = base_imu_.orientation.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = std::asin(clampUnit(sinp));

  return std::abs(roll) <= config_.max_abs_roll_rad && std::abs(pitch) <= config_.max_abs_pitch_rad;
}

bool AutoInputMux::isBaseImuReady() const
{
  bool is_base_imu_ready = true;
  static size_t imu_unchange_count = 0;
  static sensor_msgs::Imu last_imu{};
  constexpr size_t kMaxUnchangedCount = 10;

  if (!has_base_imu_ || base_imu_.header.stamp.isZero())
  {
    is_base_imu_ready = false;
  }

    const auto finite = [](double value) {
    return std::isfinite(value);
  };

  const auto& q = base_imu_.orientation;
  const auto& gyro = base_imu_.angular_velocity;
  const auto& acc = base_imu_.linear_acceleration;

  const bool value_valid =
      finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
      finite(gyro.x) && finite(gyro.y) && finite(gyro.z) &&
      finite(acc.x) && finite(acc.y) && finite(acc.z);

  if (!value_valid)
  {
    return false;
  }

  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(norm2) || norm2 < 1e-12)
  {
    return false;
  }

  if (!isBaseImuChange(base_imu_, last_imu))
  {
    ++imu_unchange_count;
  }
  else
  {
    imu_unchange_count = 0;
  }

  if (imu_unchange_count >= kMaxUnchangedCount)
  {
    is_base_imu_ready = false;
  }

  last_imu = base_imu_;

  return is_base_imu_ready;
}

bool AutoInputMux::isBaseImuChange(const sensor_msgs::Imu& current, 
                                  const sensor_msgs::Imu& previous) const
{
  return current.orientation.x != previous.orientation.x ||
         current.orientation.y != previous.orientation.y ||
         current.orientation.z != previous.orientation.z ||
         current.orientation.w != previous.orientation.w ||
         current.angular_velocity.x != previous.angular_velocity.x ||
         current.angular_velocity.y != previous.angular_velocity.y ||
         current.angular_velocity.z != previous.angular_velocity.z ||
         current.linear_acceleration.x != previous.linear_acceleration.x ||
         current.linear_acceleration.y != previous.linear_acceleration.y ||
         current.linear_acceleration.z != previous.linear_acceleration.z;
}

ObstacleType AutoInputMux::toObstacleType(uint8_t obstacle_type)
{
  switch (obstacle_type)
  {
    case 1u:
      return ObstacleType::LineClamp;
    case 2u:
      return ObstacleType::Damper;
    default:
      return ObstacleType::Unknown;
  }
}

ros::Time AutoInputMux::latestStamp(const ros::Time& lhs, const ros::Time& rhs)
{
  if (lhs.isZero())
  {
    return rhs;
  }
  if (rhs.isZero())
  {
    return lhs;
  }
  return lhs < rhs ? rhs : lhs;
}

void AutoInputMux::applyDebugOverride(AutoInputSnapshot& snapshot) const
{
  if (!has_debug_override_ || !debug_override_.enabled)
  {
    return;
  }

  snapshot.stamp = latestStamp(snapshot.stamp, debug_override_.header.stamp);

  if (debug_override_.field_mask & AutoDebugOverride::FIELD_AUTO_START_REQUESTED)
  {
    snapshot.auto_start_requested = debug_override_.auto_start_requested;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_MANUAL_RESET_REQUESTED)
  {
    snapshot.manual_reset_requested = debug_override_.manual_reset_requested;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_EMERGENCY_STOP)
  {
    snapshot.emergency_stop = debug_override_.emergency_stop;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_LOWER_ALIVE)
  {
    snapshot.lower_alive = debug_override_.lower_alive;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_IMU_READY)
  {
    snapshot.imu_ready = debug_override_.imu_ready;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_POSTURE_READY)
  {
    snapshot.posture_ready = debug_override_.posture_ready;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_GRIP_CONFIRMED)
  {
    snapshot.grip_confirmed = debug_override_.grip_confirmed;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_JOINT_FAULT)
  {
    snapshot.joint_fault = debug_override_.joint_fault;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_GRIP_FAULT)
  {
    snapshot.grip_fault = debug_override_.grip_fault;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_OBSTACLE_DETECTED)
  {
    snapshot.obstacle_detected = debug_override_.obstacle_detected;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_OBSTACLE_TYPE)
  {
    snapshot.obstacle_type = toObstacleType(debug_override_.obstacle_type);
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_CLASSIFICATION_STABLE)
  {
    snapshot.classification_stable = debug_override_.classification_stable;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_RANGE_TO_OBSTACLE)
  {
    snapshot.range_to_obstacle = debug_override_.range_to_obstacle;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_AT_CROSSING_POSITION)
  {
    snapshot.at_crossing_position = debug_override_.at_crossing_position;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_CROSSING_STEP_DONE)
  {
    snapshot.crossing_step_done = debug_override_.crossing_step_done;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_CROSSING_COMPLETE)
  {
    snapshot.crossing_complete = debug_override_.crossing_complete;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_POST_CHECK_PASSED)
  {
    snapshot.post_check_passed = debug_override_.post_check_passed;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_POST_CHECK_FAILED)
  {
    snapshot.post_check_failed = debug_override_.post_check_failed;
  }
  if (debug_override_.field_mask & AutoDebugOverride::FIELD_AUTO_RUN_PAUSE)
  {
    snapshot.auto_run_pause = debug_override_.auto_run_pause;
  }
}
}  // namespace b29_smc_auto_controller
