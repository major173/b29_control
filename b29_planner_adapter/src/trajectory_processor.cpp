// SPDX-License-Identifier: BSD-3-Clause

#include <b29_planner_adapter/trajectory_processor.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace b29_planner_adapter
{
namespace
{
constexpr double kTimeEpsilon = 1e-9;
constexpr std::size_t kMaximumGeneratedSamples = 1000000;

bool finiteVector(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
}  // namespace

TrajectoryProcessor::TrajectoryProcessor(std::array<JointSpec, kPlannerJointCount> joint_specs)
  : joint_specs_(std::move(joint_specs))
{
}

bool TrajectoryProcessor::normalize(const trajectory_msgs::JointTrajectory& input,
                                    NormalizedTrajectory& output, std::string& error) const
{
  output = NormalizedTrajectory{};
  if (input.points.empty())
  {
    error = "trajectory contains no points";
    return false;
  }

  std::array<std::size_t, kPlannerJointCount> input_indices{};
  if (!inputIndexMap(input.joint_names, input_indices, error))
  {
    return false;
  }

  const bool has_velocities = !input.points.front().velocities.empty();
  const bool has_accelerations = !input.points.front().accelerations.empty();
  if (has_accelerations && !has_velocities)
  {
    error = "trajectory accelerations require velocities";
    return false;
  }
  output.interpolation = has_accelerations ? InterpolationMode::Quintic :
                         has_velocities ? InterpolationMode::Cubic : InterpolationMode::Linear;

  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t point_index = 0; point_index < input.points.size(); ++point_index)
  {
    const trajectory_msgs::JointTrajectoryPoint& input_point = input.points[point_index];
    if (input_point.positions.size() != kPlannerJointCount)
    {
      error = "every trajectory point must contain four positions";
      return false;
    }
    if ((!input_point.velocities.empty()) != has_velocities ||
        (has_velocities && input_point.velocities.size() != kPlannerJointCount))
    {
      error = "trajectory velocities must be either absent from every point or present for all four joints";
      return false;
    }
    if ((!input_point.accelerations.empty()) != has_accelerations ||
        (has_accelerations && input_point.accelerations.size() != kPlannerJointCount))
    {
      error = "trajectory accelerations must be either absent from every point or present for all four joints";
      return false;
    }
    if (!finiteVector(input_point.positions) || !finiteVector(input_point.velocities) ||
        !finiteVector(input_point.accelerations))
    {
      error = "trajectory contains NaN or Inf";
      return false;
    }

    const double point_time = input_point.time_from_start.toSec();
    if (!std::isfinite(point_time) || point_time < 0.0 ||
        (point_index > 0 && point_time <= previous_time + kTimeEpsilon))
    {
      error = "trajectory time_from_start values must be finite, non-negative and strictly increasing";
      return false;
    }

    NormalizedPoint point;
    point.time_from_start = point_time;
    for (std::size_t output_index = 0; output_index < kPlannerJointCount; ++output_index)
    {
      const std::size_t input_index = input_indices[output_index];
      double position = input_point.positions[input_index];
      const JointSpec& spec = joint_specs_[output_index];
      if (spec.continuous && !output.points.empty())
      {
        const double previous_position = output.points.back().positions[output_index];
        position = previous_position + shortestAngularDistance(previous_position, position);
      }
      else if (!spec.continuous && (position < spec.min_position || position > spec.max_position))
      {
        error = "trajectory position for " + spec.output_name + " is outside its configured range";
        return false;
      }
      point.positions[output_index] = position;
      if (has_velocities)
      {
        point.velocities[output_index] = input_point.velocities[input_index];
      }
      if (has_accelerations)
      {
        point.accelerations[output_index] = input_point.accelerations[input_index];
      }
    }
    output.points.push_back(point);
    previous_time = point_time;
  }
  return true;
}

void TrajectoryProcessor::alignContinuousJoints(const JointVector& reference,
                                                NormalizedTrajectory& trajectory) const
{
  if (trajectory.points.empty())
  {
    return;
  }
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    if (!joint_specs_[joint_index].continuous)
    {
      continue;
    }
    const double first = trajectory.points.front().positions[joint_index];
    const double aligned_first = reference[joint_index] + shortestAngularDistance(reference[joint_index], first);
    const double offset = aligned_first - first;
    for (NormalizedPoint& point : trajectory.points)
    {
      point.positions[joint_index] += offset;
    }
  }
}

void TrajectoryProcessor::prependReferencePoint(const JointVector& reference,
                                                NormalizedTrajectory& trajectory) const
{
  if (trajectory.points.empty() || trajectory.points.front().time_from_start <= kTimeEpsilon)
  {
    return;
  }
  NormalizedPoint start;
  start.positions = reference;
  start.time_from_start = 0.0;
  trajectory.points.insert(trajectory.points.begin(), start);
}

void TrajectoryProcessor::anchorStartToReference(
    const JointVector& reference, NormalizedTrajectory& trajectory) const
{
  if (trajectory.points.empty())
  {
    return;
  }
  if (trajectory.points.front().time_from_start > kTimeEpsilon)
  {
    prependReferencePoint(reference, trajectory);
    return;
  }

  // MoveIt can retain the previous requested goal as a zero-time start after
  // an execution timeout.  Never command that stale state.  A new execution
  // always starts at the encoder snapshot captured when the Action arrived.
  NormalizedPoint& start = trajectory.points.front();
  start.positions = reference;
  start.velocities.fill(0.0);
  start.accelerations.fill(0.0);
}

JointVector TrajectoryProcessor::sample(const NormalizedTrajectory& trajectory, double trajectory_time) const
{
  if (trajectory.points.empty())
  {
    return JointVector{};
  }
  if (trajectory_time <= trajectory.points.front().time_from_start)
  {
    return trajectory.points.front().positions;
  }
  if (trajectory_time >= trajectory.points.back().time_from_start)
  {
    return trajectory.points.back().positions;
  }

  const auto upper = std::upper_bound(
      trajectory.points.begin(), trajectory.points.end(), trajectory_time,
      [](double time, const NormalizedPoint& point) { return time < point.time_from_start; });
  const NormalizedPoint& end = *upper;
  const NormalizedPoint& start = *(upper - 1);
  return sampleSegment(start, end, trajectory.interpolation, trajectory_time);
}

bool TrajectoryProcessor::buildSamples(const NormalizedTrajectory& trajectory, double publish_rate,
                                       double requested_time_scale, double max_delta,
                                       std::vector<JointVector>& samples, double& applied_time_scale,
                                       std::string& error) const
{
  samples.clear();
  if (trajectory.points.empty() || !std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(requested_time_scale) || requested_time_scale <= 0.0 ||
      !std::isfinite(max_delta) || max_delta <= 0.0)
  {
    error = "invalid trajectory sampling configuration";
    return false;
  }

  const double trajectory_duration = trajectory.points.back().time_from_start;
  applied_time_scale = requested_time_scale;
  for (int iteration = 0; iteration < 12; ++iteration)
  {
    const double real_duration = trajectory_duration * applied_time_scale;
    const std::size_t step_count = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(real_duration * publish_rate)));
    if (step_count + 1 > kMaximumGeneratedSamples)
    {
      error = "scaled trajectory would generate too many command samples";
      return false;
    }

    samples.clear();
    samples.reserve(step_count + 1);
    double largest_delta = 0.0;
    for (std::size_t step = 0; step <= step_count; ++step)
    {
      const double real_time = std::min(real_duration, static_cast<double>(step) / publish_rate);
      const double trajectory_time = applied_time_scale <= 0.0 ? trajectory_duration :
                                     std::min(trajectory_duration, real_time / applied_time_scale);
      JointVector positions = sample(trajectory, trajectory_time);
      if (!samples.empty())
      {
        for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
        {
          largest_delta = std::max(largest_delta,
                                   std::abs(positions[joint_index] - samples.back()[joint_index]));
        }
      }
      samples.push_back(positions);
    }

    if (largest_delta <= max_delta * (1.0 + 1e-9))
    {
      return true;
    }
    applied_time_scale *= (largest_delta / max_delta) * 1.001;
  }

  error = "unable to satisfy max_delta with uniform trajectory time scaling";
  samples.clear();
  return false;
}

bool TrajectoryProcessor::validateTotalDisplacement(
    const JointVector& reference, const NormalizedTrajectory& trajectory,
    const JointVector& limits, std::string& error) const
{
  if (trajectory.points.empty())
  {
    error = "trajectory contains no points";
    return false;
  }
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    if (!std::isfinite(limits[joint_index]) || limits[joint_index] <= 0.0)
    {
      error = "total displacement limits must be finite and positive";
      return false;
    }
    for (const NormalizedPoint& point : trajectory.points)
    {
      const double displacement = std::abs(
          jointError(joint_index, point.positions[joint_index], reference[joint_index]));
      if (displacement > limits[joint_index] + kTimeEpsilon)
      {
        error = "trajectory total displacement exceeds safety limit for " +
                joint_specs_[joint_index].output_name;
        return false;
      }
    }
  }
  return true;
}

bool TrajectoryProcessor::isOppositeMotionEvidence(
    std::size_t joint_index, double desired, double previous_actual, double actual,
    double target_threshold, double feedback_threshold) const
{
  if (joint_index >= kPlannerJointCount || !std::isfinite(target_threshold) ||
      !std::isfinite(feedback_threshold) || target_threshold <= 0.0 ||
      feedback_threshold <= 0.0)
  {
    return false;
  }
  const double target_error = jointError(joint_index, desired, previous_actual);
  const double feedback_step = jointError(joint_index, actual, previous_actual);
  return std::abs(target_error) >= target_threshold &&
         std::abs(feedback_step) >= feedback_threshold &&
         target_error * feedback_step < 0.0;
}

double TrajectoryProcessor::jointError(std::size_t joint_index, double desired, double actual) const
{
  if (joint_index >= kPlannerJointCount)
  {
    return std::numeric_limits<double>::infinity();
  }
  return joint_specs_[joint_index].continuous ? shortestAngularDistance(actual, desired) : desired - actual;
}

const std::array<JointSpec, kPlannerJointCount>& TrajectoryProcessor::jointSpecs() const
{
  return joint_specs_;
}

bool TrajectoryProcessor::inputIndexMap(const std::vector<std::string>& input_names,
                                        std::array<std::size_t, kPlannerJointCount>& input_indices,
                                        std::string& error) const
{
  if (input_names.size() != kPlannerJointCount ||
      std::set<std::string>(input_names.begin(), input_names.end()).size() != kPlannerJointCount)
  {
    error = "trajectory must contain exactly four unique joint names";
    return false;
  }

  std::set<std::size_t> matched_inputs;
  for (std::size_t output_index = 0; output_index < kPlannerJointCount; ++output_index)
  {
    const JointSpec& spec = joint_specs_[output_index];
    bool found = false;
    for (std::size_t input_index = 0; input_index < input_names.size(); ++input_index)
    {
      if (std::find(spec.accepted_input_names.begin(), spec.accepted_input_names.end(), input_names[input_index]) ==
          spec.accepted_input_names.end())
      {
        continue;
      }
      if (found || matched_inputs.count(input_index) != 0)
      {
        error = "joint aliases are ambiguous";
        return false;
      }
      input_indices[output_index] = input_index;
      matched_inputs.insert(input_index);
      found = true;
    }
    if (!found)
    {
      error = "trajectory is missing required joint " + spec.output_name;
      return false;
    }
  }
  return true;
}

JointVector TrajectoryProcessor::sampleSegment(const NormalizedPoint& start, const NormalizedPoint& end,
                                               InterpolationMode mode, double trajectory_time) const
{
  JointVector output{};
  const double duration = end.time_from_start - start.time_from_start;
  const double s = std::max(0.0, std::min(1.0, (trajectory_time - start.time_from_start) / duration));
  for (std::size_t joint_index = 0; joint_index < kPlannerJointCount; ++joint_index)
  {
    const double p0 = start.positions[joint_index];
    const double p1 = end.positions[joint_index];
    if (mode == InterpolationMode::Linear)
    {
      output[joint_index] = p0 + (p1 - p0) * s;
      continue;
    }

    const double v0t = start.velocities[joint_index] * duration;
    const double v1t = end.velocities[joint_index] * duration;
    if (mode == InterpolationMode::Cubic)
    {
      const double s2 = s * s;
      const double s3 = s2 * s;
      output[joint_index] = (2.0 * s3 - 3.0 * s2 + 1.0) * p0 +
                            (s3 - 2.0 * s2 + s) * v0t +
                            (-2.0 * s3 + 3.0 * s2) * p1 +
                            (s3 - s2) * v1t;
      continue;
    }

    const double a0t2 = start.accelerations[joint_index] * duration * duration;
    const double a1t2 = end.accelerations[joint_index] * duration * duration;
    const double c0 = p0;
    const double c1 = v0t;
    const double c2 = 0.5 * a0t2;
    const double displacement = p1 - (c0 + c1 + c2);
    const double velocity_residual = v1t - (c1 + 2.0 * c2);
    const double acceleration_residual = a1t2 - 2.0 * c2;
    const double c3 = 10.0 * displacement - 4.0 * velocity_residual + 0.5 * acceleration_residual;
    const double c4 = -15.0 * displacement + 7.0 * velocity_residual - acceleration_residual;
    const double c5 = 6.0 * displacement - 3.0 * velocity_residual + 0.5 * acceleration_residual;
    output[joint_index] = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))));
  }
  return output;
}

double TrajectoryProcessor::shortestAngularDistance(double source, double target)
{
  return std::atan2(std::sin(target - source), std::cos(target - source));
}
}  // namespace b29_planner_adapter
