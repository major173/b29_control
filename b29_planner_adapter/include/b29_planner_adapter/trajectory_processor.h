// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <trajectory_msgs/JointTrajectory.h>

namespace b29_planner_adapter
{
constexpr std::size_t kPlannerJointCount = 4;
using JointVector = std::array<double, kPlannerJointCount>;

struct JointSpec
{
  std::string output_name;
  std::vector<std::string> accepted_input_names;
  bool continuous{false};
  double min_position{0.0};
  double max_position{0.0};
};

enum class InterpolationMode
{
  Linear,
  Cubic,
  Quintic,
};

struct NormalizedPoint
{
  double time_from_start{0.0};
  JointVector positions{};
  JointVector velocities{};
  JointVector accelerations{};
};

struct NormalizedTrajectory
{
  std::vector<NormalizedPoint> points;
  InterpolationMode interpolation{InterpolationMode::Linear};
};

class TrajectoryProcessor
{
public:
  explicit TrajectoryProcessor(std::array<JointSpec, kPlannerJointCount> joint_specs);

  bool normalize(const trajectory_msgs::JointTrajectory& input, NormalizedTrajectory& output,
                 std::string& error) const;
  void alignContinuousJoints(const JointVector& reference, NormalizedTrajectory& trajectory) const;
  void anchorStartToReference(const JointVector& reference, NormalizedTrajectory& trajectory) const;
  void prependReferencePoint(const JointVector& reference, NormalizedTrajectory& trajectory) const;
  JointVector sample(const NormalizedTrajectory& trajectory, double trajectory_time) const;
  bool buildSamples(const NormalizedTrajectory& trajectory, double publish_rate, double requested_time_scale,
                    double max_delta, std::vector<JointVector>& samples, double& applied_time_scale,
                    std::string& error) const;
  bool validateTotalDisplacement(const JointVector& reference,
                                 const NormalizedTrajectory& trajectory,
                                 const JointVector& limits,
                                 std::string& error) const;
  bool isOppositeMotionEvidence(std::size_t joint_index, double desired,
                                double previous_actual, double actual,
                                double target_threshold,
                                double feedback_threshold) const;
  double jointError(std::size_t joint_index, double desired, double actual) const;
  const std::array<JointSpec, kPlannerJointCount>& jointSpecs() const;

private:
  bool inputIndexMap(const std::vector<std::string>& input_names,
                     std::array<std::size_t, kPlannerJointCount>& input_indices,
                     std::string& error) const;
  JointVector sampleSegment(const NormalizedPoint& start, const NormalizedPoint& end,
                            InterpolationMode mode, double trajectory_time) const;
  static double shortestAngularDistance(double source, double target);

  std::array<JointSpec, kPlannerJointCount> joint_specs_;
};
}  // namespace b29_planner_adapter
