// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <string>

#include <ros/time.h>

#include <b29_smc_auto_controller/crossing_side_profile.h>

namespace b29_smc_auto_controller
{
enum class DisconnectCableStep
{
  Idle,
  Step1LoosenGripper,
  Step2WaitGripperRespond,
  Step3UpFirstJoint,
  Step4MoveSecondJoint,
  Step5DownFirstJoint,
  Step6MoveSecondJoint,
  Step7CheckIfCableDisconnected,
  Step8ReturnToZero,
  Failed,
};

class DisconnectCableProcess
{
public:
  using JointTargets = std::array<double, 6>;

  struct Config
  {
    double wait_for_grip_respond_time{12.0};
    double step_duration{5.0};
    double step_interval{1.0};
    double succeed_threshold{0.17};
    double step6_second_joint_delta{0.20};
    double step3_first_joint_target{-0.41};
    double step4_second_joint_target{-0.12};
    double step5_first_joint_target{1.0};
  };

  enum class Outcome
  {
    None,
    Completed,
    Failed,
  };

  struct Result
  {
    JointTargets joint_targets{};
    std::string command_reason;
    bool valid{false};
  };

  struct TraceState
  {
    DisconnectCableStep step{DisconnectCableStep::Idle};
    ros::Time step_enter_time{};
    std::string transition_reason;
    double check_displacement{0.0};
    double check_reference_position{0.0};
    double succeed_threshold{0.0};
  };

  void configure(const Config& config);
  void start(const ros::Time& time, const CrossingSideProfile& profile, const std::string& reason);
  void restart(const ros::Time& time, const std::string& reason);
  void reset(const ros::Time& time, const std::string& reason);

  Result update(const ros::Time& time, const JointTargets& command_targets,
                const JointTargets& joint_feedback);
  Outcome consumeOutcome();
  TraceState traceState() const;

private:
  struct MotionSegment
  {
    DisconnectCableStep step{DisconnectCableStep::Idle};
    ros::Time start_time{};
    ros::Duration duration{};
    JointTargets start_targets{};
    JointTargets target_targets{};
    bool initialized{false};
    bool completed{false};
    ros::Time completed_time{};
  };

  void transition(DisconnectCableStep step, const ros::Time& time, const std::string& reason);
  void resetMotionSegment();
  bool applyMotionSegment(const ros::Time& time, DisconnectCableStep step,
                          const JointTargets& command_targets, const JointTargets& joint_feedback,
                          JointTargets& result_targets);
  static std::size_t jointIndex(JointIndex joint);

  Config config_{};
  const CrossingSideProfile* profile_{nullptr};
  DisconnectCableStep step_{DisconnectCableStep::Idle};
  ros::Time step_enter_time_{};
  std::string transition_reason_;
  MotionSegment motion_segment_{};
  double to_check_joint_pos_{0.0};
  double disconnect_check_displacement_{0.0};
  Outcome pending_outcome_{Outcome::None};
};
}  // namespace b29_smc_auto_controller
