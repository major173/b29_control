// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/disconnect_cable_process.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace b29_smc_auto_controller
{
namespace
{
double clampUnit(double value)
{
  return std::max(0.0, std::min(1.0, value));
}
}

void DisconnectCableProcess::configure(const Config& config)
{
  config_ = config;
}

void DisconnectCableProcess::start(const ros::Time& time, const CrossingSideProfile& profile,
                                   const std::string& reason)
{
  profile_ = &profile;
  pending_outcome_ = Outcome::None;
  max_abs_pose_joint_velocity_ = 0.0;
  velocity_within_threshold_ = false;
  velocity_stable_since_ = ros::Time{};
  resetMotionSegment();
  transition(DisconnectCableStep::Step3UpFirstJoint, time, reason);
}

void DisconnectCableProcess::restart(const ros::Time& time, const std::string& reason)
{
  pending_outcome_ = Outcome::None;
  max_abs_pose_joint_velocity_ = 0.0;
  velocity_within_threshold_ = false;
  velocity_stable_since_ = ros::Time{};
  resetMotionSegment();
  transition(DisconnectCableStep::Step1LoosenGripper, time, reason);
}

void DisconnectCableProcess::reset(const ros::Time& time, const std::string& reason)
{
  profile_ = nullptr;
  pending_outcome_ = Outcome::None;
  max_abs_pose_joint_velocity_ = 0.0;
  velocity_within_threshold_ = false;
  velocity_stable_since_ = ros::Time{};
  resetMotionSegment();
  transition(DisconnectCableStep::Idle, time, reason);
}

DisconnectCableProcess::Result DisconnectCableProcess::update(const ros::Time& time,
                                                                const JointTargets& command_targets,
                                                                const JointTargets& joint_feedback,
                                                                const JointTargets& joint_velocities)
{
  Result result;
  result.joint_targets = command_targets;
  if (profile_ == nullptr)
  {
    return result;
  }

  result.valid = true;
  result.joint_targets[jointIndex(profile_->gripper)] = static_cast<double>(static_cast<uint8_t>(GripperState::Open));
  const std::string prefix = std::string("disconnect_") + crossingSideReasonName(profile_->side) + "_";

  switch (step_)
  {
    case DisconnectCableStep::Step1LoosenGripper:
      result.command_reason = prefix + "step1_loosen_gripper";
      transition(DisconnectCableStep::Step2WaitGripperRespond, time, "open_gripper_command_sent");
      break;
    case DisconnectCableStep::Step2WaitGripperRespond:
      result.command_reason = prefix + "step2_wait_gripper";
      if ((time - step_enter_time_) >= ros::Duration(config_.wait_for_grip_respond_time))
      {
        resetMotionSegment();
        transition(DisconnectCableStep::Step3UpFirstJoint, time, "gripper_response_wait_elapsed");
      }
      break;
    case DisconnectCableStep::Step3UpFirstJoint:
      result.command_reason = prefix + "step3_up_first_joint";
      if (applyMotionSegment(time, step_, command_targets, joint_feedback, result.joint_targets))
      {
        resetMotionSegment();
        transition(DisconnectCableStep::Step5DownFirstJoint, time, "step3_motion_completed");
      }
      break;
    case DisconnectCableStep::Step5DownFirstJoint:
      result.command_reason = prefix + "step5_down_first_joint";
      if (applyMotionSegment(time, step_, command_targets, joint_feedback, result.joint_targets))
      {
        resetMotionSegment();
        transition(DisconnectCableStep::Step7CheckIfCableDisconnected, time, "step5_motion_completed");
      }
      break;
    case DisconnectCableStep::Step7CheckIfCableDisconnected:
    {
      result.command_reason = prefix + "step7_check_disconnect";
      // The locked arm's first joint is now the only commanded disconnect
      // joint. Step7 is therefore a pure three-pose-joint settling check.
      max_abs_pose_joint_velocity_ = 0.0;
      velocity_within_threshold_ = true;
      for (JointIndex joint : profile_->pose_joints)
      {
        const double velocity = joint_velocities[jointIndex(joint)];
        if (!std::isfinite(velocity))
        {
          max_abs_pose_joint_velocity_ = std::numeric_limits<double>::infinity();
          velocity_within_threshold_ = false;
          break;
        }
        max_abs_pose_joint_velocity_ = std::max(max_abs_pose_joint_velocity_, std::abs(velocity));
        if (std::abs(velocity) > config_.settle_velocity_threshold)
        {
          velocity_within_threshold_ = false;
        }
      }

      if (!velocity_within_threshold_)
      {
        velocity_stable_since_ = ros::Time{};
        break;
      }
      if (velocity_stable_since_.isZero())
      {
        velocity_stable_since_ = time;
      }
      if ((time - velocity_stable_since_) >= ros::Duration(config_.settle_duration))
      {
        pending_outcome_ = Outcome::Completed;
      }
      break;
    }
    case DisconnectCableStep::Failed:
      result.command_reason = prefix + "failed";
      break;
    case DisconnectCableStep::Idle:
    default:
      result.valid = false;
      break;
  }

  return result;
}

DisconnectCableProcess::Outcome DisconnectCableProcess::consumeOutcome()
{
  const Outcome outcome = pending_outcome_;
  pending_outcome_ = Outcome::None;
  return outcome;
}

DisconnectCableProcess::TraceState DisconnectCableProcess::traceState() const
{
  TraceState state;
  state.step = step_;
  state.step_enter_time = step_enter_time_;
  state.transition_reason = transition_reason_;
  state.max_abs_pose_joint_velocity = max_abs_pose_joint_velocity_;
  state.settle_velocity_threshold = config_.settle_velocity_threshold;
  state.velocity_within_threshold = velocity_within_threshold_;
  state.velocity_stable_since = velocity_stable_since_;
  state.settle_duration = config_.settle_duration;
  return state;
}

void DisconnectCableProcess::transition(DisconnectCableStep step, const ros::Time& time,
                                        const std::string& reason)
{
  step_ = step;
  step_enter_time_ = time;
  transition_reason_ = reason;
}

void DisconnectCableProcess::resetMotionSegment()
{
  motion_segment_ = MotionSegment{};
}

bool DisconnectCableProcess::applyMotionSegment(const ros::Time& time, DisconnectCableStep step,
                                                 const JointTargets& command_targets,
                                                 const JointTargets& /*joint_feedback*/,
                                                 JointTargets& result_targets)
{
  if (!motion_segment_.initialized || motion_segment_.step != step)
  {
    motion_segment_.step = step;
    motion_segment_.start_time = time;
    motion_segment_.duration = ros::Duration(config_.step_duration);
    motion_segment_.start_targets = command_targets;
    motion_segment_.target_targets = command_targets;
    motion_segment_.initialized = true;

    const std::size_t first_joint = jointIndex(profile_->actuator_first_leg);
    switch (step)
    {
      case DisconnectCableStep::Step3UpFirstJoint:
        motion_segment_.target_targets[first_joint] =
            profile_->motion_sign * config_.step3_first_joint_target;
        break;
      case DisconnectCableStep::Step5DownFirstJoint:
        motion_segment_.target_targets[first_joint] =
            profile_->motion_sign * config_.step5_first_joint_target;
        break;
      default:
        break;
    }
  }

  const double duration = motion_segment_.duration.toSec();
  double alpha = 1.0;
  if (duration > 0.0)
  {
    alpha = clampUnit((time - motion_segment_.start_time).toSec() / duration);
  }
  for (std::size_t index = 0; index < result_targets.size(); ++index)
  {
    result_targets[index] = motion_segment_.start_targets[index] +
                            (motion_segment_.target_targets[index] - motion_segment_.start_targets[index]) * alpha;
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
  return (time - motion_segment_.completed_time) >= ros::Duration(config_.step_interval);
}

std::size_t DisconnectCableProcess::jointIndex(JointIndex joint)
{
  return static_cast<std::size_t>(joint);
}
}  // namespace b29_smc_auto_controller
