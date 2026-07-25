// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <ros/time.h>

#include <b29_smc_auto_controller/crossing_side_profile.h>
#include <b29_smc_auto_controller/disconnect_cable_process.h>

namespace b29_smc_auto_controller
{
class ObstacleCrossingRuntime
{
public:
  struct Config
  {
    double wait_for_grip_respond_time{12.0};
    int retry_limit{2};
  };

  enum class PlannerOutcome
  {
    None,
    Completed,
    ManualReleased,
    TotalWatchdogExpired,
  };

  enum class PlannerAction
  {
    None,
    Start,
    RevokeForSafety,
    RevokeForManualIntervention,
  };

  struct Inputs
  {
    bool traversing{false};
    bool obstacle_detected{false};
    bool obstacle_within_threshold{false};
    double cruise_speed{0.0};
    bool grip_confirmed{false};
    bool safety_blocked{false};
  };

  struct Events
  {
    DisconnectCableProcess::Outcome disconnect{DisconnectCableProcess::Outcome::None};
    PlannerOutcome planner{PlannerOutcome::None};
    bool remote_control_completion_rising_edge{false};
  };

  struct Actions
  {
    bool start_disconnect_process{false};
    bool restart_disconnect_process{false};
    bool reset_disconnect_process{false};
    PlannerAction planner_action{PlannerAction::None};
    bool start_remote_control_session{false};
    bool reset_remote_control_session{false};
  };

  struct TraceState
  {
    ObstacleCrossingStage stage{ObstacleCrossingStage::Idle};
    CrossingSide crossing_side{CrossingSide::None};
    CrossingSide first_crossing_side{CrossingSide::Left};
    ros::Time stage_enter_time{};
    std::string transition_reason;
    FailedOperation failed_operation{FailedOperation::None};
    std::uint32_t retry_count{0};
    std::uint32_t close_grippers_retry_count{0};
    std::uint32_t left_disconnect_retry_count{0};
    std::uint32_t right_disconnect_retry_count{0};
    std::uint32_t left_regrip_retry_count{0};
    std::uint32_t right_regrip_retry_count{0};
    std::uint32_t retry_limit{0};
    ros::Time gripper_wait_start_time{};
    double wait_for_grip_respond_time{0.0};
    std::string last_failure_reason;
  };

  void configure(const Config& config);
  Actions update(const ros::Time& time, const Inputs& inputs, const Events& events);
  Actions reset(const ros::Time& time, bool grip_confirmed, const std::string& reason);
  void reportFailure(const std::string& reason);

  ObstacleCrossingStage stage() const { return stage_; }
  CrossingSide crossingSide() const { return crossing_side_; }
  const CrossingSideProfile* currentProfile() const;
  GravityCompensationMode gravityCompensationMode() const { return gravity_compensation_latched_mode_; }
  bool stageUsesLastEffectiveTargets() const;
  std::string manualInterventionCommandReason() const;
  TraceState traceState() const;

private:
  struct RetryCounts
  {
    int close_grippers{0};
    std::array<int, 2> disconnect{{0, 0}};
    std::array<int, 2> regrip{{0, 0}};

    void reset();
  };

  void transition(ObstacleCrossingStage stage, const ros::Time& time, const std::string& reason);
  void enterManualIntervention(FailedOperation operation, CrossingSide side, const ros::Time& time,
                               Actions& actions, const std::string& reason);
  void enterRemoteControl(const ros::Time& time, const std::string& reason, Actions& actions);
  void enterOpenGripperBeforeGravityCompensation(CrossingSide side, const ros::Time& time,
                                                 const std::string& reason);
  void enterCompleteWaitObstacleClear(const ros::Time& time, const std::string& reason,
                                      Actions& actions);
  void refreshGravityCompensationLatch();
  bool consumeGripConfirmation(bool grip_confirmed);
  bool gripWaitElapsed(const ros::Time& time) const;
  int* retryCounter(FailedOperation operation, CrossingSide side);
  const int* retryCounter(FailedOperation operation, CrossingSide side) const;
  FailedOperation failedOperationForStage() const;
  static CrossingSide oppositeSide(CrossingSide side);
  static CrossingSide firstSideFromCruiseSpeed(double cruise_speed);
  static const char* failedOperationReasonName(FailedOperation operation);

  Config config_{};
  ObstacleCrossingStage stage_{ObstacleCrossingStage::Idle};
  CrossingSide crossing_side_{CrossingSide::None};
  CrossingSide first_crossing_side_{CrossingSide::Left};
  FailedOperation failed_operation_{FailedOperation::None};
  RetryCounts retry_counts_{};
  ros::Time gripper_wait_start_time_{};
  ros::Time stage_enter_time_{};
  std::string transition_reason_;
  std::string last_failure_reason_;
  bool grip_confirmation_low_seen_{false};
  GravityCompensationMode gravity_compensation_latched_mode_{GravityCompensationMode::Off};
};
}  // namespace b29_smc_auto_controller
