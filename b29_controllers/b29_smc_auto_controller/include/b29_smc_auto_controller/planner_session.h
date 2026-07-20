// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>

#include <ros/time.h>

#include <b29_smc_auto_controller/PlannerControlState.h>
#include <b29_smc_auto_controller/PlannerJointCommand.h>

namespace b29_smc_auto_controller
{
class PlannerSession
{
public:
  using Positions = std::array<double, 4>;

  struct Config
  {
    double max_delta_per_command{0.10};
    double command_timeout{1.0};
    double total_watchdog_timeout{120.0};
  };

  struct CommandResult
  {
    bool accepted{false};
    bool duplicate{false};
    uint8_t reject_reason{PlannerControlState::REJECT_NONE};
    std::string message;
  };

  struct CompletionResult
  {
    bool accepted{false};
    std::string message;
  };

  PlannerSession();
  explicit PlannerSession(const Config& config);

  void configure(const Config& config);
  uint32_t start(uint8_t crossing_side, const Positions& reference_positions, const ros::Time& time);
  void stop(uint8_t exit_reason, bool completed);
  bool refreshReferenceIfUncommanded(const Positions& reference_positions, const ros::Time& time);

  CommandResult acceptCommand(const PlannerJointCommand& command, const ros::Time& receive_time);
  CompletionResult requestCompletion(uint32_t session_id, uint32_t final_sequence);
  bool consumeCompletionRequest(uint32_t& session_id, uint32_t& final_sequence);

  bool latestCommandIsFresh(const ros::Time& time, Positions& positions) const;
  // True only after this session has accepted a command and its command stream
  // has then been absent longer than command_timeout.
  bool acceptedCommandTimedOut(const ros::Time& time) const;
  bool totalWatchdogExpired(const ros::Time& time) const;
  bool active() const;
  PlannerControlState buildState(const ros::Time& stamp) const;

private:
  CommandResult rejectCommand(uint32_t sequence, uint8_t reason, const std::string& message);
  static bool samePositions(const Positions& lhs, const PlannerJointCommand::_positions_type& rhs);

  mutable std::mutex mutex_;
  Config config_;
  uint32_t session_id_{0};
  bool active_{false};
  bool accepting_commands_{false};
  uint8_t crossing_side_{PlannerControlState::CROSSING_SIDE_NONE};
  ros::Time session_start_time_{};
  Positions reference_positions_{};
  ros::Time reference_stamp_{};
  Positions last_positions_{};
  bool has_accepted_command_{false};
  uint32_t last_accepted_sequence_{0};
  uint32_t last_rejected_sequence_{0};
  uint8_t reject_reason_{PlannerControlState::REJECT_NONE};
  uint32_t rejection_count_{0};
  ros::Time last_command_receive_time_{};
  uint32_t last_completed_session_id_{0};
  uint32_t last_completed_final_sequence_{0};
  uint8_t exit_reason_{PlannerControlState::EXIT_NONE};
  bool completion_request_pending_{false};
  uint32_t completion_request_session_id_{0};
  uint32_t completion_request_final_sequence_{0};
};
}  // namespace b29_smc_auto_controller
