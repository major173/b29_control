// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include <ros/time.h>

#include <b29_smc_auto_controller/auto_types.h>
#include <b29_smc_auto_controller/planner_session.h>

namespace b29_smc_auto_controller
{
// Coordinates PlannerControl stage integration while PlannerSession retains
// the formal Planner protocol state and validation rules.
class PlannerControlCoordinator
{
public:
  using Positions = PlannerSession::Positions;

  enum class Mode
  {
    Normal,
    Debug,
  };

  enum class Outcome
  {
    None,
    Completed,
    ManualReleased,
    TotalWatchdogExpired,
  };

  enum class ExitCause
  {
    SafetyRevoked,
    ManualIntervention,
    ControllerStopped,
  };

  struct Config
  {
    Mode mode{Mode::Normal};
    PlannerSession::Config session{};
  };

  struct DispatchTicket
  {
    bool available{false};
    bool pending{false};
    std::uint32_t sequence{0};
    Positions positions{};
  };

  struct ManualReleaseResult
  {
    bool accepted{false};
    std::string message;
  };

  struct TraceState
  {
    bool manual_release_enabled{false};
    bool release_received{false};
    bool point_available{false};
    bool point_fresh{false};
    bool override_applied{false};
  };

  void configure(const Config& config);

  Mode mode() const { return mode_; }
  bool isNormalMode() const { return mode_ == Mode::Normal; }
  bool isDebugMode() const { return mode_ == Mode::Debug; }

  void enter(CrossingSide side, const Positions& reference_positions, const ros::Time& time);
  void leave(ExitCause cause);
  void refreshReferenceIfUncommanded(const Positions& reference_positions, const ros::Time& time);

  PlannerSession::CommandResult acceptCommand(const PlannerJointCommand& command,
                                              const ros::Time& receive_time);
  PlannerSession::CompletionResult requestCompletion(std::uint32_t session_id,
                                                     std::uint32_t final_sequence);
  ManualReleaseResult requestManualRelease();

  void beginCycle();
  Outcome pollOutcome(const ros::Time& time);
  DispatchTicket commandCandidate(const ros::Time& time);
  void reportDispatch(const DispatchTicket& ticket, bool physically_dispatched);

  PlannerControlState buildState(const ros::Time& stamp) const;
  TraceState traceState(const ros::Time& time) const;

private:
  static std::uint8_t plannerCrossingSide(CrossingSide side);
  static std::uint8_t plannerExitReason(ExitCause cause);

  Mode mode_{Mode::Normal};
  PlannerSession session_{};
  std::atomic<bool> active_{false};
  std::atomic<bool> release_requested_{false};
  bool release_received_for_stage_{false};
  bool override_applied_{false};
};
}  // namespace b29_smc_auto_controller
