// SPDX-License-Identifier: BSD-3-Clause
#include <b29_smc_auto_controller/planner_control_coordinator.h>

namespace b29_smc_auto_controller
{
void PlannerControlCoordinator::configure(const Config& config)
{
  mode_ = config.mode;
  session_.configure(config.session);
  active_.store(false);
  release_requested_.store(false);
  release_received_for_stage_ = false;
  override_applied_ = false;
}

void PlannerControlCoordinator::enter(CrossingSide side, const Positions& reference_positions,
                                      const ros::Time& time)
{
  release_requested_.store(false);
  release_received_for_stage_ = false;
  override_applied_ = false;
  active_.store(true);
  if (isNormalMode())
  {
    session_.start(plannerCrossingSide(side), reference_positions, time);
  }
}

void PlannerControlCoordinator::leave(ExitCause cause)
{
  release_requested_.store(false);
  release_received_for_stage_ = false;
  override_applied_ = false;
  active_.store(false);
  if (isNormalMode())
  {
    session_.stop(plannerExitReason(cause), false);
  }
}

void PlannerControlCoordinator::refreshReferenceIfUncommanded(const Positions& reference_positions,
                                                               const ros::Time& time)
{
  if (isNormalMode() && active_.load())
  {
    session_.refreshReferenceIfUncommanded(reference_positions, time);
  }
}

PlannerSession::CommandResult PlannerControlCoordinator::acceptCommand(
    const PlannerJointCommand& command, const ros::Time& receive_time)
{
  if (!isNormalMode())
  {
    PlannerSession::CommandResult result;
    result.reject_reason = PlannerControlState::REJECT_NOT_ACTIVE;
    result.message = "formal Planner commands are disabled in debug interface mode";
    return result;
  }
  return session_.acceptCommand(command, receive_time);
}

PlannerSession::CompletionResult PlannerControlCoordinator::requestCompletion(
    std::uint32_t session_id, std::uint32_t final_sequence)
{
  if (!isNormalMode())
  {
    return {false, "formal Planner completion is disabled in debug interface mode"};
  }
  return session_.requestCompletion(session_id, final_sequence);
}

PlannerControlCoordinator::ManualReleaseResult PlannerControlCoordinator::requestManualRelease()
{
  if (!isDebugMode())
  {
    return {false, "Planner release is only available in debug interface mode"};
  }
  if (!active_.load())
  {
    return {false, "PlannerControl is not active"};
  }
  release_requested_.store(true);
  return {true, "Planner release accepted; transition will occur in the next control cycle"};
}

void PlannerControlCoordinator::beginCycle()
{
  override_applied_ = false;
}

PlannerControlCoordinator::Outcome PlannerControlCoordinator::pollOutcome(const ros::Time& time)
{
  if (!active_.load())
  {
    return Outcome::None;
  }

  if (isDebugMode())
  {
    if (!release_requested_.exchange(false))
    {
      return Outcome::None;
    }
    release_received_for_stage_ = true;
    active_.store(false);
    return Outcome::ManualReleased;
  }

  std::uint32_t completed_session = 0;
  std::uint32_t final_sequence = 0;
  if (session_.consumeCompletionRequest(completed_session, final_sequence))
  {
    session_.stop(PlannerControlState::EXIT_COMPLETED, true);
    active_.store(false);
    return Outcome::Completed;
  }
  if (session_.totalWatchdogExpired(time))
  {
    session_.stop(PlannerControlState::EXIT_TOTAL_WATCHDOG_TIMEOUT, false);
    active_.store(false);
    return Outcome::TotalWatchdogExpired;
  }
  return Outcome::None;
}

PlannerControlCoordinator::DispatchTicket PlannerControlCoordinator::commandCandidate(const ros::Time& time)
{
  DispatchTicket ticket;
  if (!isNormalMode() || !active_.load())
  {
    return ticket;
  }

  if (!session_.latestCommandIsFresh(time, ticket.positions, &ticket.sequence, &ticket.pending))
  {
    return ticket;
  }
  ticket.available = true;
  override_applied_ = true;
  return ticket;
}

void PlannerControlCoordinator::reportDispatch(const DispatchTicket& ticket, bool physically_dispatched)
{
  if (!isNormalMode() || !ticket.available || !ticket.pending || !physically_dispatched)
  {
    return;
  }
  session_.markCommandDispatched(ticket.sequence, ticket.positions);
}

PlannerControlState PlannerControlCoordinator::buildState(const ros::Time& stamp) const
{
  return session_.buildState(stamp);
}

PlannerControlCoordinator::TraceState PlannerControlCoordinator::traceState(const ros::Time& time) const
{
  TraceState state;
  state.manual_release_enabled = isDebugMode();
  state.release_received = release_received_for_stage_ || release_requested_.load();
  state.override_applied = override_applied_;
  if (!isNormalMode())
  {
    return state;
  }

  const PlannerControlState planner_state = session_.buildState(ros::Time{});
  state.point_available = planner_state.active && planner_state.has_accepted_command;
  Positions positions;
  state.point_fresh = session_.latestCommandIsFresh(time, positions);
  return state;
}

std::uint8_t PlannerControlCoordinator::plannerCrossingSide(CrossingSide side)
{
  if (side == CrossingSide::Left)
  {
    return PlannerControlState::CROSSING_SIDE_LEFT;
  }
  if (side == CrossingSide::Right)
  {
    return PlannerControlState::CROSSING_SIDE_RIGHT;
  }
  return PlannerControlState::CROSSING_SIDE_NONE;
}

std::uint8_t PlannerControlCoordinator::plannerExitReason(ExitCause cause)
{
  switch (cause)
  {
    case ExitCause::SafetyRevoked:
      return PlannerControlState::EXIT_SAFETY_REVOKED;
    case ExitCause::ManualIntervention:
      return PlannerControlState::EXIT_MANUAL_INTERVENTION;
    case ExitCause::ControllerStopped:
      return PlannerControlState::EXIT_CONTROLLER_STOPPED;
  }
  return PlannerControlState::EXIT_SAFETY_REVOKED;
}
}  // namespace b29_smc_auto_controller
