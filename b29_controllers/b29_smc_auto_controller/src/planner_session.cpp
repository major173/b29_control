// SPDX-License-Identifier: BSD-3-Clause

#include <b29_smc_auto_controller/planner_session.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace b29_smc_auto_controller
{
PlannerSession::PlannerSession() = default;

PlannerSession::PlannerSession(const Config& config) : config_(config)
{
}

void PlannerSession::configure(const Config& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

uint32_t PlannerSession::start(uint8_t crossing_side, const Positions& reference_positions, const ros::Time& time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++session_id_;
  if (session_id_ == 0)
  {
    ++session_id_;
  }
  active_ = true;
  accepting_commands_ = true;
  crossing_side_ = crossing_side;
  session_start_time_ = time;
  reference_positions_ = reference_positions;
  reference_stamp_ = time;
  last_positions_ = Positions{};
  has_accepted_command_ = false;
  last_accepted_sequence_ = 0;
  last_rejected_sequence_ = 0;
  reject_reason_ = PlannerControlState::REJECT_NONE;
  rejection_count_ = 0;
  last_command_receive_time_ = ros::Time{};
  exit_reason_ = PlannerControlState::EXIT_NONE;
  completion_request_pending_ = false;
  completion_request_session_id_ = 0;
  completion_request_final_sequence_ = 0;
  return session_id_;
}

bool PlannerSession::refreshReferenceIfUncommanded(
    const Positions& reference_positions, const ros::Time& time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !accepting_commands_ || has_accepted_command_ ||
      !std::all_of(reference_positions.begin(), reference_positions.end(),
                   [](double position) { return std::isfinite(position); }))
  {
    return false;
  }

  reference_positions_ = reference_positions;
  reference_stamp_ = time;
  return true;
}

void PlannerSession::stop(uint8_t exit_reason, bool completed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_)
  {
    return;
  }

  active_ = false;
  accepting_commands_ = false;
  crossing_side_ = PlannerControlState::CROSSING_SIDE_NONE;
  exit_reason_ = exit_reason;
  completion_request_pending_ = false;
  if (completed)
  {
    last_completed_session_id_ = session_id_;
    last_completed_final_sequence_ = last_accepted_sequence_;
  }
}

PlannerSession::CommandResult PlannerSession::acceptCommand(const PlannerJointCommand& command,
                                                            const ros::Time& receive_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_NOT_ACTIVE,
                         "PlannerControl session is not active");
  }
  if (!accepting_commands_)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_NOT_ACCEPTING,
                         "PlannerControl session is not accepting commands");
  }
  if (command.session_id != session_id_)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_SESSION_MISMATCH,
                         "command session_id does not match the active session");
  }
  if (command.sequence == 0)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_INVALID_SEQUENCE,
                         "command sequence must start at 1");
  }
  if (command.header.stamp.isZero() ||
      std::abs((receive_time - command.header.stamp).toSec()) > config_.command_timeout)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_COMMAND_STALE,
                         "command timestamp is outside the configured timeout");
  }
  for (double position : command.positions)
  {
    if (!std::isfinite(position))
    {
      return rejectCommand(command.sequence, PlannerControlState::REJECT_NON_FINITE,
                           "command contains NaN or Inf");
    }
  }

  if (has_accepted_command_ && command.sequence == last_accepted_sequence_)
  {
    if (!samePositions(last_positions_, command.positions))
    {
      return rejectCommand(command.sequence, PlannerControlState::REJECT_DUPLICATE_SEQUENCE_MISMATCH,
                           "duplicate sequence contains different positions");
    }
    last_command_receive_time_ = receive_time;
    return CommandResult{true, true, PlannerControlState::REJECT_NONE,
                         "duplicate command accepted idempotently"};
  }

  const uint32_t expected_sequence = has_accepted_command_ ? last_accepted_sequence_ + 1u : 1u;
  if (command.sequence < expected_sequence)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_INVALID_SEQUENCE,
                         "command sequence is older than the expected sequence");
  }
  if (command.sequence > expected_sequence)
  {
    return rejectCommand(command.sequence, PlannerControlState::REJECT_SEQUENCE_GAP,
                         "command sequence skipped an unaccepted point");
  }

  const Positions& reference = has_accepted_command_ ? last_positions_ : reference_positions_;
  for (std::size_t index = 0; index < reference.size(); ++index)
  {
    if (std::abs(command.positions[index] - reference[index]) > config_.max_delta_per_command)
    {
      return rejectCommand(command.sequence, PlannerControlState::REJECT_DELTA_EXCEEDED,
                           has_accepted_command_ ? "command delta exceeds max_delta_per_command"
                                                 : "first command delta from current posture exceeds max_delta_per_command");
    }
  }

  std::copy(command.positions.begin(), command.positions.end(), last_positions_.begin());
  has_accepted_command_ = true;
  last_accepted_sequence_ = command.sequence;
  last_command_receive_time_ = receive_time;
  return CommandResult{true, false, PlannerControlState::REJECT_NONE, "command accepted"};
}

PlannerSession::CompletionResult PlannerSession::requestCompletion(uint32_t session_id, uint32_t final_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_id == last_completed_session_id_ && final_sequence == last_completed_final_sequence_)
  {
    return CompletionResult{true, "PlannerControl session already completed"};
  }
  if (!active_)
  {
    return CompletionResult{false, "PlannerControl session is not active"};
  }
  if (session_id != session_id_)
  {
    return CompletionResult{false, "completion session_id does not match the active session"};
  }
  if (!has_accepted_command_)
  {
    return CompletionResult{false, "no Planner command has been accepted in this session"};
  }
  if (final_sequence != last_accepted_sequence_)
  {
    return CompletionResult{false, "final_sequence is not the last accepted sequence"};
  }
  if (completion_request_pending_)
  {
    if (completion_request_session_id_ == session_id && completion_request_final_sequence_ == final_sequence)
    {
      return CompletionResult{true, "completion request already pending"};
    }
    return CompletionResult{false, "a different completion request is already pending"};
  }

  completion_request_pending_ = true;
  completion_request_session_id_ = session_id;
  completion_request_final_sequence_ = final_sequence;
  return CompletionResult{true, "completion request accepted for the next control cycle"};
}

bool PlannerSession::consumeCompletionRequest(uint32_t& session_id, uint32_t& final_sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!completion_request_pending_ || !active_ || completion_request_session_id_ != session_id_ ||
      !has_accepted_command_ || completion_request_final_sequence_ != last_accepted_sequence_)
  {
    completion_request_pending_ = false;
    return false;
  }

  session_id = completion_request_session_id_;
  final_sequence = completion_request_final_sequence_;
  completion_request_pending_ = false;
  return true;
}

bool PlannerSession::latestCommandIsFresh(const ros::Time& time, Positions& positions) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !has_accepted_command_ || last_command_receive_time_.isZero() ||
      (time - last_command_receive_time_).toSec() > config_.command_timeout)
  {
    return false;
  }
  positions = last_positions_;
  return true;
}

bool PlannerSession::acceptedCommandTimedOut(const ros::Time& time) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_ && has_accepted_command_ && !last_command_receive_time_.isZero() &&
         (time - last_command_receive_time_).toSec() > config_.command_timeout;
}

bool PlannerSession::totalWatchdogExpired(const ros::Time& time) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_ && !session_start_time_.isZero() &&
         (time - session_start_time_).toSec() >= config_.total_watchdog_timeout;
}

bool PlannerSession::active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

PlannerControlState PlannerSession::buildState(const ros::Time& stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PlannerControlState state;
  state.header.stamp = stamp;
  state.session_id = session_id_;
  state.active = active_;
  state.accepting_commands = accepting_commands_;
  state.crossing_side = crossing_side_;
  state.has_accepted_command = has_accepted_command_;
  state.last_accepted_sequence = last_accepted_sequence_;
  state.last_rejected_sequence = last_rejected_sequence_;
  state.reject_reason = reject_reason_;
  state.rejection_count = rejection_count_;
  state.last_completed_session_id = last_completed_session_id_;
  state.exit_reason = exit_reason_;
  state.max_delta_per_command = config_.max_delta_per_command;
  state.command_timeout = config_.command_timeout;
  std::copy(reference_positions_.begin(), reference_positions_.end(),
            state.reference_positions.begin());
  state.reference_stamp = reference_stamp_;
  return state;
}

PlannerSession::CommandResult PlannerSession::rejectCommand(uint32_t sequence, uint8_t reason,
                                                            const std::string& message)
{
  last_rejected_sequence_ = sequence;
  reject_reason_ = reason;
  ++rejection_count_;
  return CommandResult{false, false, reason, message};
}

bool PlannerSession::samePositions(const Positions& lhs, const PlannerJointCommand::_positions_type& rhs)
{
  return std::equal(lhs.begin(), lhs.end(), rhs.begin());
}
}  // namespace b29_smc_auto_controller
