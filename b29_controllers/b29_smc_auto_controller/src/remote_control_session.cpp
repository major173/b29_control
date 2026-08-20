#include <b29_smc_auto_controller/remote_control_session.h>

#include <algorithm>
#include <cmath>

namespace b29_smc_auto_controller
{
void RemoteControlSession::configure(const Config& config)
{
  config_ = config;
}

void RemoteControlSession::start(const Positions& reference_positions, const Input& input)
{
  targets_ = reference_positions;
  raw_increments_.fill(0.0);
  applied_increments_.fill(0.0);
  last_sample_sequence_ = input.sample_sequence;
  completion_baseline_ = input.completion_rising_edge_sequence;
  active_ = true;
}

void RemoteControlSession::reset()
{
  targets_.fill(0.0);
  raw_increments_.fill(0.0);
  applied_increments_.fill(0.0);
  last_sample_sequence_ = 0;
  completion_baseline_ = 0;
  active_ = false;
}

RemoteControlSession::UpdateResult RemoteControlSession::update(const Input& input)
{
  UpdateResult result;
  if (!active_)
  {
    return result;
  }
  if (input.completion_rising_edge_sequence > completion_baseline_)
  {
    result.completion_rising_edge = true;
    return result;
  }
  if (input.sample_sequence == last_sample_sequence_)
  {
    return result;
  }

  last_sample_sequence_ = input.sample_sequence;
  raw_increments_ = input.increments;
  applied_increments_.fill(0.0);
  result.sample_consumed = true;
  if (!input.increments_valid)
  {
    return result;
  }
  for (double increment : raw_increments_)
  {
    if (!std::isfinite(increment))
    {
      return result;
    }
  }
  for (std::size_t index = 0; index < raw_increments_.size(); ++index)
  {
    if (std::abs(raw_increments_[index]) <= config_.increment_deadband)
    {
      continue;
    }
    const double scaled_increment =
        raw_increments_[index] * config_.increment_scale * config_.joint_direction_signs[index];
    applied_increments_[index] =
        std::max(-config_.max_increment_per_sample,
                 std::min(config_.max_increment_per_sample, scaled_increment));
    targets_[index] += applied_increments_[index];
  }
  result.increments_applied = true;
  return result;
}
}  // namespace b29_smc_auto_controller
