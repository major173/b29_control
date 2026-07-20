#include <b29_smc_auto_controller/remote_control_session.h>

#include <algorithm>
#include <cmath>

namespace b29_smc_auto_controller
{
void RemoteControlSession::configure(double max_increment_per_sample)
{
  max_increment_per_sample_ = max_increment_per_sample;
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
  if (!input.valid)
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
    applied_increments_[index] =
        std::max(-max_increment_per_sample_,
                 std::min(max_increment_per_sample_, raw_increments_[index]));
    targets_[index] += applied_increments_[index];
  }
  result.increments_applied = true;
  return result;
}
}  // namespace b29_smc_auto_controller
