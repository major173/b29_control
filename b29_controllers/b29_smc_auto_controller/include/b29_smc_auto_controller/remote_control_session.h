#pragma once

#include <array>
#include <cstdint>

namespace b29_smc_auto_controller
{
class RemoteControlSession
{
public:
  using Positions = std::array<double, 4>;

  struct Config
  {
    double increment_deadband{0.002};
    double increment_scale{0.05};
    double max_increment_per_sample{0.10};
    Positions joint_direction_signs{{-1.0, 1.0, 1.0, 1.0}};
  };

  struct Input
  {
    Positions increments{};
    bool increments_valid{false};
    std::uint64_t sample_sequence{0};
    std::uint64_t completion_rising_edge_sequence{0};
  };

  struct UpdateResult
  {
    bool completion_rising_edge{false};
    bool sample_consumed{false};
    bool increments_applied{false};
  };

  void configure(const Config& config);
  void start(const Positions& reference_positions, const Input& input);
  void reset();
  UpdateResult update(const Input& input);

  const Positions& targets() const { return targets_; }
  const Positions& rawIncrements() const { return raw_increments_; }
  const Positions& appliedIncrements() const { return applied_increments_; }

private:
  Config config_{};
  Positions targets_{};
  Positions raw_increments_{};
  Positions applied_increments_{};
  std::uint64_t last_sample_sequence_{0};
  std::uint64_t completion_baseline_{0};
  bool active_{false};
};
}  // namespace b29_smc_auto_controller
