#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <std_msgs/Header.h>

namespace steering_engine_hw
{
struct RemoteControlData
{
  std_msgs::Header header;
  std::array<double, 4> joint_increments{{0.0, 0.0, 0.0, 0.0}};
  bool stage_complete{false};
  bool valid{false};
  std::uint64_t sample_sequence{0};
  std::uint64_t completion_rising_edge_sequence{0};
};

class RemoteControlHandle
{
public:
  RemoteControlHandle() = default;
  RemoteControlHandle(const std::string& name, RemoteControlData* data) : name_(name), data_(data) {}

  std::string getName() const { return name_; }
  const RemoteControlData& getData() const { return *data_; }
  bool valid() const { return data_ != nullptr; }

private:
  std::string name_;
  RemoteControlData* data_{nullptr};
};

class RemoteControlInterface : public hardware_interface::HardwareResourceManager<RemoteControlHandle>
{
};
}  // namespace steering_engine_hw
