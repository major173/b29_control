#pragma once

#include <cstdint>
#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <std_msgs/Header.h>

namespace steering_engine_hw{

enum class ObstacleType : uint8_t
{
    OBSTACLE_UNKNOWN=0,
    OBSTACLE_LINE_CLAMP=1,
    OBSTACLE_DAMPER=2
};

struct AutoStateData
{   
    std_msgs::Header header;

    bool lower_alive{false};
    bool grip_confirmed{false};
    bool joint_fault{false};
    bool grip_fault{false};
    bool imu_ready{false};

    ObstacleType obstacle_type{ObstacleType::OBSTACLE_UNKNOWN};
    bool classification_stable{false};

    bool at_crossing_position{false};
    bool post_check_passed{false};
    bool post_check_failed{false};

    uint8_t cruise_drive_request_raw{0};
    bool cruise_drive_request_valid{true};
    bool auto_start{false};
    bool manual_reset{false};
    bool obstacle_crossing_trigger{false};
    std::uint64_t auto_start_rising_edge_sequence{0};
    std::uint64_t manual_reset_rising_edge_sequence{0};
    std::uint64_t obstacle_trigger_rising_edge_sequence{0};
    std::uint64_t obstacle_trigger_falling_edge_sequence{0};

    uint8_t gravity_compensation_mode{0};
};

class AutoStateHandle
{
public:
    AutoStateHandle() = default;
    AutoStateHandle(const std::string& name, AutoStateData* data)
        : name_(name), data_(data) {}

    std::string getName() const { return name_; }
    const AutoStateData& getData() const { return *data_; }
    bool valid() const { return data_ != nullptr; }
    void setGravityCompensationMode(uint8_t mode) const
    {
        if (data_)
        {
            data_->gravity_compensation_mode = mode;
        }
    }

private:
    std::string name_;
    AutoStateData* data_{nullptr};
};

class AutoStateInterface : public hardware_interface::HardwareResourceManager<AutoStateHandle>
{
};

}   // namespace steering_engine_hw
