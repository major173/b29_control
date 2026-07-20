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

    bool obstacle_detected{false};
    ObstacleType obstacle_type{ObstacleType::OBSTACLE_UNKNOWN};
    bool classification_stable{false};
    double range_to_obstacle{0.0};
    
    bool at_crossing_position{false};
    bool post_check_passed{false};
    bool post_check_failed{false};

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
