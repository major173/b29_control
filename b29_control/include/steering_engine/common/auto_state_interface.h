#pragma once

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

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

    bool obstacle_detected{false};
    ObstacleType obstacle_type{ObstacleType::OBSTACLE_UNKNOWN};
    bool classification_stable{false};
    double range_to_obstacle{0.0};
    
    bool at_crossing_position{false};
    bool post_check_passed{false};
    bool post_check_failed{false};
};

class AutoStateHandle
{
public:
    AutoStateHandle() = default;
    AutoStateHandle(const std::string& name, const AutoStateData* data)
        : name_(name), data_(data) {}

    std::string getName() const { return name_; }
    const AutoStateData& getData() const { return *data_; }

private:
    std::string name_;
    const AutoStateData* data_{nullptr};
};

class AutoStateInterface : public hardware_interface::HardwareResourceManager<AutoStateHandle>
{
};

}   // namespace steering_engine_hw