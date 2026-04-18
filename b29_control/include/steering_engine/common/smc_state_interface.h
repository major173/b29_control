#ifndef _SMC_STATE_INTERFACE_H_
#define _SMC_STATE_INTERFACE_H_

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace steering_engine_hw{

struct SmcStateData
{   
    bool lower_alive{false};
    bool imu_ready{false};
    bool grip_confirmed{false};
    bool joint_fault{false};
    bool grip_fault{false};
};

class SmcStateHandle
{
public:
    SmcStateHandle() = default;
    SmcStateHandle(const std::string& name, const SmcStateData* data)
        : name_(name), data_(data) {}

    std::string getName() const { return name_; }
    const SmcStateData& getData() const { return *data_; }

private:
    std::string name_;
    const SmcStateData* data_{nullptr};
};

class SmcStateInterface : public hardware_interface::HardwareResourceManager<SmcStateHandle>
{
};

}   // namespace steering_engine_hw


#endif  // _SMC_STATE_INTERFACE_H_