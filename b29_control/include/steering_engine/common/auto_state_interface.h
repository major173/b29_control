
#define _SMC_STATE_INTERFACE_H_

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace steering_engine_hw {

    struct AutoStateData{
        bool lower_alive{false};
        bool imu_ready{false};
        bool grip_confirmed{false};
        bool joint_fault{false};
        bool grip_fault{false};
    };

    class AutoStateHandle{
        public:
           AutoStateHandle() = default;
           AutoStateHandle(const std::string& name, const AutoStateData* data)
           :name_(name), data_(data){}

           std::string getName() const { return name_; }
           const AutoStateData &getData() const { return *data_; }

        private:
            std::string name_;
            const AutoStateData* data_{nullptr};
    };

    class AutoStateInterface : public hardware_interface::HardwareResourceManager<AutoStateHandle>
    {
    };

}   // namespace steering_engine_hw

