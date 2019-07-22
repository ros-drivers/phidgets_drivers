#include <functional>
#include <memory>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_input.h"
#include "phidgets_api/digital_inputs.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

DigitalInputs::DigitalInputs(int32_t serial_number, int hub_port,
                             bool is_hub_port_device,
                             std::function<void(int, int)> input_handler)
{
    PhidgetReturnCode ret;

    PhidgetDigitalInputHandle di_handle;

    ret = PhidgetDigitalInput_create(&di_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalInput handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(di_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_DIGITALINPUT,
                                        &input_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get DigitalInput device channel count",
                             ret);
    }

    dis_.resize(input_count_);
    for (uint32_t i = 0; i < input_count_; ++i)
    {
        dis_[i] = std::make_unique<DigitalInput>(
            serial_number, hub_port, is_hub_port_device, i, input_handler);
    }
}

DigitalInputs::~DigitalInputs()
{
}

uint32_t DigitalInputs::getInputCount() const noexcept
{
    return input_count_;
}

bool DigitalInputs::getInputValue(int index) const
{
    return dis_.at(index)->getInputValue();
}

}  // namespace phidgets
