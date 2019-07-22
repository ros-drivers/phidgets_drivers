#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/analog_input.h"
#include "phidgets_api/analog_inputs.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

AnalogInputs::AnalogInputs(int32_t serial_number, int hub_port,
                           bool is_hub_port_device,
                           std::function<void(int, double)> input_handler)
{
    PhidgetReturnCode ret;

    PhidgetVoltageInputHandle ai_handle;

    ret = PhidgetVoltageInput_create(&ai_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create AnalogInput handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(ai_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_VOLTAGEINPUT,
                                        &input_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get AnalogInput device channel count",
                             ret);
    }

    ais_.resize(input_count_);
    for (uint32_t i = 0; i < input_count_; ++i)
    {
        ais_[i] = std::make_unique<AnalogInput>(
            serial_number, hub_port, is_hub_port_device, i, input_handler);
    }
}

AnalogInputs::~AnalogInputs()
{
}

uint32_t AnalogInputs::getInputCount() const noexcept
{
    return input_count_;
}

double AnalogInputs::getSensorValue(int index) const
{
    return ais_.at(index)->getSensorValue();
}

}  // namespace phidgets
