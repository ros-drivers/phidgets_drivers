#include <memory>
#include <stdexcept>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_output.h"
#include "phidgets_api/digital_outputs.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

DigitalOutputs::DigitalOutputs(int32_t serial_number, int hub_port,
                               bool is_hub_port_device)
{
    PhidgetReturnCode ret;

    PhidgetDigitalOutputHandle do_handle;

    ret = PhidgetDigitalOutput_create(&do_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalOutput handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(do_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_DIGITALOUTPUT,
                                        &output_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get DigitalOutput device channel count",
                             ret);
    }

    dos_.resize(output_count_);
    for (uint32_t i = 0; i < output_count_; ++i)
    {
        dos_[i] = std::make_unique<DigitalOutput>(serial_number, hub_port,
                                                  is_hub_port_device, i);
    }
}

DigitalOutputs::~DigitalOutputs()
{
}

uint32_t DigitalOutputs::getOutputCount() const noexcept
{
    return output_count_;
}

void DigitalOutputs::setOutputState(int index, bool state) const
{
    dos_.at(index)->setOutputState(state);
}

}  // namespace phidgets
