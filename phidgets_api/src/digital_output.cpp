#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_output.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

DigitalOutput::DigitalOutput(int32_t serial_number, int hub_port,
                             bool is_hub_port_device, int channel)
{
    PhidgetReturnCode ret;

    ret = PhidgetDigitalOutput_create(&do_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalOutput handle for channel " +
                std::to_string(channel),
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(do_handle_),
                                   serial_number, hub_port, is_hub_port_device,
                                   channel);
}

DigitalOutput::~DigitalOutput()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(do_handle_);
    helpers::closeAndDelete(&handle);
}

void DigitalOutput::setOutputState(bool state) const
{
    PhidgetReturnCode ret = PhidgetDigitalOutput_setState(do_handle_, state);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set start for DigitalOutput", ret);
    }
}

}  // namespace phidgets
