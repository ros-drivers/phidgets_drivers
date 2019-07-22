#include <functional>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_input.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

DigitalInput::DigitalInput(int32_t serial_number, int hub_port,
                           bool is_hub_port_device, int channel,
                           std::function<void(int, int)> input_handler)
    : channel_(channel), input_handler_(input_handler)
{
    PhidgetReturnCode ret = PhidgetDigitalInput_create(&di_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalInput handle for channel " +
                std::to_string(channel),
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(di_handle_),
                                   serial_number, hub_port, is_hub_port_device,
                                   channel);

    ret = PhidgetDigitalInput_setOnStateChangeHandler(di_handle_,
                                                      StateChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for DigitalInput channel " +
                std::to_string(channel),
            ret);
    }
}

DigitalInput::~DigitalInput()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(di_handle_);
    helpers::closeAndDelete(&handle);
}

bool DigitalInput::getInputValue() const
{
    int state;
    PhidgetReturnCode ret = PhidgetDigitalInput_getState(di_handle_, &state);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get digital input state", ret);
    }

    return !!state;
}

void DigitalInput::stateChangeHandler(int state) const
{
    input_handler_(channel_, state);
}

void DigitalInput::StateChangeHandler(
    PhidgetDigitalInputHandle /* input_handle */, void *ctx, int state)
{
    ((DigitalInput *)ctx)->stateChangeHandler(state);
}

}  // namespace phidgets
