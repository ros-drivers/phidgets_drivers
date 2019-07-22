#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/analog_input.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

AnalogInput::AnalogInput(int32_t serial_number, int hub_port,
                         bool is_hub_port_device, int channel,
                         std::function<void(int, double)> input_handler)
    : channel_(channel), input_handler_(input_handler)
{
    PhidgetReturnCode ret = PhidgetVoltageInput_create(&ai_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create AnalogInput handle for channel " +
                std::to_string(channel),
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(ai_handle_),
                                   serial_number, hub_port, is_hub_port_device,
                                   channel);

    ret = PhidgetVoltageInput_setOnVoltageChangeHandler(
        ai_handle_, VoltageChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for AnalogInput channel " +
                std::to_string(channel),
            ret);
    }
}

AnalogInput::~AnalogInput()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(ai_handle_);
    helpers::closeAndDelete(&handle);
}

double AnalogInput::getSensorValue() const
{
    double sensor_value;
    PhidgetReturnCode ret =
        PhidgetVoltageInput_getSensorValue(ai_handle_, &sensor_value);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get analog sensor value", ret);
    }

    return sensor_value;
}

void AnalogInput::voltageChangeHandler(double sensorValue) const
{
    input_handler_(channel_, sensorValue);
}

void AnalogInput::VoltageChangeHandler(
    PhidgetVoltageInputHandle /* input_handle */, void *ctx, double sensorValue)
{
    ((AnalogInput *)ctx)->voltageChangeHandler(sensorValue);
}

}  // namespace phidgets
