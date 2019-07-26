#include <functional>
#include <stdexcept>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"
#include "phidgets_api/temperature.h"

namespace phidgets {

Temperature::Temperature(int32_t serial_number, int hub_port,
                         bool is_hub_port_device,
                         std::function<void(double)> temperature_handler)
    : temperature_handler_(temperature_handler)
{
    PhidgetReturnCode ret =
        PhidgetTemperatureSensor_create(&temperature_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create TemperatureSensor handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(temperature_handle_), serial_number,
        hub_port, is_hub_port_device, 0);

    ret = PhidgetTemperatureSensor_setOnTemperatureChangeHandler(
        temperature_handle_, TemperatureChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for Temperature",
                             ret);
    }
}

Temperature::~Temperature()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(temperature_handle_);
    helpers::closeAndDelete(&handle);
}

void Temperature::setThermocoupleType(ThermocoupleType type)
{
    PhidgetReturnCode ret = PhidgetTemperatureSensor_setThermocoupleType(
        temperature_handle_,
        static_cast<PhidgetTemperatureSensor_ThermocoupleType>(type));
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set Temperature thermocouple type",
                             ret);
    }
}

double Temperature::getTemperature() const
{
    double current_temperature;
    PhidgetReturnCode ret = PhidgetTemperatureSensor_getTemperature(
        temperature_handle_, &current_temperature);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get temperature", ret);
    }
    return current_temperature;
}

void Temperature::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret = PhidgetTemperatureSensor_setDataInterval(
        temperature_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Temperature::temperatureChangeHandler(double temperature) const
{
    temperature_handler_(temperature);
}

void Temperature::TemperatureChangeHandler(
    PhidgetTemperatureSensorHandle /* temperature_handle */, void *ctx,
    double temperature)
{
    ((Temperature *)ctx)->temperatureChangeHandler(temperature);
}

}  // namespace phidgets
