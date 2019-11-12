/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <functional>
#include <stdexcept>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.hpp"
#include "phidgets_api/temperature.hpp"

namespace phidgets {

Temperature::Temperature(int32_t serial_number, int hub_port,
                         bool is_hub_port_device,
                         std::function<void(double)> temperature_handler)
    : serial_number_(serial_number), temperature_handler_(temperature_handler)
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

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(temperature_handle_),
            &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to get serial number for temperature",
                                 ret);
        }
    }
}

Temperature::~Temperature()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(temperature_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t Temperature::getSerialNumber() const noexcept
{
    return serial_number_;
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
    (reinterpret_cast<Temperature *>(ctx))
        ->temperatureChangeHandler(temperature);
}

}  // namespace phidgets
