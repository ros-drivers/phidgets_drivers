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
#include <memory>
#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/analog_input.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

AnalogInput::AnalogInput(int32_t serial_number, int hub_port,
                         bool is_hub_port_device, int channel,
                         std::function<void(int, double)> input_handler)
    : serial_number_(serial_number),
      channel_(channel),
      input_handler_(input_handler)
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

    if (!is_hub_port_device)
    {
        ret =
            PhidgetVoltageInput_setVoltageRange(ai_handle_, VOLTAGE_RANGE_AUTO);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to set analog input voltage range",
                                 ret);
        }
    }

    ret = PhidgetVoltageInput_setOnVoltageChangeHandler(
        ai_handle_, VoltageChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for AnalogInput channel " +
                std::to_string(channel),
            ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(ai_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to get serial number for analog input channel " +
                    std::to_string(channel),
                ret);
        }
    }
}

AnalogInput::~AnalogInput()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(ai_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t AnalogInput::getSerialNumber() const noexcept
{
    return serial_number_;
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

void AnalogInput::setDataInterval(uint32_t data_interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetVoltageInput_setDataInterval(ai_handle_, data_interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set analog data interval", ret);
    }
}

void AnalogInput::voltageChangeHandler(double sensorValue) const
{
    input_handler_(channel_, sensorValue);
}

void AnalogInput::VoltageChangeHandler(
    PhidgetVoltageInputHandle /* input_handle */, void *ctx, double sensorValue)
{
    (reinterpret_cast<AnalogInput *>(ctx))->voltageChangeHandler(sensorValue);
}

}  // namespace phidgets
