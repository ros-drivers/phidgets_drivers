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

#include "phidgets_api/accelerometer.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Accelerometer::Accelerometer(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(const double[3], double)> data_handler)
    : serial_number_(serial_number), data_handler_(data_handler)
{
    PhidgetReturnCode ret = PhidgetAccelerometer_create(&accel_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Accelerometer handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(accel_handle_), serial_number, hub_port,
        is_hub_port_device, 0);

    ret = PhidgetAccelerometer_setOnAccelerationChangeHandler(
        accel_handle_, DataHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for acceleration",
                             ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(accel_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to get serial number for acceleration",
                                 ret);
        }
    }
}

Accelerometer::~Accelerometer()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(accel_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t Accelerometer::getSerialNumber() const noexcept
{
    return serial_number_;
}

void Accelerometer::getAcceleration(double &x, double &y, double &z,
                                    double &timestamp) const
{
    double accel[3];
    PhidgetReturnCode ret =
        PhidgetAccelerometer_getAcceleration(accel_handle_, &accel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration", ret);
    }

    x = accel[0];
    y = accel[1];
    z = accel[2];

    double ts;
    ret = PhidgetAccelerometer_getTimestamp(accel_handle_, &ts);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration timestamp", ret);
    }

    timestamp = ts;
}

void Accelerometer::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetAccelerometer_setDataInterval(accel_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Accelerometer::dataHandler(const double acceleration[3],
                                double timestamp) const
{
    data_handler_(acceleration, timestamp);
}

void Accelerometer::DataHandler(PhidgetAccelerometerHandle /* input_handle */,
                                void *ctx, const double acceleration[3],
                                double timestamp)
{
    (reinterpret_cast<Accelerometer *>(ctx))
        ->dataHandler(acceleration, timestamp);
}

}  // namespace phidgets
