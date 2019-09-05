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

#include "phidgets_api/gyroscope.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Gyroscope::Gyroscope(int32_t serial_number, int hub_port,
                     bool is_hub_port_device,
                     std::function<void(const double[3], double)> data_handler)
    : data_handler_(data_handler)
{
    PhidgetReturnCode ret = PhidgetGyroscope_create(&gyro_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Gyroscope handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(gyro_handle_), serial_number, hub_port,
        is_hub_port_device, 0);

    ret = PhidgetGyroscope_setOnAngularRateUpdateHandler(gyro_handle_,
                                                         DataHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set Gyroscope update handler", ret);
    }
}

Gyroscope::~Gyroscope()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(gyro_handle_);
    helpers::closeAndDelete(&handle);
}

void Gyroscope::zero() const
{
    PhidgetReturnCode ret = PhidgetGyroscope_zero(gyro_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to calibrate Gyroscope", ret);
    }
}

void Gyroscope::getAngularRate(double &x, double &y, double &z,
                               double &timestamp) const
{
    double angular_rate[3];
    PhidgetReturnCode ret =
        PhidgetGyroscope_getAngularRate(gyro_handle_, &angular_rate);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get angular rate from gyroscope", ret);
    }

    x = angular_rate[0];
    y = angular_rate[1];
    z = angular_rate[2];

    double ts;
    ret = PhidgetGyroscope_getTimestamp(gyro_handle_, &ts);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get timestamp from gyroscope", ret);
    }

    timestamp = ts;
}

void Gyroscope::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetGyroscope_setDataInterval(gyro_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Gyroscope::dataHandler(const double angular_rate[3],
                            double timestamp) const
{
    data_handler_(angular_rate, timestamp);
}

void Gyroscope::DataHandler(PhidgetGyroscopeHandle /* input_handle */,
                            void *ctx, const double angular_rate[3],
                            double timestamp)
{
    ((Gyroscope *)ctx)->dataHandler(angular_rate, timestamp);
}

}  // namespace phidgets
