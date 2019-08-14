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

#include "phidgets_api/magnetometer.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Magnetometer::Magnetometer(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(const double[3], double)> data_handler)
    : data_handler_(data_handler)
{
    PhidgetReturnCode ret = PhidgetMagnetometer_create(&mag_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Magnetometer handle", ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(mag_handle_),
                                   serial_number, hub_port, is_hub_port_device,
                                   0);

    ret = PhidgetMagnetometer_setOnMagneticFieldChangeHandler(
        mag_handle_, DataHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for Magnetometer",
                             ret);
    }
}

Magnetometer::~Magnetometer()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(mag_handle_);
    helpers::closeAndDelete(&handle);
}

void Magnetometer::setCompassCorrectionParameters(
    double cc_mag_field, double cc_offset0, double cc_offset1,
    double cc_offset2, double cc_gain0, double cc_gain1, double cc_gain2,
    double cc_T0, double cc_T1, double cc_T2, double cc_T3, double cc_T4,
    double cc_T5)
{
    PhidgetReturnCode ret = PhidgetMagnetometer_setCorrectionParameters(
        mag_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
        cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set magnetometer correction parameters",
                             ret);
    }
}

void Magnetometer::getMagneticField(double &x, double &y, double &z,
                                    double &timestamp) const
{
    double mag_field[3];
    PhidgetReturnCode ret =
        PhidgetMagnetometer_getMagneticField(mag_handle_, &mag_field);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get magnetic field", ret);
    }

    x = mag_field[0];
    y = mag_field[1];
    z = mag_field[2];

    double ts;
    ret = PhidgetMagnetometer_getTimestamp(mag_handle_, &ts);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get magnetic field timestamp", ret);
    }

    timestamp = ts;
}

void Magnetometer::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetMagnetometer_setDataInterval(mag_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Magnetometer::dataHandler(const double magnetic_field[3],
                               double timestamp) const
{
    data_handler_(magnetic_field, timestamp);
}

void Magnetometer::DataHandler(PhidgetMagnetometerHandle /* input_handle */,
                               void *ctx, const double magnetic_field[3],
                               double timestamp)
{
    ((Magnetometer *)ctx)->dataHandler(magnetic_field, timestamp);
}

}  // namespace phidgets
