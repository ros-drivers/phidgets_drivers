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

#include "phidgets_api/phidget22.h"
#include "phidgets_api/humidity.h"

namespace phidgets {

Humidity::Humidity(int32_t serial_number, int hub_port, bool is_hub_port_device,
                   std::function<void(double)> humidity_handler)
    : humidity_handler_(humidity_handler)
{
    PhidgetReturnCode ret = PhidgetHumiditySensor_create(&humidity_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create HumiditySensor handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(humidity_handle_), serial_number,
        hub_port, is_hub_port_device, 0);

    ret = PhidgetHumiditySensor_setOnHumidityChangeHandler(
        humidity_handle_, HumidityChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for Humidity", ret);
    }
}

Humidity::~Humidity()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(humidity_handle_);
    helpers::closeAndDelete(&handle);
}

double Humidity::getHumidity() const
{
    double current_humidity;
    PhidgetReturnCode ret =
        PhidgetHumiditySensor_getHumidity(humidity_handle_, &current_humidity);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get humidity", ret);
    }
    return current_humidity;
}

void Humidity::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetHumiditySensor_setDataInterval(humidity_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Humidity::humidityChangeHandler(double humidity) const
{
    humidity_handler_(humidity);
}

void Humidity::HumidityChangeHandler(
    PhidgetHumiditySensorHandle /* humidity_handle */, void *ctx,
    double humidity)
{
    ((Humidity *)ctx)->humidityChangeHandler(humidity);
}

}  // namespace phidgets
