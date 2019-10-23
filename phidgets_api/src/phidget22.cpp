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

#include <string>

#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Phidget22Error::Phidget22Error(const std::string &msg, PhidgetReturnCode code)
    : std::exception()
{
    const char *error_ptr;
    PhidgetReturnCode ret = Phidget_getErrorDescription(code, &error_ptr);
    if (ret == EPHIDGET_OK)
    {
        msg_ = msg + ": " + std::string(error_ptr);
    } else
    {
        msg_ = msg + ": Unknown error";
    }
}

const char *Phidget22Error::what() const noexcept
{
    return msg_.c_str();
}

PhidgetChannel::PhidgetChannel(const ChannelAddress &channel_address)
    : channel_address_(channel_address)
{
}

int32_t PhidgetChannel::getSerialNumber() const noexcept
{
    return channel_address_.serial_number;
}

void PhidgetChannel::updateSerialNumber(PhidgetHandle handle)
{
    if (channel_address_.serial_number == -1)
    {
        PhidgetReturnCode ret = Phidget_getDeviceSerialNumber(handle,
                                            &channel_address_.serial_number);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to get serial number", ret);
        }
    }
}

namespace helpers {

void openWaitForAttachment(PhidgetHandle handle,
                           const ChannelAddress &channel_address)
{
    PhidgetReturnCode ret;

    ret = Phidget_setDeviceSerialNumber(handle, channel_address.serial_number);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device serial number", ret);
    }

    ret = Phidget_setHubPort(handle, channel_address.hub_port);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device hub port", ret);
    }

    ret =
        Phidget_setIsHubPortDevice(handle, channel_address.is_hub_port_device);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device is hub port device", ret);
    }

    ret = Phidget_setChannel(handle, channel_address.channel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device channel", ret);
    }

    ret = Phidget_openWaitForAttachment(handle, PHIDGET_TIMEOUT_DEFAULT);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to open device", ret);
    }
}

void closeAndDelete(PhidgetHandle *handle) noexcept
{
    Phidget_close(*handle);
    Phidget_delete(handle);
}

}  // namespace helpers
}  // namespace phidgets
