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
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_input.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

DigitalInput::DigitalInput(const ChannelAddress &channel_address,
                           std::function<void(int, int)> input_handler)
    : channel_address_(channel_address), input_handler_(input_handler)
{
    PhidgetReturnCode ret = PhidgetDigitalInput_create(&di_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalInput handle for channel " +
                std::to_string(channel_address_.channel),
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(di_handle_),
                                   channel_address_);

    ret = PhidgetDigitalInput_setOnStateChangeHandler(di_handle_,
                                                      StateChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for DigitalInput channel " +
                std::to_string(channel_address_.channel),
            ret);
    }

    if (channel_address_.serial_number == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(di_handle_),
            &channel_address_.serial_number);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to get serial number for digital input channel " +
                    std::to_string(channel_address_.channel),
                ret);
        }
    }
}

DigitalInput::~DigitalInput()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(di_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t DigitalInput::getSerialNumber() const noexcept
{
    return channel_address_.serial_number;
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
    input_handler_(channel_address_.channel, state);
}

void DigitalInput::StateChangeHandler(
    PhidgetDigitalInputHandle /* input_handle */, void *ctx, int state)
{
    ((DigitalInput *)ctx)->stateChangeHandler(state);
}

}  // namespace phidgets
