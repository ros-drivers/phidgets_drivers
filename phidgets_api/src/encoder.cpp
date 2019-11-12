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

#include <libphidget22/phidget22.h>

#include "phidgets_api/encoder.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Encoder::Encoder(
    int32_t serial_number, int hub_port, bool is_hub_port_device, int channel,
    std::function<void(int, int, double, int)> position_change_handler)
    : serial_number_(serial_number),
      channel_(channel),
      position_change_handler_(position_change_handler)
{
    // create the handle
    PhidgetReturnCode ret = PhidgetEncoder_create(&encoder_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Encoder handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(encoder_handle_), serial_number,
        hub_port, is_hub_port_device, channel);

    ret = PhidgetEncoder_setOnPositionChangeHandler(
        encoder_handle_, PositionChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for Encoder channel " +
                std::to_string(channel),
            ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(encoder_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to get serial number for encoder channel " +
                    std::to_string(channel),
                ret);
        }
    }
}

Encoder::~Encoder()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(encoder_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t Encoder::getSerialNumber() const noexcept
{
    return serial_number_;
}

int64_t Encoder::getPosition() const
{
    int64_t position;
    PhidgetReturnCode ret =
        PhidgetEncoder_getPosition(encoder_handle_, &position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get position for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }

    return position;
}

void Encoder::setPosition(int64_t position) const
{
    PhidgetReturnCode ret =
        PhidgetEncoder_setPosition(encoder_handle_, position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set position for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

int64_t Encoder::getIndexPosition() const
{
    int64_t position;
    PhidgetReturnCode ret =
        PhidgetEncoder_getIndexPosition(encoder_handle_, &position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to get index position for Encoder channel " +
                std::to_string(channel_),
            ret);
    }

    return position;
}

bool Encoder::getEnabled() const
{
    int enabled;
    PhidgetReturnCode ret =
        PhidgetEncoder_getEnabled(encoder_handle_, &enabled);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get enabled for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }

    return enabled == PTRUE;
}

void Encoder::setEnabled(bool enabled) const
{
    PhidgetReturnCode ret =
        PhidgetEncoder_setEnabled(encoder_handle_, enabled ? PTRUE : PFALSE);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set enabled for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

void Encoder::positionChangeHandler(int position_change, double time,
                                    int index_triggered)
{
    position_change_handler_(channel_, position_change, time, index_triggered);
}

void Encoder::PositionChangeHandler(PhidgetEncoderHandle /* phid */, void *ctx,
                                    int position_change, double time,
                                    int index_triggered)
{
    (reinterpret_cast<Encoder *>(ctx))
        ->positionChangeHandler(position_change, time, index_triggered);
}

}  // namespace phidgets
