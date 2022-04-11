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
#include <vector>

#include "phidgets_api/encoder.h"
#include "phidgets_api/encoders.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Encoders::Encoders(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(int, int, double, int)> position_change_handler)
{
    PhidgetReturnCode ret;

    PhidgetEncoderHandle enc_handle;

    ret = PhidgetEncoder_create(&enc_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create Encoder handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(enc_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_ENCODER,
                                        &encoder_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get Encoder device channel count", ret);
    }

    encs_.resize(encoder_count_);
    for (uint32_t i = 0; i < encoder_count_; ++i)
    {
        encs_[i] = std::make_unique<Encoder>(serial_number, hub_port,
                                             is_hub_port_device, i,
                                             position_change_handler);
    }
}

Encoders::~Encoders()
{
}

uint32_t Encoders::getEncoderCount() const
{
    return encoder_count_;
}

int64_t Encoders::getPosition(int index) const
{
    return encs_.at(index)->getPosition();
}

void Encoders::setPosition(int index, int64_t position) const
{
    return encs_.at(index)->setPosition(position);
}

int64_t Encoders::getIndexPosition(int index) const
{
    return encs_.at(index)->getIndexPosition();
}

bool Encoders::getEnabled(int index) const
{
    return encs_.at(index)->getEnabled();
}

void Encoders::setEnabled(int index, bool enabled) const
{
    return encs_.at(index)->setEnabled(enabled);
}

Phidget_EncoderIOMode Encoders::getIOMode(int index) const
{
    return encs_.at(index)->getIOMode();
}

void Encoders::setIOMode(int index, Phidget_EncoderIOMode IOMode) const
{
    return encs_.at(index)->setIOMode(IOMode);
}

uint32_t Encoders::getDataInterval(int index) const
{
    return encs_.at(index)->getDataInterval();
}

void Encoders::setDataInterval(int index, uint32_t data_interval_ms) const
{
    return encs_.at(index)->setDataInterval(data_interval_ms);
}

}  // namespace phidgets
