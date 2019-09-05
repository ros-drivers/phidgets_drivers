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

#include <memory>
#include <stdexcept>

#include <libphidget22/phidget22.h>

#include "phidgets_api/digital_output.h"
#include "phidgets_api/digital_outputs.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

DigitalOutputs::DigitalOutputs(int32_t serial_number, int hub_port,
                               bool is_hub_port_device)
{
    PhidgetReturnCode ret;

    PhidgetDigitalOutputHandle do_handle;

    ret = PhidgetDigitalOutput_create(&do_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create DigitalOutput handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(do_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_DIGITALOUTPUT,
                                        &output_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get DigitalOutput device channel count",
                             ret);
    }

    dos_.resize(output_count_);
    for (uint32_t i = 0; i < output_count_; ++i)
    {
        dos_[i] = std::make_unique<DigitalOutput>(serial_number, hub_port,
                                                  is_hub_port_device, i);
    }
}

DigitalOutputs::~DigitalOutputs()
{
}

uint32_t DigitalOutputs::getOutputCount() const noexcept
{
    return output_count_;
}

void DigitalOutputs::setOutputState(int index, bool state) const
{
    dos_.at(index)->setOutputState(state);
}

}  // namespace phidgets
