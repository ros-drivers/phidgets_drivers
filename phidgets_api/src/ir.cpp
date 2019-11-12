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

#include "phidgets_api/ir.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

IR::IR(int32_t serial_number,
       std::function<void(const char *, uint32_t, int)> code_handler)
    : serial_number_(serial_number), code_handler_(code_handler)
{
    // create the handle
    PhidgetReturnCode ret = PhidgetIR_create(&ir_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create IR handle", ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(ir_handle_),
                                   serial_number, 0, false, 0);

    // register ir data callback
    ret = PhidgetIR_setOnCodeHandler(ir_handle_, CodeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set code handler for ir", ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(ir_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to get serial number for IR", ret);
        }
    }
}

IR::~IR()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(ir_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t IR::getSerialNumber() const noexcept
{
    return serial_number_;
}

void IR::codeHandler(const char *code, uint32_t bit_count, int is_repeat) const
{
    code_handler_(code, bit_count, is_repeat);
}

void IR::CodeHandler(PhidgetIRHandle /* ir */, void *ctx, const char *code,
                     uint32_t bit_count, int is_repeat)
{
    (reinterpret_cast<IR *>(ctx))->codeHandler(code, bit_count, is_repeat);
}

}  // namespace phidgets
