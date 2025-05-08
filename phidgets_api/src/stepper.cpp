/*
 * Copyright (c) 2025, Open Source Robotics Foundation
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
#include <cassert>

#include <libphidget22/phidget22.h>

#include "phidgets_api/stepper.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Stepper::Stepper(int32_t serial_number, int hub_port, bool is_hub_port_device,
                 int channel,
                 std::function<void(int, double)> on_position_change_handler,
                 std::function<void(int, double)> on_velocity_change_handler,
                 std::function<void(int)> on_stoppped_handler)
    : serial_number_(serial_number),
      channel_(channel),
      on_position_change_handler_(on_position_change_handler),
      on_velocity_change_handler_(on_velocity_change_handler),
      on_stoppped_handler_(on_stoppped_handler)
{
    PhidgetReturnCode ret = PhidgetStepper_create(&stepper_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Stepper handle for channel " +
                                 std::to_string(channel),
                             ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(stepper_handle_), serial_number,
        hub_port, is_hub_port_device, channel);

    ret = PhidgetStepper_setOnVelocityChangeHandler(
        stepper_handle_, onVelocityChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set velocity handler for Stepper channel " +
                std::to_string(channel),
            ret);
    }

    ret = PhidgetStepper_setOnPositionChangeHandler(
        stepper_handle_, onPositionChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set position handler for Stepper channel " +
                std::to_string(channel),
            ret);
    }

    ret = PhidgetStepper_setOnStoppedHandler(stepper_handle_, onStoppedHandler,
                                             this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set stopped handler for Stepper channel " +
                std::to_string(channel),
            ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(stepper_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to get serial number for Stepper channel " +
                    std::to_string(channel),
                ret);
        }
    }
}

Stepper::~Stepper()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(stepper_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t Stepper::getSerialNumber() const noexcept
{
    return serial_number_;
}

#define GETTER(T, F)                                                       \
    T Stepper::F() const                                                   \
    {                                                                      \
        T val;                                                             \
        PhidgetReturnCode ret = PhidgetStepper_##F(stepper_handle_, &val); \
        if (ret != EPHIDGET_OK)                                            \
        {                                                                  \
            throw Phidget22Error("Stepper::" #F                            \
                                 " failed for Stepper channel " +          \
                                     std::to_string(channel_),             \
                                 ret);                                     \
        }                                                                  \
        return val;                                                        \
    }

#define SETTER(T, F)                                                        \
    void Stepper::F(T value)                                                \
    {                                                                       \
        PhidgetReturnCode ret = PhidgetStepper_##F(stepper_handle_, value); \
        if (ret != EPHIDGET_OK)                                             \
        {                                                                   \
            throw Phidget22Error("Stepper::" #F                             \
                                 " failed for Stepper channel " +           \
                                     std::to_string(channel_),              \
                                 ret);                                      \
        }                                                                   \
    }

GETTER(PhidgetStepper_ControlMode, getControlMode);
SETTER(PhidgetStepper_ControlMode, setControlMode);
GETTER(double, getTargetPosition);
SETTER(double, setTargetPosition);

GETTER(double, getPosition);
SETTER(double, addPositionOffset);
GETTER(double, getMinPosition);
GETTER(double, getMaxPosition);

GETTER(double, getVelocity);
GETTER(double, getAcceleration);
SETTER(double, setAcceleration);
GETTER(double, getMinAcceleration);
GETTER(double, getMaxAcceleration);

GETTER(double, getVelocityLimit);
SETTER(double, setVelocityLimit);
GETTER(double, getMinVelocityLimit);
GETTER(double, getMaxVelocityLimit);

GETTER(double, getCurrentLimit);
SETTER(double, setCurrentLimit);
SETTER(double, setHoldingCurrentLimit);
GETTER(double, getHoldingCurrentLimit);
GETTER(double, getMinCurrentLimit);
GETTER(double, getMaxCurrentLimit);

SETTER(double, setRescaleFactor);
GETTER(double, getRescaleFactor);

SETTER(double, setDataRate);
GETTER(double, getDataRate);
GETTER(double, getMinDataRate);
GETTER(double, getMaxDataRate);

GETTER(uint32_t, getDataInterval);
SETTER(uint32_t, setDataInterval);
GETTER(uint32_t, getMinDataInterval);
GETTER(uint32_t, getMaxDataInterval);

SETTER(uint32_t, enableFailsafe);
GETTER(uint32_t, getMinFailsafeTime);
GETTER(uint32_t, getMaxFailsafeTime);

GETTER(int, getIsMoving);
SETTER(int, setEngaged);
GETTER(int, getEngaged);

void Stepper::resetFailesafe()
{
    PhidgetReturnCode ret = PhidgetStepper_resetFailsafe(stepper_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to reset failsafe for Stepper channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

void Stepper::positionChangeHandler(double position)
{
    if (this->on_position_change_handler_)
    {
        this->on_position_change_handler_(channel_, position);
    }
}

void Stepper::velocityChangeHandler(double velocity)
{
    if (this->on_velocity_change_handler_)
    {
        this->on_velocity_change_handler_(channel_, velocity);
    }
}

void Stepper::stoppedHandler()
{
    if (this->on_stoppped_handler_)
    {
        this->on_stoppped_handler_(channel_);
    }
}

void Stepper::onPositionChangeHandler(PhidgetStepperHandle /* motor_handle */,
                                      void *ctx, double position)
{
    Stepper *stepper = (reinterpret_cast<Stepper *>(ctx));
    assert(stepper);
    stepper->positionChangeHandler(position);
}

void Stepper::onVelocityChangeHandler(PhidgetStepperHandle /* motor_handle */,
                                      void *ctx, double velocity)
{
    Stepper *stepper = (reinterpret_cast<Stepper *>(ctx));
    assert(stepper);
    stepper->velocityChangeHandler(velocity);
}

void Stepper::onStoppedHandler(PhidgetStepperHandle /* motor_handle */,
                               void *ctx)
{
    Stepper *stepper = (reinterpret_cast<Stepper *>(ctx));
    assert(stepper);
    stepper->stoppedHandler();
}

}  // namespace phidgets
