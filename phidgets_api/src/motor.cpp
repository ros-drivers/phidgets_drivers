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

#include "phidgets_api/motor.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Motor::Motor(const ChannelAddress &channel_address,
             std::function<void(int, double)> duty_cycle_change_handler,
             std::function<void(int, double)> back_emf_change_handler)
    : PhidgetChannel(channel_address),
      duty_cycle_change_handler_(duty_cycle_change_handler),
      back_emf_change_handler_(back_emf_change_handler)
{
    PhidgetReturnCode ret = PhidgetDCMotor_create(&motor_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Motor handle for channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(motor_handle_), channel_address_);

    ret = PhidgetDCMotor_setOnVelocityUpdateHandler(
        motor_handle_, DutyCycleChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set duty cycle update handler for Motor channel " +
                std::to_string(channel_address_.channel),
            ret);
    }

    back_emf_sensing_supported_ = true;
    ret = PhidgetDCMotor_setBackEMFSensingState(motor_handle_, 1);
    if (ret == EPHIDGET_UNSUPPORTED)
    {
        back_emf_sensing_supported_ = false;
    } else if (ret == EPHIDGET_OK)
    {
        ret = PhidgetDCMotor_setOnBackEMFChangeHandler(
            motor_handle_, BackEMFChangeHandler, this);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to set back EMF update handler for Motor channel " +
                    std::to_string(channel_address_.channel),
                ret);
        }
    } else
    {
        throw Phidget22Error(
            "Failed to set back EMF sensing state Motor channel " +
                std::to_string(channel_address_.channel),
            ret);
    }

    updateSerialNumber(reinterpret_cast<PhidgetHandle>(motor_handle_));
}

Motor::~Motor()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(motor_handle_);
    helpers::closeAndDelete(&handle);
}

double Motor::getDutyCycle() const
{
    double duty_cycle;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getVelocity(motor_handle_, &duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get duty cycle for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
    return duty_cycle;
}

void Motor::setDutyCycle(double duty_cycle) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setTargetVelocity(motor_handle_, duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set duty cycle for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
}

double Motor::getAcceleration() const
{
    double accel;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getAcceleration(motor_handle_, &accel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
    return accel;
}

void Motor::setAcceleration(double acceleration) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setAcceleration(motor_handle_, acceleration);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set acceleration for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
}

bool Motor::backEMFSensingSupported() const
{
    return back_emf_sensing_supported_;
}

double Motor::getBackEMF() const
{
    if (!back_emf_sensing_supported_)
    {
        throw Phidget22Error("Back EMF sensing not supported",
                             EPHIDGET_UNSUPPORTED);
    }
    double backemf;
    PhidgetReturnCode ret = PhidgetDCMotor_getBackEMF(motor_handle_, &backemf);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get back EMF for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
    return backemf;
}

void Motor::setDataInterval(uint32_t data_interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setDataInterval(motor_handle_, data_interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval for Motor channel " +
                                 std::to_string(channel_address_.channel),
                             ret);
    }
}

double Motor::getBraking() const
{
    double braking;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getBrakingStrength(motor_handle_, &braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to get braking strength for Motor channel " +
                std::to_string(channel_address_.channel),
            ret);
    }
    return braking;
}

void Motor::setBraking(double braking) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setTargetBrakingStrength(motor_handle_, braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set braking strength for Motor channel " +
                std::to_string(channel_address_.channel),
            ret);
    }
}

void Motor::dutyCycleChangeHandler(double duty_cycle) const
{
    duty_cycle_change_handler_(channel_address_.channel, duty_cycle);
}

void Motor::backEMFChangeHandler(double back_emf) const
{
    back_emf_change_handler_(channel_address_.channel, back_emf);
}

void Motor::DutyCycleChangeHandler(PhidgetDCMotorHandle /* motor_handle */,
                                   void *ctx, double duty_cycle)
{
    ((Motor *)ctx)->dutyCycleChangeHandler(duty_cycle);
}

void Motor::BackEMFChangeHandler(PhidgetDCMotorHandle /* motor_handle */,
                                 void *ctx, double back_emf)
{
    ((Motor *)ctx)->backEMFChangeHandler(back_emf);
}

}  // namespace phidgets
