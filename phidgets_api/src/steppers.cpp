/*
 * Copyright (c) 2019, Howard Hughes Medical Institute
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

#include "phidgets_api/steppers.hpp"

namespace phidgets {

Steppers::Steppers(int32_t serial_number, int hub_port, bool is_hub_port_device,
                   std::function<void(int, double)> position_change_handler,
                   std::function<void(int, double)> velocity_change_handler)
{
    PhidgetReturnCode ret;

    PhidgetStepperHandle stepper_handle;

    ret = PhidgetStepper_create(&stepper_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create Stepper handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(stepper_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_STEPPER,
                                        &stepper_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get Stepper device channel count", ret);
    }

    steppers_.resize(stepper_count_);
    for (uint32_t i = 0; i < stepper_count_; ++i)
    {
        steppers_[i] = std::make_unique<Stepper>(
            serial_number, hub_port, is_hub_port_device, i,
            position_change_handler, velocity_change_handler);
    }
}

Steppers::~Steppers()
{
}

int32_t Steppers::getSerialNumber() const noexcept
{
    return steppers_.at(0)->getSerialNumber();
}

uint32_t Steppers::getStepperCount() const noexcept
{
    return stepper_count_;
}

double Steppers::getAcceleration(int channel) const
{
    return steppers_.at(channel)->getAcceleration();
}

void Steppers::setAcceleration(int channel, double acceleration) const
{
    steppers_.at(channel)->setAcceleration(acceleration);
}

double Steppers::getMinAcceleration(int channel) const
{
    return steppers_.at(channel)->getMinAcceleration();
}

double Steppers::getMaxAcceleration(int channel) const
{
    return steppers_.at(channel)->getMaxAcceleration();
}

bool Steppers::stepControlMode(int channel) const
{
    return steppers_.at(channel)->stepControlMode();
}

void Steppers::setStepControlMode(int channel, bool step_control_mode) const
{
    steppers_.at(channel)->setStepControlMode(step_control_mode);
}

double Steppers::getCurrentLimit(int channel) const
{
    return steppers_.at(channel)->getCurrentLimit();
}

void Steppers::setCurrentLimit(int channel, double current_limit) const
{
    steppers_.at(channel)->setCurrentLimit(current_limit);
}

double Steppers::getMinCurrentLimit(int channel) const
{
    return steppers_.at(channel)->getMinCurrentLimit();
}

double Steppers::getMaxCurrentLimit(int channel) const
{
    return steppers_.at(channel)->getMaxCurrentLimit();
}

uint32_t Steppers::getDataInterval(int channel) const
{
    return steppers_.at(channel)->getDataInterval();
}

void Steppers::setDataInterval(int channel, uint32_t data_interval_ms) const
{
    steppers_.at(channel)->setDataInterval(data_interval_ms);
}

uint32_t Steppers::getMinDataInterval(int channel) const
{
    return steppers_.at(channel)->getMinDataInterval();
}

uint32_t Steppers::getMaxDataInterval(int channel) const
{
    return steppers_.at(channel)->getMaxDataInterval();
}

bool Steppers::getEngaged(int channel) const
{
    return steppers_.at(channel)->getEngaged();
}

void Steppers::setEngaged(int channel, bool engaged) const
{
    steppers_.at(channel)->setEngaged(engaged);
}

void Steppers::enableFailsafe(int channel, uint32_t failsafe_time_ms) const
{
    steppers_.at(channel)->enableFailsafe(failsafe_time_ms);
}

uint32_t Steppers::getMinFailsafeTime(int channel) const
{
    return steppers_.at(channel)->getMinFailsafeTime();
}

uint32_t Steppers::getMaxFailsafeTime(int channel) const
{
    return steppers_.at(channel)->getMaxFailsafeTime();
}

double Steppers::getHoldingCurrentLimit(int channel) const
{
    return steppers_.at(channel)->getHoldingCurrentLimit();
}

void Steppers::setHoldingCurrentLimit(int channel,
                                      double holding_current_limit) const
{
    steppers_.at(channel)->setHoldingCurrentLimit(holding_current_limit);
}

double Steppers::getPosition(int channel) const
{
    return steppers_.at(channel)->getPosition();
}

double Steppers::getMinPosition(int channel) const
{
    return steppers_.at(channel)->getMinPosition();
}

double Steppers::getMaxPosition(int channel) const
{
    return steppers_.at(channel)->getMaxPosition();
}

void Steppers::addPositionOffset(int channel, double position_offset) const
{
    steppers_.at(channel)->addPositionOffset(position_offset);
}

double Steppers::getRescaleFactor(int channel) const
{
    return steppers_.at(channel)->getRescaleFactor();
}

void Steppers::setRescaleFactor(int channel, double rescale_factor) const
{
    steppers_.at(channel)->setRescaleFactor(rescale_factor);
}

void Steppers::resetFailsafe(int channel) const
{
    steppers_.at(channel)->resetFailsafe();
}

double Steppers::getTargetPosition(int channel) const
{
    return steppers_.at(channel)->getTargetPosition();
}

void Steppers::setTargetPosition(int channel, double target_position) const
{
    steppers_.at(channel)->setTargetPosition(target_position);
}

double Steppers::getVelocity(int channel) const
{
    return steppers_.at(channel)->getVelocity();
}

double Steppers::getVelocityLimit(int channel) const
{
    return steppers_.at(channel)->getVelocityLimit();
}

void Steppers::setVelocityLimit(int channel, double velocity_limit) const
{
    steppers_.at(channel)->setVelocityLimit(velocity_limit);
}

double Steppers::getMinVelocityLimit(int channel) const
{
    return steppers_.at(channel)->getMinVelocityLimit();
}

double Steppers::getMaxVelocityLimit(int channel) const
{
    return steppers_.at(channel)->getMaxVelocityLimit();
}

void Steppers::positionChangeHandler(int channel, double position) const
{
    steppers_.at(channel)->positionChangeHandler(position);
}

void Steppers::velocityChangeHandler(int channel, double velocity) const
{
    steppers_.at(channel)->velocityChangeHandler(velocity);
}

}  // namespace phidgets
