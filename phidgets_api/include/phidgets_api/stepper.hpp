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

#ifndef PHIDGETS_API_STEPPER_H
#define PHIDGETS_API_STEPPER_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.hpp"

namespace phidgets {

class Stepper final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Stepper)

    explicit Stepper(int32_t serial_number, int hub_port,
                     bool is_hub_port_device, int channel,
                     std::function<void(int, double)> position_change_handler,
                     std::function<void(int, double)> velocity_change_handler);

    ~Stepper();

    int32_t getSerialNumber() const noexcept;

    double getAcceleration() const;
    void setAcceleration(double acceleration) const;
    double getMinAcceleration() const;
    double getMaxAcceleration() const;
    bool stepControlMode() const;
    void setStepControlMode(bool step_control_mode) const;
    double getCurrentLimit() const;
    void setCurrentLimit(double current_limit) const;
    double getMinCurrentLimit() const;
    double getMaxCurrentLimit() const;
    uint32_t getDataInterval() const;
    void setDataInterval(uint32_t data_interval_ms) const;
    uint32_t getMinDataInterval() const;
    uint32_t getMaxDataInterval() const;
    bool getEngaged() const;
    void setEngaged(bool engaged) const;
    void enableFailsafe(uint32_t failsafe_time_ms) const;
    uint32_t getMinFailsafeTime() const;
    uint32_t getMaxFailsafeTime() const;
    double getHoldingCurrentLimit() const;
    void setHoldingCurrentLimit(double holding_current_limit) const;
    double getPosition() const;
    double getMinPosition() const;
    double getMaxPosition() const;
    void addPositionOffset(double position_offset) const;
    double getRescaleFactor() const;
    void setRescaleFactor(double rescale_factor) const;
    void resetFailsafe() const;
    double getTargetPosition() const;
    void setTargetPosition(double target_position) const;
    double getVelocity() const;
    double getVelocityLimit() const;
    void setVelocityLimit(double velocity_limit) const;
    double getMinVelocityLimit() const;
    double getMaxVelocityLimit() const;

    void positionChangeHandler(double position) const;
    void velocityChangeHandler(double velocity) const;

  private:
    int32_t serial_number_;
    int channel_;
    std::function<void(int, double)> position_change_handler_;
    std::function<void(int, double)> velocity_change_handler_;
    PhidgetStepperHandle stepper_handle_;

    static void PositionChangeHandler(PhidgetStepperHandle stepper_handle,
                                      void *ctx, double position);
    static void VelocityChangeHandler(PhidgetStepperHandle stepper_handle,
                                      void *ctx, double velocity);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_STEPPER_H
