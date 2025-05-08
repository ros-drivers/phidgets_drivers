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

#ifndef PHIDGETS_API_STEPPER_HPP
#define PHIDGETS_API_STEPPER_HPP

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.hpp"

namespace phidgets {

class Stepper final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Stepper)

    explicit Stepper(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        int channel,
        std::function<void(int, double)> on_position_change_handler,
        std::function<void(int, double)> on_velocity_change_handler,
        std::function<void(int)> on_stoppped_handler);

    ~Stepper();

    int32_t getSerialNumber() const noexcept;

    PhidgetStepper_ControlMode getControlMode() const;
    void setControlMode(PhidgetStepper_ControlMode controlMode);

    void enableFailsafe(uint32_t failsafeTime);
    void resetFailesafe();
    uint32_t getMinFailsafeTime() const;
    uint32_t getMaxFailsafeTime() const;

    int getIsMoving() const;
    int getEngaged() const;
    void setEngaged(int engaged);

    void addPositionOffset(double offset);
    double getPosition() const;
    double getMinPosition() const;
    double getMaxPosition() const;
    double getTargetPosition() const;
    void setTargetPosition(double position);

    double getVelocity() const;
    double getVelocityLimit() const;
    void setVelocityLimit(double velocity);
    double getMinVelocityLimit() const;
    double getMaxVelocityLimit() const;

    double getAcceleration() const;
    void setAcceleration(double acceleration);

    double getCurrentLimit() const;
    void setCurrentLimit(double current_limit);

    double getHoldingCurrentLimit() const;
    void setHoldingCurrentLimit(double holding_current_limit);

    double getMinAcceleration() const;
    double getMaxAcceleration() const;

    double getMinCurrentLimit() const;
    double getMaxCurrentLimit() const;

    void setRescaleFactor(double factor);
    double getRescaleFactor() const;

    void setDataInterval(uint32_t data_interval_ms);
    uint32_t getDataInterval() const;
    uint32_t getMinDataInterval() const;
    uint32_t getMaxDataInterval() const;

    void setDataRate(double data_rate);
    double getDataRate() const;
    double getMinDataRate() const;
    double getMaxDataRate() const;

  private:
    int32_t serial_number_;
    int channel_;
    PhidgetStepperHandle stepper_handle_{nullptr};
    std::function<void(int, double)> on_position_change_handler_;
    std::function<void(int, double)> on_velocity_change_handler_;
    std::function<void(int)> on_stoppped_handler_;

    void positionChangeHandler(double position);
    void velocityChangeHandler(double velocity);
    void stoppedHandler();

    static void onPositionChangeHandler(PhidgetStepperHandle motor_handle,
                                        void *ctx, double position);
    static void onVelocityChangeHandler(PhidgetStepperHandle motor_handle,
                                        void *ctx, double velocity);
    static void onStoppedHandler(PhidgetStepperHandle motor_handle, void *ctx);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_STEPPER_HPP
