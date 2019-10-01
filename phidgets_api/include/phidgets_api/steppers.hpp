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

#ifndef PHIDGETS_API_STEPPERS_H
#define PHIDGETS_API_STEPPERS_H

#include <functional>
#include <memory>
#include <vector>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.hpp"
#include "phidgets_api/stepper.hpp"

namespace phidgets {

class Steppers final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Steppers)

    explicit Steppers(int32_t serial_number, int hub_port,
                      bool is_hub_port_device,
                      std::function<void(int, double)> position_change_handler,
                      std::function<void(int, double)> velocity_change_handler);

    ~Steppers();

    int32_t getSerialNumber() const noexcept;

    uint32_t getStepperCount() const noexcept;

    double getAcceleration(int channel) const;
    void setAcceleration(int channel, double acceleration) const;
    double getMinAcceleration(int channel) const;
    double getMaxAcceleration(int channel) const;
    bool stepControlMode(int channel) const;
    void setStepControlMode(int channel, bool step_control_mode) const;
    double getCurrentLimit(int channel) const;
    void setCurrentLimit(int channel, double current_limit) const;
    double getMinCurrentLimit(int channel) const;
    double getMaxCurrentLimit(int channel) const;
    uint32_t getDataInterval(int channel) const;
    void setDataInterval(int channel, uint32_t data_interval_ms) const;
    uint32_t getMinDataInterval(int channel) const;
    uint32_t getMaxDataInterval(int channel) const;
    bool getEngaged(int channel) const;
    void setEngaged(int channel, bool engaged) const;
    void enableFailsafe(int channel, uint32_t failsafe_time_ms) const;
    uint32_t getMinFailsafeTime(int channel) const;
    uint32_t getMaxFailsafeTime(int channel) const;
    double getHoldingCurrentLimit(int channel) const;
    void setHoldingCurrentLimit(int channel,
                                double holding_current_limit) const;
    double getPosition(int channel) const;
    double getMinPosition(int channel) const;
    double getMaxPosition(int channel) const;
    void addPositionOffset(int channel, double position_offset) const;
    double getRescaleFactor(int channel) const;
    void setRescaleFactor(int channel, double rescale_factor) const;
    void resetFailsafe(int channel) const;
    double getTargetPosition(int channel) const;
    void setTargetPosition(int channel, double target_position) const;
    double getVelocity(int channel) const;
    double getVelocityLimit(int channel) const;
    void setVelocityLimit(int channel, double velocity_limit) const;
    double getMinVelocityLimit(int channel) const;
    double getMaxVelocityLimit(int channel) const;

    void positionChangeHandler(int channel, double position) const;
    void velocityChangeHandler(int channel, double velocity) const;

  private:
    uint32_t stepper_count_;
    std::vector<std::unique_ptr<Stepper>> steppers_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_STEPPERS_H
