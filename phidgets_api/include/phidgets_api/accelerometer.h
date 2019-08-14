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

#ifndef PHIDGETS_API_ACCELEROMETER_H
#define PHIDGETS_API_ACCELEROMETER_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Accelerometer final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Accelerometer)

    explicit Accelerometer(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        std::function<void(const double[3], double)> data_handler);

    ~Accelerometer();

    void getAcceleration(double &x, double &y, double &z,
                         double &timestamp) const;

    void setDataInterval(uint32_t interval_ms) const;

    void dataHandler(const double acceleration[3], double timestamp) const;

  private:
    std::function<void(const double[3], double)> data_handler_;
    PhidgetAccelerometerHandle accel_handle_;

    static void DataHandler(PhidgetAccelerometerHandle input_handle, void *ctx,
                            const double acceleration[3], double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ACCELEROMETER_H
