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

#ifndef PHIDGETS_API_ENCODER_H
#define PHIDGETS_API_ENCODER_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Encoder final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Encoder)

    explicit Encoder(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        int channel,
        std::function<void(int, int, double, int)> position_change_handler);

    ~Encoder();

    /** @brief Reads the current position of an encoder
     */
    int64_t getPosition() const;

    /** @brief Sets the offset of an encoder such that current position is the
     * specified value
     * @param position The new value that should be returned by
     * 'getPosition(index)' at the current position of the encoder*/
    void setPosition(int64_t position) const;

    /** @brief Gets the position of an encoder the last time an index pulse
     * occured. An index pulse in this context refers to an input from the
     * encoder the pulses high once every revolution.
     */
    int64_t getIndexPosition() const;

    /** @brief Checks if an encoder is powered on and receiving events
     */
    bool getEnabled() const;

    /** @brief Set the powered state of an encoder. If an encoder is not
     * enabled, it will not be given power, and events and changes in position
     * will not be captured.
     * @param enabled The new powered state of the encoder*/
    void setEnabled(bool enabled) const;

    Phidget_EncoderIOMode getIOMode() const;
    void setIOMode(Phidget_EncoderIOMode io_mode) const;

    uint32_t getDataInterval() const;
    void setDataInterval(uint32_t data_interval_ms) const;

    void positionChangeHandler(int position_change, double time,
                               int index_triggered);

  private:
    int channel_;
    std::function<void(int, int, double, int)> position_change_handler_;
    PhidgetEncoderHandle encoder_handle_;

    static void PositionChangeHandler(PhidgetEncoderHandle phid, void *ctx,
                                      int position_change, double time_change,
                                      int index_triggered);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ENCODER_H
