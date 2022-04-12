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

#ifndef PHIDGETS_API_ENCODERS_H
#define PHIDGETS_API_ENCODERS_H

#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/encoder.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class Encoders final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Encoders)

    explicit Encoders(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        std::function<void(int, int, double, int)> position_change_handler);

    ~Encoders();

    /**@brief Gets the number of encoder input channels supported by this board
     */
    uint32_t getEncoderCount() const;

    /** @brief Reads the current position of an encoder
     * @param index The index of the encoder to read */
    int64_t getPosition(int index) const;

    /** @brief Sets the offset of an encoder such that current position is the
     * specified value
     * @param index The index of the encoder to set
     * @param position The new value that should be returned by
     * 'getPosition(index)' at the current position of the encoder*/
    void setPosition(int index, int64_t position) const;

    /** @brief Gets the position of an encoder the last time an index pulse
     * occured. An index pulse in this context refers to an input from the
     * encoder the pulses high once every revolution.
     * @param index The index of the encoder to read */
    int64_t getIndexPosition(int index) const;

    /** @brief Checks if an encoder is powered on and receiving events
     * @param index The index of the encoder to check */
    bool getEnabled(int index) const;

    /** @brief Set the powered state of an encoder. If an encoder is not
     * enabled, it will not be given power, and events and changes in position
     * will not be captured.
     * @param index The index of the encoder to change
     * @param enabled The new powered state of the encoder*/
    void setEnabled(int index, bool enabled) const;

    Phidget_EncoderIOMode getIOMode(int index) const;
    void setIOMode(int index, Phidget_EncoderIOMode io_mode) const;

    uint32_t getDataInterval(int index) const;
    void setDataInterval(int index, uint32_t data_interval_ms) const;

  private:
    uint32_t encoder_count_;
    std::vector<std::unique_ptr<Encoder>> encs_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ENCODERS_H
