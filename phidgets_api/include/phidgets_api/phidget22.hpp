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

#ifndef PHIDGETS_API_PHIDGET22_H
#define PHIDGETS_API_PHIDGET22_H

#include <stdexcept>
#include <string>
#include <functional>

#include <libphidget22/phidget22.h>

#define PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Classname) \
    Classname(const Classname &) = delete;             \
    void operator=(const Classname &) = delete;        \
    Classname(Classname &&) = delete;                  \
    void operator=(Classname &&) = delete;

namespace phidgets {

class Phidget22Error final : public std::exception
{
  public:
    explicit Phidget22Error(const std::string &msg, PhidgetReturnCode code);

    const char *what() const noexcept override;

  private:
    std::string msg_;
};

struct ChannelInfo
{
  int32_t serial_number{ PHIDGET_SERIALNUMBER_ANY };
  int hub_port{ PHIDGET_HUBPORT_ANY };
  bool is_hub_port_device{ false };
  int channel{ PHIDGET_CHANNEL_ANY };
};

class PhidgetComponent
{
public:
  PhidgetComponent(std::string name);
  virtual bool hasHandle() const override;
  virtual void setOnAttachHandler(std::function<void(PhidgetHandle, void *)> on_attach_handler) override;
private:
  std::string name_;
  void on_attach_handler();
};

class Phidget : public PhidgetComponent
{
};

class PhidgetComposite: public PhidgetComponent
{

};

namespace helpers {

void openWaitForAttachment(PhidgetHandle handle, int32_t serial_number,
                           int hub_port, bool is_hub_port_device, int channel);

void closeAndDelete(PhidgetHandle *handle) noexcept;

}  // namespace helpers
}  // namespace phidgets

#endif  // PHIDGETS_API_PHIDGET22_H
