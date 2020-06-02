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

#ifndef PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_HPP
#define PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_HPP

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/temperature.hpp"

namespace phidgets {

class TemperatureRosI final : public rclcpp::Node
{
  public:
    explicit TemperatureRosI(const rclcpp::NodeOptions& options);

  private:
    std::unique_ptr<Temperature> temperature_;
    std::mutex temperature_mutex_;
    double last_temperature_reading_{0.0};
    bool got_first_data_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_pub_;
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    double publish_rate_;

    void publishLatest();

    void temperatureChangeCallback(double temperature);
};

}  // namespace phidgets

#endif  // PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_HPP
