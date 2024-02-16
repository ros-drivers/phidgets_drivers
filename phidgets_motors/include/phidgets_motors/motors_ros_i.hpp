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

#ifndef PHIDGETS_MOTORS_MOTORS_ROS_I_HPP
#define PHIDGETS_MOTORS_MOTORS_ROS_I_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/motors.hpp"

namespace phidgets {

class DutyCycleSetter final
{
  public:
    explicit DutyCycleSetter(Motors* motors, int index, rclcpp::Node* node,
                             const std::string& topicname);

  private:
    void setMsgCallback(const std_msgs::msg::Float64::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    Motors* motors_;
    int index_;
};

struct MotorVals {
    std::unique_ptr<DutyCycleSetter> duty_cycle_sub;
    double last_duty_cycle_val;
    double last_back_emf_val;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_cycle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_emf_pub;
};

class MotorsRosI final : public rclcpp::Node
{
  public:
    explicit MotorsRosI(const rclcpp::NodeOptions& options);

  private:
    std::unique_ptr<Motors> motors_;
    std::mutex motor_mutex_;
    std::vector<MotorVals> motor_vals_;

    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    double publish_rate_;
    std::string server_name_;
    std::string server_ip_;

    void publishLatestDutyCycle(int index);
    void publishLatestBackEMF(int index);

    void dutyCycleChangeCallback(int channel, double duty_cycle);

    void backEMFChangeCallback(int channel, double back_emf);
};

}  // namespace phidgets

#endif  // PHIDGETS_MOTORS_MOTORS_ROS_I_HPP
