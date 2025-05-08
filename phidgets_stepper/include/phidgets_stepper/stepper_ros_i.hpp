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

#ifndef PHIDGETS_STEPPER_STEPPER_ROS_I_HPP
#define PHIDGETS_STEPPER_STEPPER_ROS_I_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "phidgets_api/stepper.hpp"
#include "phidgets_msgs/msg/stepper_config.hpp"
#include "phidgets_msgs/msg/stepper_state.hpp"
#include "phidgets_msgs/msg/stepper_command.hpp"

namespace phidgets {

class StepperRosI final : public rclcpp::Node
{
  public:
    explicit StepperRosI(const rclcpp::NodeOptions& options);

  private:
    bool ready;
    std::unique_ptr<Stepper> stepper_;
    std::mutex stepper_mutex_;

    void timerCallback();
    void configTimerCallback();
    void failsafeTimerCallback();
    rclcpp::TimerBase::SharedPtr config_timer_;
    rclcpp::TimerBase::SharedPtr failsafe_timer_;
    rclcpp::TimerBase::SharedPtr timer_;
    double publish_rate_;
    std::string server_name_;
    std::string server_ip_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service_;
    void zeroCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    phidgets_msgs::msg::StepperCommand lastCommand;
    void commandCallback(const phidgets_msgs::msg::StepperCommand& msg);

    std::string base_frame_;
    sensor_msgs::msg::JointState joint_;
    void updateJoint();

    // All config variables handled trough the StepperConfig message
    phidgets_msgs::msg::StepperConfig config_;
    void updateConfig();

    // All state variables handled trough the StepperState message
    phidgets_msgs::msg::StepperState state_;
    void updateState();

    // All config variables handled as ros param
    uint32_t failsafeTime_;       // RW
    double positionOffset_;       // WO
    double acceleration_;         // RW
    double velocityLimit_;        // RW
    double currentLimit_;         // RW
    double holdingCurrentLimit_;  // RW
    double rescaleFactor_;        // RW
    uint32_t dataInterval_;       // RW
    double dataRate_;             // RW
    void applyParameters();

    rclcpp::Subscription<phidgets_msgs::msg::StepperCommand>::SharedPtr
        command_sub_;
    rclcpp::Publisher<phidgets_msgs::msg::StepperConfig>::SharedPtr config_pub_;
    rclcpp::Publisher<phidgets_msgs::msg::StepperState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    void positionChangeCallback(int channel, double position);

    void velocityChangeCallback(int channel, double velocity);

    void stoppedCallback(int channel);
};

}  // namespace phidgets

#endif  // PHIDGETS_STEPPER_STEPPER_ROS_I_HPP
