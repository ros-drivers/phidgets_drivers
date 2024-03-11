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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/motors.hpp"
#include "phidgets_motors/motors_ros_i.hpp"

namespace phidgets {

MotorsRosI::MotorsRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_motors_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Motors");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    int data_interval_ms = this->declare_parameter("data_interval_ms", 250);

    double braking_strength = this->declare_parameter("braking_strength", 0.0);

    publish_rate_ = this->declare_parameter("publish_rate", 0.0);
    if (publish_rate_ > 1000.0)
    {
        throw std::runtime_error("Publish rate must be <= 1000");
    }

    this->declare_parameter("server_name",
                            rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("server_ip",
                            rclcpp::ParameterType::PARAMETER_STRING);
    if (this->get_parameter("server_name", server_name_) &&
        this->get_parameter("server_ip", server_ip_))
    {
        PhidgetNet_addServer(server_name_.c_str(), server_ip_.c_str(), 5661, "",
                             0);

        RCLCPP_INFO(get_logger(), "Using phidget server %s at IP %s",
                    server_name_.c_str(), server_ip_.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "Connecting to Phidgets Motors serial %d, hub port %d ...",
                serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(motor_mutex_);

    uint32_t n_motors;
    try
    {
        motors_ = std::make_unique<Motors>(
            serial_num, hub_port, false,
            std::bind(&MotorsRosI::dutyCycleChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotorsRosI::backEMFChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        n_motors = motors_->getMotorCount();
        RCLCPP_INFO(get_logger(), "Connected to serial %d, %u motors",
                    motors_->getSerialNumber(), n_motors);

        motor_vals_.resize(n_motors);
        for (uint32_t i = 0; i < n_motors; i++)
        {
            char topicname[100];
            snprintf(topicname, sizeof(topicname), "set_motor_duty_cycle%02d",
                     i);
            motor_vals_[i].duty_cycle_sub = std::make_unique<DutyCycleSetter>(
                motors_.get(), i, this, topicname);

            snprintf(topicname, sizeof(topicname), "motor_duty_cycle%02d", i);
            motor_vals_[i].duty_cycle_pub =
                this->create_publisher<std_msgs::msg::Float64>(topicname, 1);
            motor_vals_[i].last_duty_cycle_val = motors_->getDutyCycle(i);

            snprintf(topicname, sizeof(topicname), "motor_back_emf%02d", i);
            motor_vals_[i].back_emf_pub =
                this->create_publisher<std_msgs::msg::Float64>(topicname, 1);
            if (motors_->backEMFSensingSupported(i))
            {
                motor_vals_[i].last_back_emf_val = motors_->getBackEMF(i);
            } else
            {
                RCLCPP_INFO(get_logger(),
                            "Back EMF sensing not supported for %s", topicname);
            }

            motors_->setDataInterval(i, data_interval_ms);
            motors_->setBraking(i, braking_strength);
        }
    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Motors: %s", err.what());
        throw;
    }

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&MotorsRosI::timerCallback, this));
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (uint32_t i = 0; i < n_motors; ++i)
        {
            publishLatestDutyCycle(i);
            publishLatestBackEMF(i);
        }
    }
}

void MotorsRosI::publishLatestDutyCycle(int index)
{
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = motor_vals_[index].last_duty_cycle_val;
    motor_vals_[index].duty_cycle_pub->publish(std::move(msg));
}

void MotorsRosI::publishLatestBackEMF(int index)
{
    if (motors_->backEMFSensingSupported(index))
    {
        auto backemf_msg = std::make_unique<std_msgs::msg::Float64>();
        backemf_msg->data = motor_vals_[index].last_back_emf_val;
        motor_vals_[index].back_emf_pub->publish(std::move(backemf_msg));
    }
}

void MotorsRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (int i = 0; i < static_cast<int>(motor_vals_.size()); ++i)
    {
        publishLatestDutyCycle(i);
        publishLatestBackEMF(i);
    }
}

DutyCycleSetter::DutyCycleSetter(Motors* motors, int index, rclcpp::Node* node,
                                 const std::string& topicname)
    : motors_(motors), index_(index)
{
    subscription_ = node->create_subscription<std_msgs::msg::Float64>(
        topicname, rclcpp::QoS(1),
        std::bind(&DutyCycleSetter::setMsgCallback, this,
                  std::placeholders::_1));
}

void DutyCycleSetter::setMsgCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    try
    {
        motors_->setDutyCycle(index_, msg->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        // If the data was wrong, the lower layers will throw an exception; just
        // catch and ignore here so we don't crash the node.
    }
}

void MotorsRosI::dutyCycleChangeCallback(int channel, double duty_cycle)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_duty_cycle_val = duty_cycle;

        if (publish_rate_ <= 0.0)
        {
            publishLatestDutyCycle(channel);
        }
    }
}

void MotorsRosI::backEMFChangeCallback(int channel, double back_emf)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_back_emf_val = back_emf;

        if (publish_rate_ <= 0.0)
        {
            publishLatestBackEMF(channel);
        }
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::MotorsRosI)
