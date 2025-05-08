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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "phidgets_api/stepper.hpp"
#include "phidgets_stepper/stepper_ros_i.hpp"

namespace phidgets {

StepperRosI::StepperRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_stepper_node", options), ready(false)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Stepper");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int channel_num = this->declare_parameter("channel", 0);

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    // only used if the device is on a VINT hub_port
    bool is_hub_port_device =
        this->declare_parameter("is_hub_port_device", false);

    base_frame_ =
        this->declare_parameter("base_frame", std::string("phidgets"));
    joint_.header.frame_id = base_frame_;
    joint_.name.resize(1);
    joint_.position.resize(1);
    joint_.velocity.resize(1);
    joint_.effort.resize(1);
    joint_.name[0] =
        this->declare_parameter("joint_name", std::string("stepper"));

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
                "Connecting to Phidgets Stepper serial %d, hub port %d ...",
                serial_num, hub_port);

    lastCommand.mode =
        phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED;
    lastCommand.target = 0;
    lastCommand.velocity = 0;

    config_pub_ = this->create_publisher<phidgets_msgs::msg::StepperConfig>(
        "~/config", 1);
    state_pub_ =
        this->create_publisher<phidgets_msgs::msg::StepperState>("~/state", 1);
    joint_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("~/joint", 1);
    zero_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/zero", std::bind(&StepperRosI::zeroCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(stepper_mutex_);

    try
    {
        stepper_ = std::make_unique<Stepper>(
            serial_num, hub_port, is_hub_port_device, channel_num,
            std::bind(&StepperRosI::positionChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&StepperRosI::velocityChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&StepperRosI::stoppedCallback, this,
                      std::placeholders::_1));

        // TODO: Add callback for parameter change
        dataInterval_ = this->declare_parameter("data_interval_ms", 250);
        failsafeTime_ = this->declare_parameter("failsafe_time_ms", 1000);
        positionOffset_ = this->declare_parameter("position_offset", 0.0);
        rescaleFactor_ = this->declare_parameter("rescale_factor",
                                                 stepper_->getRescaleFactor());
        stepper_->setRescaleFactor(rescaleFactor_);
        RCLCPP_INFO(this->get_logger(), "Rescale factor: %f rad/tick",
                    rescaleFactor_);
        acceleration_ = this->declare_parameter("acceleration",
                                                stepper_->getMaxAcceleration());
        velocityLimit_ = this->declare_parameter(
            "velocity_limit", stepper_->getMaxVelocityLimit());
        currentLimit_ = this->declare_parameter("current_limit",
                                                stepper_->getMaxCurrentLimit());
        holdingCurrentLimit_ = this->declare_parameter(
            "holding_current_limit", stepper_->getMaxCurrentLimit());

        // Initial value, just in case it gets engaged at the wrong moment
        stepper_->setTargetPosition(stepper_->getPosition());

        applyParameters();
        updateConfig();
        updateState();
        updateJoint();

    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Stepper: %s", err.what());
        throw;
    }

    command_sub_ =
        this->create_subscription<phidgets_msgs::msg::StepperCommand>(
            "~/command", 1,
            std::bind(&StepperRosI::commandCallback, this,
                      std::placeholders::_1));

    config_pub_->publish(config_);
    state_pub_->publish(state_);
    joint_pub_->publish(joint_);

    failsafe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(failsafeTime_ / 2),
        std::bind(&StepperRosI::failsafeTimerCallback, this));
    config_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StepperRosI::configTimerCallback, this));

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&StepperRosI::timerCallback, this));
    }

    ready = true;
}

void StepperRosI::applyParameters()
{
    stepper_->setDataInterval(dataInterval_);
    stepper_->enableFailsafe(failsafeTime_);
    stepper_->resetFailesafe();
    stepper_->addPositionOffset(positionOffset_);
    stepper_->setRescaleFactor(rescaleFactor_);
    stepper_->setAcceleration(acceleration_);
    stepper_->setVelocityLimit(velocityLimit_);
    stepper_->setCurrentLimit(currentLimit_);
    stepper_->setHoldingCurrentLimit(holdingCurrentLimit_);
}

void StepperRosI::updateConfig()
{
    config_.min_failsafe_time = stepper_->getMinFailsafeTime();
    config_.max_failsafe_time = stepper_->getMaxFailsafeTime();
    config_.min_position = stepper_->getMinPosition();
    config_.max_position = stepper_->getMaxPosition();
    config_.min_acceleration = stepper_->getMinAcceleration();
    config_.max_acceleration = stepper_->getMaxAcceleration();
    config_.min_velocity_limit = stepper_->getMinVelocityLimit();
    config_.max_velocity_limit = stepper_->getMaxVelocityLimit();
    config_.min_current_limit = stepper_->getMinCurrentLimit();
    config_.max_current_limit = stepper_->getMaxCurrentLimit();
    config_.min_data_interval = stepper_->getMinDataInterval();
    config_.max_data_interval = stepper_->getMaxDataInterval();
}

void StepperRosI::updateState()
{
    state_.header.stamp = this->get_clock()->now();
    state_.is_moving = stepper_->getIsMoving();
    state_.is_engaged = stepper_->getEngaged();
    state_.target_position = stepper_->getTargetPosition();
}

void StepperRosI::updateJoint()
{
    joint_.header.stamp = this->get_clock()->now();
    joint_.position[0] = stepper_->getPosition();
    joint_.velocity[0] = stepper_->getVelocity();
    joint_.effort[0] = 0.0;  // Not available
}

void StepperRosI::configTimerCallback()
{
    std::lock_guard<std::mutex> lock(stepper_mutex_);
    updateConfig();
    config_pub_->publish(config_);
}

void StepperRosI::failsafeTimerCallback()
{
    std::lock_guard<std::mutex> lock(stepper_mutex_);
    stepper_->resetFailesafe();
}

void StepperRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(stepper_mutex_);
    updateState();
    state_pub_->publish(state_);
    updateJoint();
    joint_pub_->publish(joint_);
}

void StepperRosI::positionChangeCallback(int /*channel*/, double position)
{
    if (ready)
    {
        joint_.position[0] = position;
        joint_pub_->publish(joint_);
    }
}

void StepperRosI::velocityChangeCallback(int /*channel*/, double velocity)
{
    if (ready)
    {
        joint_.velocity[0] = velocity;
        joint_pub_->publish(joint_);
    }
}

void StepperRosI::stoppedCallback(int /*channel*/)
{
    if (ready)
    {
        state_.is_moving = false;
        state_pub_->publish(state_);
    }
}

void StepperRosI::commandCallback(const phidgets_msgs::msg::StepperCommand& msg)
{
    if (ready)
    {
        std::lock_guard<std::mutex> lock(stepper_mutex_);
        try
        {
            if ((msg.mode != lastCommand.mode) ||
                (msg.mode ==
                 phidgets_msgs::msg::StepperCommand::CONTROL_MODE_DISENGAGED))
            {
                stepper_->setEngaged(0);
            }
            if (msg.mode ==
                phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STEP)
            {
                stepper_->setControlMode(CONTROL_MODE_STEP);
                stepper_->setVelocityLimit(msg.velocity);
                stepper_->setTargetPosition(msg.target);
                stepper_->setEngaged(1);
            } else if (msg.mode ==
                       phidgets_msgs::msg::StepperCommand::CONTROL_MODE_STOP)
            {
                stepper_->setControlMode(CONTROL_MODE_STEP);
                stepper_->setVelocityLimit(msg.velocity);
                stepper_->setTargetPosition(stepper_->getPosition());
                stepper_->setEngaged(1);
            } else if (msg.mode ==
                       phidgets_msgs::msg::StepperCommand::CONTROL_MODE_RUN)
            {
                stepper_->setControlMode(CONTROL_MODE_RUN);
                stepper_->setVelocityLimit(msg.velocity);
                stepper_->setEngaged(1);
            } else if (msg.mode == phidgets_msgs::msg::StepperCommand::
                                       CONTROL_MODE_DISENGAGED)
            {
                // Nothing to do
            }
            lastCommand = msg;
        } catch (const Phidget22Error& err)
        {
            RCLCPP_ERROR(get_logger(), "Stepper: %s", err.what());
        }
    }
}

void StepperRosI::zeroCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (ready)
    {
        std::lock_guard<std::mutex> lock(stepper_mutex_);
        stepper_->addPositionOffset(-stepper_->getPosition());
        RCLCPP_INFO(this->get_logger(), "Stepper: set position to zero");
        response->success = true;
        response->message = "set current position to zero";
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::StepperRosI)
