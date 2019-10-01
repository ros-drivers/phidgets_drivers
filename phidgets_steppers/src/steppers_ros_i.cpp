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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "phidgets_api/steppers.hpp"
#include "phidgets_steppers/steppers_ros_i.hpp"

namespace phidgets {

StepperServices::StepperServices(Steppers* steppers, int channel,
                                 rclcpp::Node* node)
    : steppers_(steppers), channel_(channel)
{
    char service_name[SERVICE_NAME_LENGTH_MAX];

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_enabled%02d",
             (int)channel);
    set_enabled_service_ = node->create_service<std_srvs::srv::SetBool>(
        service_name, std::bind(&StepperServices::setEnabledCallback, this,
                                std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_target_position%02d",
             (int)channel);
    set_target_position_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setTargetPositionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_velocity_limit%02d",
             (int)channel);
    set_velocity_limit_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setVelocityLimitCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_acceleration%02d",
             (int)channel);
    set_acceleration_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setAccelerationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_current_limit%02d",
             (int)channel);
    set_current_limit_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setCurrentLimitCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX,
             "set_holding_current_limit%02d", (int)channel);
    set_holding_current_limit_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setHoldingCurrentLimitCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "set_position%02d",
             (int)channel);
    set_position_service_ =
        node->create_service<phidgets_msgs::srv::SetFloat64>(
            service_name,
            std::bind(&StepperServices::setPositionCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "get_settings%02d",
             (int)channel);
    get_settings_service_ =
        node->create_service<phidgets_msgs::srv::GetStepperSettings>(
            service_name,
            std::bind(&StepperServices::getSettingsCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

    snprintf(service_name, SERVICE_NAME_LENGTH_MAX, "get_setting_ranges%02d",
             (int)channel);
    get_setting_ranges_service_ =
        node->create_service<phidgets_msgs::srv::GetStepperSettingRanges>(
            service_name,
            std::bind(&StepperServices::getSettingRangesCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
}

void StepperServices::setEnabledCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setEngaged(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setTargetPositionCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setTargetPosition(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setVelocityLimitCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setVelocityLimit(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setAccelerationCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setAcceleration(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setCurrentLimitCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setCurrentLimit(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setHoldingCurrentLimitCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        steppers_->setHoldingCurrentLimit(channel_, req->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::setPositionCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res)
{
    bool success = true;
    try
    {
        double position = steppers_->getPosition(channel_);
        double position_offset = req->data - position;
        steppers_->addPositionOffset(channel_, position_offset);
        position = steppers_->getPosition(channel_);
        steppers_->positionChangeHandler(channel_, position);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::getSettingsCallback(
    const std::shared_ptr<phidgets_msgs::srv::GetStepperSettings::Request> req,
    std::shared_ptr<phidgets_msgs::srv::GetStepperSettings::Response> res)
{
    (void)req;  // remove unused parameter warning
    bool success = true;
    try
    {
        res->enabled = steppers_->getEngaged(channel_);
        res->target_position = steppers_->getTargetPosition(channel_);
        res->velocity_limit = steppers_->getVelocityLimit(channel_);
        res->acceleration = steppers_->getAcceleration(channel_);
        res->current_limit = steppers_->getCurrentLimit(channel_);
        res->holding_current_limit =
            steppers_->getHoldingCurrentLimit(channel_);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

void StepperServices::getSettingRangesCallback(
    const std::shared_ptr<phidgets_msgs::srv::GetStepperSettingRanges::Request>
        req,
    std::shared_ptr<phidgets_msgs::srv::GetStepperSettingRanges::Response> res)
{
    (void)req;  // remove unused parameter warning
    bool success = true;
    try
    {
        res->position.min = steppers_->getMinPosition(channel_);
        res->position.max = steppers_->getMaxPosition(channel_);
        res->velocity_limit.min = steppers_->getMinVelocityLimit(channel_);
        res->velocity_limit.max = steppers_->getMaxVelocityLimit(channel_);
        res->acceleration.min = steppers_->getMinAcceleration(channel_);
        res->acceleration.max = steppers_->getMaxAcceleration(channel_);
        res->current_limit.min = steppers_->getMinCurrentLimit(channel_);
        res->current_limit.max = steppers_->getMaxCurrentLimit(channel_);
    } catch (const phidgets::Phidget22Error& err)
    {
        success = false;
    }
    res->success = success;
}

SteppersRosI::SteppersRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_steppers_node", options)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Steppers");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    RCLCPP_INFO(get_logger(),
                "Connecting to Phidgets Steppers serial %d, hub port %d ...",
                serial_num, hub_port);

    int data_interval_ms = this->declare_parameter("data_interval_ms", 250);

    publish_rate_ = this->declare_parameter("publish_rate", 0.0);
    if (publish_rate_ > 1000.0)
    {
        throw std::runtime_error("Publish rate must be <= 1000");
    }

    int watchdog_interval_ms =
        this->declare_parameter("watchdog_interval_ms", 1000);
    if ((watchdog_interval_ms < 500.0) || (watchdog_interval_ms > 10000.0))
    {
        throw std::runtime_error(
            "Watchdog interval must be >= 500 and <= 10000");
    }
    // divide watchdog interval by 4 to abide rule about 1/3 failsafe time
    // update
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int64_t>(watchdog_interval_ms / 4)),
        std::bind(&SteppersRosI::watchdogTimerCallback, this));

    frame_id_ = this->declare_parameter("frame_id", "stepper");

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(stepper_mutex_);

    int n_steppers;
    try
    {
        steppers_ = std::make_unique<Steppers>(
            serial_num, hub_port, false,
            std::bind(&SteppersRosI::positionChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SteppersRosI::velocityChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        n_steppers = steppers_->getStepperCount();
        if (n_steppers != 1)
        {
            RCLCPP_INFO(get_logger(), "Connected to serial %d, %d steppers",
                        steppers_->getSerialNumber(), n_steppers);
        } else
        {
            RCLCPP_INFO(get_logger(), "Connected to serial %d, %d stepper",
                        steppers_->getSerialNumber(), n_steppers);
        }

        stepper_data_to_pub_.resize(n_steppers);
        for (size_t channel = 0; channel < stepper_data_to_pub_.size();
             channel++)
        {
            steppers_->setDataInterval(channel, data_interval_ms);

            char parameter_name[PARAMETER_NAME_LENGTH_MAX];
            sprintf(parameter_name, "joint_name%02d", (int)channel);
            stepper_data_to_pub_[channel].joint_name =
                this->declare_parameter(parameter_name, parameter_name);

            sprintf(parameter_name, "rescale_factor%02d", (int)channel);
            double rescale_factor =
                this->declare_parameter(parameter_name, 1.0);
            steppers_->setRescaleFactor(channel, rescale_factor);

            sprintf(parameter_name, "position_control_mode%02d", (int)channel);
            bool position_control_mode =
                this->declare_parameter(parameter_name, true);
            steppers_->setStepControlMode(channel, position_control_mode);

            // setting position control mode sets velocity limit to 0 but
            // misreports it so fix by setting to 0.0 explicitly
            steppers_->setVelocityLimit(channel, 0.0);

            stepper_data_to_pub_[channel].stepper_srvs =
                std::make_unique<StepperServices>(steppers_.get(), channel,
                                                  this);

            // initialize current limits to zero to prevent motor damage
            steppers_->setCurrentLimit(channel, 0.0);
            steppers_->setHoldingCurrentLimit(channel, 0.0);

            steppers_->enableFailsafe(channel, watchdog_interval_ms);

            stepper_data_to_pub_[channel].last_position_val =
                steppers_->getPosition(channel);
            stepper_data_to_pub_[channel].last_velocity_val =
                steppers_->getVelocity(channel);
        }
    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Steppers: %s", err.what());
        throw;
    }

    stepper_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 100);

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&SteppersRosI::publishTimerCallback, this));
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        publishLatestJointStates();
    }
}

void SteppersRosI::publishTimerCallback()
{
    std::lock_guard<std::mutex> lock(stepper_mutex_);
    publishLatestJointStates();
}

void SteppersRosI::publishLatestJointStates()
{
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    for (size_t channel = 0; channel < stepper_data_to_pub_.size(); channel++)
    {
        msg.name.push_back(stepper_data_to_pub_[channel].joint_name);
        msg.position.push_back(stepper_data_to_pub_[channel].last_position_val);
        msg.velocity.push_back(stepper_data_to_pub_[channel].last_velocity_val);
    }
    stepper_pub_->publish(msg);
}

void SteppersRosI::watchdogTimerCallback()
{
    std::lock_guard<std::mutex> lock(stepper_mutex_);
    kickWatchdogTimers();
}

void SteppersRosI::kickWatchdogTimers()
{
    for (size_t channel = 0; channel < steppers_->getStepperCount(); channel++)
    {
        steppers_->resetFailsafe(channel);
    }
}

void SteppersRosI::positionChangeCallback(int channel, double position)
{
    if (static_cast<int>(stepper_data_to_pub_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(stepper_mutex_);
        stepper_data_to_pub_[channel].last_position_val = position;

        if (publish_rate_ <= 0.0)
        {
            publishLatestJointStates();
        }
    }
}

void SteppersRosI::velocityChangeCallback(int channel, double velocity)
{
    if (static_cast<int>(stepper_data_to_pub_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(stepper_mutex_);
        stepper_data_to_pub_[channel].last_velocity_val = velocity;

        if (publish_rate_ <= 0.0)
        {
            publishLatestJointStates();
        }
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::SteppersRosI)
