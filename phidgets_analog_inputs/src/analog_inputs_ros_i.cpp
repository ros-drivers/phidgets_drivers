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

#include "phidgets_analog_inputs/analog_inputs_ros_i.hpp"
#include "phidgets_api/analog_inputs.hpp"

namespace phidgets {

AnalogInputsRosI::AnalogInputsRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_analog_inputs_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets AnalogInputs");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    // only used if the device is on a VINT hub_port
    bool is_hub_port_device =
        this->declare_parameter("is_hub_port_device", false);

    int data_interval_ms = this->declare_parameter("data_interval_ms", 250);

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

    RCLCPP_INFO(
        get_logger(),
        "Connecting to Phidgets AnalogInputs serial %d, hub port %d ...",
        serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(ai_mutex_);

    uint32_t n_in;
    try
    {
        ais_ = std::make_unique<AnalogInputs>(
            serial_num, hub_port, is_hub_port_device,
            std::bind(&AnalogInputsRosI::sensorChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        n_in = ais_->getInputCount();
        RCLCPP_INFO(get_logger(), "Connected to serial %d, %u inputs",
                    ais_->getSerialNumber(), n_in);
        val_to_pubs_.resize(n_in);
        for (uint32_t i = 0; i < n_in; i++)
        {
            char str[100];
            snprintf(str, sizeof(str), "gain%02d", i);
            val_to_pubs_[i].gain = this->declare_parameter(str, 1.0);

            snprintf(str, sizeof(str), "offset%02d", i);
            val_to_pubs_[i].offset = this->declare_parameter(str, 0.0);

            snprintf(str, sizeof(str), "analog_input%02d", i);
            val_to_pubs_[i].pub =
                this->create_publisher<std_msgs::msg::Float64>(str, 1);

            ais_->setDataInterval(i, data_interval_ms);
        }
    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "AnalogInputs: %s", err.what());
        throw;
    }

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&AnalogInputsRosI::timerCallback, this));
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (uint32_t i = 0; i < n_in; ++i)
        {
            publishLatest(i);
        }
    }
}

void AnalogInputsRosI::publishLatest(int index)
{
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = val_to_pubs_[index].last_val * val_to_pubs_[index].gain +
                val_to_pubs_[index].offset;
    val_to_pubs_[index].pub->publish(std::move(msg));
}

void AnalogInputsRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(ai_mutex_);
    for (int i = 0; i < static_cast<int>(val_to_pubs_.size()); ++i)
    {
        publishLatest(i);
    }
}

void AnalogInputsRosI::sensorChangeCallback(int index, double sensor_value)
{
    if (static_cast<int>(val_to_pubs_.size()) > index)
    {
        std::lock_guard<std::mutex> lock(ai_mutex_);
        val_to_pubs_[index].last_val = sensor_value;

        if (publish_rate_ <= 0.0)
        {
            publishLatest(index);
        }
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::AnalogInputsRosI)
