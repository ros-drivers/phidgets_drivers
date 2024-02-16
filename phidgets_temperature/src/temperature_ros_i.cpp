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

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_temperature/temperature_ros_i.hpp"

namespace phidgets {

TemperatureRosI::TemperatureRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_temperature_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Temperature");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    int thermocouple_type = this->declare_parameter("thermocouple_type", 0);

    int data_interval_ms = this->declare_parameter("data_interval_ms", 500);

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
                "Connecting to Phidgets Temperature serial %d, hub port %d, "
                "thermocouple "
                "type %d ...",
                serial_num, hub_port, thermocouple_type);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(temperature_mutex_);

    try
    {
        temperature_ = std::make_unique<Temperature>(
            serial_num, hub_port, false,
            std::bind(&TemperatureRosI::temperatureChangeCallback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Connected to serial %d",
                    temperature_->getSerialNumber());

        temperature_->setDataInterval(data_interval_ms);

        if (thermocouple_type != 0)
        {
            temperature_->setThermocoupleType(
                static_cast<ThermocoupleType>(thermocouple_type));
        }
    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Temperature: %s", err.what());
        throw;
    }

    temperature_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("temperature", 1);

    got_first_data_ = false;

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&TemperatureRosI::timerCallback, this));
    } else
    {
        // We'd like to get the temperature on startup here, but it can take
        // some some time for the setThermocoupleType() call above to complete
        // down on the sensor.  Instead, we wait for the first change callback
        // before we start publishing.
    }
}

void TemperatureRosI::publishLatest()
{
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = last_temperature_reading_;
    temperature_pub_->publish(std::move(msg));
}

void TemperatureRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    if (got_first_data_)
    {
        publishLatest();
    }
}

void TemperatureRosI::temperatureChangeCallback(double temperature)
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    last_temperature_reading_ = temperature;

    if (!got_first_data_)
    {
        got_first_data_ = true;
    }

    if (publish_rate_ <= 0.0)
    {
        publishLatest();
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::TemperatureRosI)
