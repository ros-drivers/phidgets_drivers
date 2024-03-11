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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/analog_outputs.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "phidgets_analog_outputs/analog_outputs_ros_i.hpp"
#include "phidgets_msgs/srv/set_analog_output.hpp"

namespace phidgets {

AnalogOutputsRosI::AnalogOutputsRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_analog_outputs_node", options)
{
    RCLCPP_INFO(get_logger(), "Starting Phidgets Analog Outputs");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

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

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    bool is_hub_port_device = this->declare_parameter(
        "is_hub_port_device",
        false);  // only used if the device is on a VINT hub_port

    bool force_enable = this->declare_parameter(
        "force_enable", true);  // auto enable all outputs

    RCLCPP_INFO(
        get_logger(),
        "Connecting to Phidgets AnalogOutputs serial %d, hub port %d ...",
        serial_num, hub_port);

    try
    {
        aos_ = std::make_unique<AnalogOutputs>(serial_num, hub_port,
                                               is_hub_port_device);

    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "AnalogOutputs: %s", err.what());
        throw;
    }

    int n_out = aos_->getOutputCount();
    RCLCPP_INFO(get_logger(), "Connected %d outputs", n_out);
    out_subs_.resize(n_out);
    for (int i = 0; i < n_out; i++)
    {
        char topicname[] = "analog_output00";
        snprintf(topicname, sizeof(topicname), "analog_output%02d", i);
        out_subs_[i] = std::make_unique<AnalogOutputSetter>(aos_.get(), i, this,
                                                            topicname);

        if (force_enable)
        {
            aos_->setEnabledOutput(i, 1);  // auto enable output
        }
    }
    out_srv_ = this->create_service<phidgets_msgs::srv::SetAnalogOutput>(
        "set_analog_output",
        std::bind(&AnalogOutputsRosI::setSrvCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

bool AnalogOutputsRosI::setSrvCallback(
    const std::shared_ptr<phidgets_msgs::srv::SetAnalogOutput::Request> req,
    std::shared_ptr<phidgets_msgs::srv::SetAnalogOutput::Response> res)
{
    aos_->setOutputVoltage(req->index, req->voltage);
    res->success = true;
    return true;
}

AnalogOutputSetter::AnalogOutputSetter(AnalogOutputs* aos, int index,
                                       AnalogOutputsRosI* node,
                                       const std::string& topicname)
    : aos_(aos), index_(index)
{
    subscription_ = node->create_subscription<std_msgs::msg::Float64>(
        topicname, rclcpp::QoS(10),
        std::bind(&AnalogOutputSetter::setMsgCallback, this,
                  std::placeholders::_1));
}

void AnalogOutputSetter::setMsgCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    aos_->setOutputVoltage(index_, msg->data);
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::AnalogOutputsRosI)
