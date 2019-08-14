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

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "phidgets_ik/ik_ros_i.h"
#include "phidgets_msgs/SetDigitalOutput.h"

namespace phidgets {

IKRosI::IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets IK");

    ROS_INFO("Opening device");
    int serial_num;
    if (!nh_private_.getParam("serial", serial_num))
    {
        serial_num = -1;  // default open any device
    }
    int digital_inputs_hub_port;
    if (!nh_private.getParam("digital_inputs_hub_port",
                             digital_inputs_hub_port))
    {
        digital_inputs_hub_port =
            0;  // only used if the device is on a VINT hub_port
    }
    bool digital_inputs_is_hub_port_device;
    if (!nh_private.getParam("digital_inputs_is_hub_port_device",
                             digital_inputs_is_hub_port_device))
    {
        // only used if the device is on a VINT hub_port
        digital_inputs_is_hub_port_device = false;
    }
    int digital_outputs_hub_port;
    if (!nh_private.getParam("digital_outputs_hub_port",
                             digital_outputs_hub_port))
    {
        digital_outputs_hub_port =
            0;  // only used if the device is on a VINT hub_port
    }
    bool digital_outputs_is_hub_port_device;
    if (!nh_private.getParam("digital_outputs_is_hub_port_device",
                             digital_outputs_is_hub_port_device))
    {
        // only used if the device is on a VINT hub_port
        digital_outputs_is_hub_port_device = false;
    }
    int analog_inputs_hub_port;
    if (!nh_private.getParam("analog_inputs_hub_port", analog_inputs_hub_port))
    {
        analog_inputs_hub_port =
            0;  // only used if the device is on a VINT hub_port
    }
    bool analog_inputs_is_hub_port_device;
    if (!nh_private.getParam("analog_inputs_is_hub_port_device",
                             analog_inputs_is_hub_port_device))
    {
        // only used if the device is on a VINT hub_port
        analog_inputs_is_hub_port_device = false;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 0;
    }

    ROS_INFO(
        "Waiting for Phidgets DigitalInputs serial %d, hub port %d to be "
        "attached...",
        serial_num, digital_inputs_hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(ik_mutex_);

    dis_ = std::make_unique<DigitalInputs>(
        serial_num, digital_inputs_hub_port, digital_inputs_is_hub_port_device,
        std::bind(&IKRosI::stateChangeCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    int n_dis = dis_->getInputCount();
    ROS_INFO("Connected %d inputs", n_dis);
    for (int i = 0; i < n_dis; i++)
    {
        char topicname[] = "digital_input00";
        snprintf(topicname, sizeof(topicname), "digital_input%02d", i);
        di_pubs_.push_back(nh_.advertise<std_msgs::Bool>(topicname, 1));
        last_di_vals_[i] = dis_->getInputValue(i);
    }

    ROS_INFO(
        "Waiting for Phidgets DigitalOutputs serial %d, hub port %d to be "
        "attached...",
        serial_num, digital_outputs_hub_port);

    dos_ =
        std::make_unique<DigitalOutputs>(serial_num, digital_outputs_hub_port,
                                         digital_outputs_is_hub_port_device);

    int n_out = dos_->getOutputCount();
    ROS_INFO("Connected %d outputs", n_out);
    out_subs_.resize(n_out);
    for (int i = 0; i < n_out; i++)
    {
        char topicname[] = "digital_output00";
        snprintf(topicname, sizeof(topicname), "digital_output%02d", i);
        out_subs_[i] = std::make_unique<IKDigitalOutputSetter>(dos_.get(), i,
                                                               nh, topicname);
    }
    out_srv_ = nh_.advertiseService("set_digital_output",
                                    &IKRosI::setSrvCallback, this);

    ROS_INFO(
        "Waiting for Phidgets AnalogInputs serial %d, hub port %d to be "
        "attached...",
        serial_num, analog_inputs_hub_port);

    ais_ = std::make_unique<AnalogInputs>(
        serial_num, analog_inputs_hub_port, analog_inputs_is_hub_port_device,
        std::bind(&IKRosI::sensorChangeCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    int n_ais = ais_->getInputCount();
    ROS_INFO("Connected %d inputs", n_ais);
    for (int i = 0; i < n_ais; i++)
    {
        char topicname[] = "analog_input00";
        snprintf(topicname, sizeof(topicname), "analog_input%02d", i);
        ai_pubs_.push_back(nh_.advertise<std_msgs::Float64>(topicname, 1));
        last_ai_vals_[i] = ais_->getSensorValue(i);
    }

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &IKRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (int i = 0; i < n_dis; ++i)
        {
            publishLatestDI(i);
        }
        for (int i = 0; i < n_ais; ++i)
        {
            publishLatestAI(i);
        }
    }
}

void IKRosI::publishLatestDI(int index)
{
    std_msgs::Bool msg;
    msg.data = last_di_vals_[index];
    di_pubs_[index].publish(msg);
}

void IKRosI::stateChangeCallback(int index, int input_value)
{
    if (static_cast<int>(last_di_vals_.size()) > index)
    {
        std::lock_guard<std::mutex> lock(ik_mutex_);
        last_di_vals_[index] = input_value == 0;

        if (publish_rate_ <= 0)
        {
            publishLatestDI(index);
        }
    }
}

bool IKRosI::setSrvCallback(phidgets_msgs::SetDigitalOutput::Request& req,
                            phidgets_msgs::SetDigitalOutput::Response& res)
{
    dos_->setOutputState(req.index, req.state);
    res.success = true;
    return true;
}

IKDigitalOutputSetter::IKDigitalOutputSetter(DigitalOutputs* dos, int index,
                                             ros::NodeHandle nh,
                                             const std::string& topicname)
    : dos_(dos), index_(index)
{
    subscription_ = nh.subscribe(topicname, 1,
                                 &IKDigitalOutputSetter::setMsgCallback, this);
}

void IKDigitalOutputSetter::setMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
    dos_->setOutputState(index_, msg->data);
}

void IKRosI::publishLatestAI(int index)
{
    double VREF = 5.0;
    // get rawsensorvalue and divide by 4096, which according to the
    // documentation for both the IK888 and IK222 are the maximum sensor value
    // Multiply by VREF=5.0V to get voltage
    std_msgs::Float64 msg;
    msg.data = VREF * last_ai_vals_[index] / 4095.0;
    ai_pubs_[index].publish(msg);
}

void IKRosI::sensorChangeCallback(int index, double sensor_value)
{
    if (static_cast<int>(last_ai_vals_.size()) > index)
    {
        std::lock_guard<std::mutex> lock(ik_mutex_);
        last_ai_vals_[index] = sensor_value;

        if (publish_rate_ <= 0)
        {
            publishLatestAI(index);
        }
    }
}

void IKRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(ik_mutex_);
    for (int i = 0; i < static_cast<int>(last_di_vals_.size()); ++i)
    {
        publishLatestDI(i);
    }
    for (int i = 0; i < static_cast<int>(last_ai_vals_.size()); ++i)
    {
        publishLatestAI(i);
    }
}

}  // namespace phidgets
