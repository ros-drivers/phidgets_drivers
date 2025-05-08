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

#include <functional>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_current_inputs/current_inputs_ros_i.h"
#include "phidgets_api/current_inputs.h"

namespace phidgets {

CurrentInputsRosI::CurrentInputsRosI(ros::NodeHandle nh,
                                     ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets CurrentInputs");

    int serial_num;
    if (!nh_private_.getParam("serial", serial_num))
    {
        serial_num = -1;  // default open any device
    }
    int hub_port;
    if (!nh_private.getParam("hub_port", hub_port))
    {
        hub_port = 0;  // only used if the device is on a VINT hub_port
    }
    bool is_hub_port_device;
    if (!nh_private.getParam("is_hub_port_device", is_hub_port_device))
    {
        // only used if the device is on a VINT hub_port
        is_hub_port_device = false;
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 250;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 0;
    }
    if (nh_private.getParam("server_name", server_name_) &&
        nh_private.getParam("server_ip", server_ip_))
    {
        PhidgetNet_addServer(server_name_.c_str(), server_ip_.c_str(), 5661, "",
                             0);

        ROS_INFO("Using phidget server %s at IP %s", server_name_.c_str(),
                 server_ip_.c_str());
    }

    ROS_INFO("Connecting to Phidgets CurrentInputs serial %d, hub port %d ...",
             serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(ai_mutex_);

    int n_in;
    try
    {
        ais_ = std::make_unique<CurrentInputs>(
            serial_num, hub_port, is_hub_port_device,
            std::bind(&CurrentInputsRosI::sensorChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        n_in = ais_->getInputCount();
        ROS_INFO("Connected %d inputs", n_in);
        val_to_pubs_.resize(n_in);
        for (int i = 0; i < n_in; i++)
        {
            char topicname[] = "current_input00";
            snprintf(topicname, sizeof(topicname), "current_input%02d", i);
            val_to_pubs_[i].pub =
                nh_.advertise<std_msgs::Float64>(topicname, 1);

            ais_->setDataInterval(i, data_interval_ms);

            val_to_pubs_[i].last_val = ais_->getSensorValue(i);
        }
    } catch (const Phidget22Error& err)
    {
        ROS_ERROR("CurrentInputs: %s", err.what());
        throw;
    }

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &CurrentInputsRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (int i = 0; i < n_in; ++i)
        {
            publishLatest(i);
        }
    }
}

void CurrentInputsRosI::publishLatest(int index)
{
    std_msgs::Float64 msg;
    msg.data = val_to_pubs_[index].last_val;
    val_to_pubs_[index].pub.publish(msg);
}

void CurrentInputsRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(ai_mutex_);
    for (int i = 0; i < static_cast<int>(val_to_pubs_.size()); ++i)
    {
        publishLatest(i);
    }
}

void CurrentInputsRosI::sensorChangeCallback(int index, double sensor_value)
{
    if (static_cast<int>(val_to_pubs_.size()) > index)
    {
        std::lock_guard<std::mutex> lock(ai_mutex_);
        val_to_pubs_[index].last_val = sensor_value;

        if (publish_rate_ <= 0)
        {
            publishLatest(index);
        }
    }
}

}  // namespace phidgets
