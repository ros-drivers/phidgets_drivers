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
#include <numeric>
#include <string>
#include <vector>

#include <sensor_msgs/JointState.h>

#include "phidgets_high_speed_encoder/high_speed_encoder_ros_i.h"
#include "phidgets_msgs/EncoderDecimatedSpeed.h"

namespace phidgets {

HighSpeedEncoderRosI::HighSpeedEncoderRosI(ros::NodeHandle nh,
                                           ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Encoders");

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
    if (!nh_private_.getParam("frame_id", frame_id_))
    {
        frame_id_ = "encoder_link";
    }
    if (!nh_private_.getParam("speed_filter_samples_len",
                              speed_filter_samples_len_))
    {
        speed_filter_samples_len_ = 10;
    }
    if (!nh_private_.getParam("speed_filter_idle_iter_loops_before_reset",
                              speed_filter_idle_iter_loops_before_reset_))
    {
        speed_filter_idle_iter_loops_before_reset_ = 1;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 0;
    }

    ROS_INFO("Connecting to Phidgets Encoders serial %d, hub port %d ...",
             serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(encoder_mutex_);

    int n_encs;
    try
    {
        encs_ = std::make_unique<Encoders>(
            serial_num, hub_port, false,
            std::bind(&HighSpeedEncoderRosI::positionChangeHandler, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));

        n_encs = encs_->getEncoderCount();
        ROS_INFO("Connected %d encoders", n_encs);
        enc_data_to_pub_.resize(n_encs);
        for (int i = 0; i < n_encs; i++)
        {
            char str[100];
            sprintf(str, "joint%u_name", i);
            if (!nh_private_.getParam(str, enc_data_to_pub_[i].joint_name))
            {
                enc_data_to_pub_[i].joint_name = "joint" + std::to_string(i);
            }

            sprintf(str, "joint%u_tick2rad", i);
            if (!nh_private_.getParam(str, enc_data_to_pub_[i].joint_tick2rad))
            {
                enc_data_to_pub_[i].joint_tick2rad = 1.0;
            }

            ROS_INFO("Channel %u: '%s'='%s'", i, str,
                     enc_data_to_pub_[i].joint_name.c_str());

            char buf[100];
            sprintf(buf, "joint_states_ch%u_decim_speed", i);
            ROS_INFO("Publishing decimated speed of channel %u to topic: %s", i,
                     buf);
            enc_data_to_pub_[i].encoder_decimspeed_pub =
                nh_.advertise<phidgets_msgs::EncoderDecimatedSpeed>(buf, 10);
            encs_->setEnabled(i, true);
        }
    } catch (const Phidget22Error& err)
    {
        ROS_ERROR("Encoders: %s", err.what());
        throw;
    }

    encoder_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &HighSpeedEncoderRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (int i = 0; i < n_encs; ++i)
        {
            publishLatest(i);
        }
    }
}

void HighSpeedEncoderRosI::publishLatest(int channel)
{
    int64_t absolute_position = encs_->getPosition(channel);

    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();
    js_msg.header.frame_id = frame_id_;

    js_msg.name.resize(enc_data_to_pub_.size());
    for (size_t i = 0; i < enc_data_to_pub_.size(); ++i)
    {
        js_msg.name[i] = enc_data_to_pub_[i].joint_name;
    }

    js_msg.position.resize(enc_data_to_pub_.size());
    js_msg.velocity.resize(enc_data_to_pub_.size());
    js_msg.effort.clear();

    for (size_t i = 0; i < enc_data_to_pub_.size(); ++i)
    {
        js_msg.position[i] =
            absolute_position * enc_data_to_pub_[i].joint_tick2rad;
        js_msg.velocity[i] = enc_data_to_pub_[i].instantaneous_speed *
                             enc_data_to_pub_[i].joint_tick2rad;
        enc_data_to_pub_[i].instantaneous_speed = 0.0;  // Reset speed

        if (speed_filter_samples_len_ > 0)
        {
            if (!enc_data_to_pub_[i].speed_buffer_updated)
            {
                if (++enc_data_to_pub_[i].loops_without_update_speed_buffer >=
                    speed_filter_idle_iter_loops_before_reset_)
                {
                    phidgets_msgs::EncoderDecimatedSpeed e;
                    e.header.stamp = ros::Time::now();
                    e.header.frame_id = frame_id_;
                    e.avr_speed = .0;
                    enc_data_to_pub_[i].encoder_decimspeed_pub.publish(e);
                }
            } else
            {
                enc_data_to_pub_[i].loops_without_update_speed_buffer = 0;

                if (enc_data_to_pub_[i].speeds_buffer.size() >=
                    static_cast<size_t>(speed_filter_samples_len_))
                {
                    const double avrg =
                        std::accumulate(
                            enc_data_to_pub_[i].speeds_buffer.begin(),
                            enc_data_to_pub_[i].speeds_buffer.end(), 0.0) /
                        enc_data_to_pub_[i].speeds_buffer.size();
                    enc_data_to_pub_[i].speeds_buffer.clear();

                    phidgets_msgs::EncoderDecimatedSpeed e;
                    e.header.stamp = ros::Time::now();
                    e.header.frame_id = frame_id_;
                    e.avr_speed = avrg * enc_data_to_pub_[i].joint_tick2rad;
                    enc_data_to_pub_[i].encoder_decimspeed_pub.publish(e);
                }
            }
        }
    }

    encoder_pub_.publish(js_msg);
}

void HighSpeedEncoderRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    for (int i = 0; i < static_cast<int>(enc_data_to_pub_.size()); ++i)
    {
        publishLatest(i);
    }
}

void HighSpeedEncoderRosI::positionChangeHandler(int channel,
                                                 int position_change,
                                                 double time,
                                                 int /* index_triggered */)
{
    if (static_cast<int>(enc_data_to_pub_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);

        double instantaneous_speed = position_change / (time * 1e-6);
        enc_data_to_pub_[channel].instantaneous_speed = instantaneous_speed;
        enc_data_to_pub_[channel].speeds_buffer.push_back(instantaneous_speed);
        enc_data_to_pub_[channel].speed_buffer_updated = true;
        enc_data_to_pub_[channel].loops_without_update_speed_buffer = 0;

        if (publish_rate_ <= 0)
        {
            publishLatest(channel);
        }
    }
}

}  // namespace phidgets
