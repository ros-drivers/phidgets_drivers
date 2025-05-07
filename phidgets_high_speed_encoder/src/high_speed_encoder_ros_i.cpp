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
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "phidgets_high_speed_encoder/high_speed_encoder_ros_i.hpp"
#include "phidgets_msgs/msg/encoder_decimated_speed.hpp"

namespace phidgets {

HighSpeedEncoderRosI::HighSpeedEncoderRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_high_speed_encoder_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Encoders");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    frame_id_ = this->declare_parameter("frame_id", "encoder_link");

    speed_filter_samples_len_ =
        this->declare_parameter("speed_filter_samples_len", 10);

    speed_filter_idle_iter_loops_before_reset_ =
        this->declare_parameter("speed_filter_idle_iter_loops_before_reset", 1);

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
                "Connecting to Phidgets Encoders serial %d, hub port %d ...",
                serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(encoder_mutex_);

    uint32_t n_encs;
    try
    {
        encs_ = std::make_unique<Encoders>(
            serial_num, hub_port, false,
            std::bind(&HighSpeedEncoderRosI::positionChangeHandler, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));

        n_encs = encs_->getEncoderCount();
        RCLCPP_INFO(get_logger(), "Connected to serial %d, %u encoders",
                    encs_->getSerialNumber(), n_encs);
        enc_data_to_pub_.resize(n_encs);
        for (uint32_t i = 0; i < n_encs; i++)
        {
            char str[100];
            sprintf(str, "joint%u_name", i);
            enc_data_to_pub_[i].joint_name =
                this->declare_parameter(str, "joint" + std::to_string(i));

            sprintf(str, "joint%u_tick2rad", i);
            enc_data_to_pub_[i].joint_tick2rad =
                this->declare_parameter(str, 1.0);

            RCLCPP_INFO(get_logger(), "Channel %u: '%s'='%s'", i, str,
                        enc_data_to_pub_[i].joint_name.c_str());

            char buf[100];
            sprintf(buf, "joint_states_ch%u_decim_speed", i);
            RCLCPP_INFO(get_logger(),
                        "Publishing decimated speed of channel %u to topic: %s",
                        i, buf);
            enc_data_to_pub_[i].encoder_decimspeed_pub = this->create_publisher<
                phidgets_msgs::msg::EncoderDecimatedSpeed>(buf, 10);
            encs_->setEnabled(i, true);
        }
    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Encoders: %s", err.what());
        throw;
    }

    encoder_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 100);

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&HighSpeedEncoderRosI::timerCallback, this));
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        publishLatest();
    }
}

void HighSpeedEncoderRosI::publishLatest()
{
    auto js_msg = std::make_unique<sensor_msgs::msg::JointState>();
    js_msg->header.stamp = this->now();
    js_msg->header.frame_id = frame_id_;

    const auto numEncoders = enc_data_to_pub_.size();

    js_msg->name.resize(numEncoders);
    for (size_t i = 0; i < numEncoders; ++i)
    {
        js_msg->name[i] = enc_data_to_pub_[i].joint_name;
    }

    js_msg->position.resize(numEncoders);
    js_msg->velocity.resize(numEncoders);
    js_msg->effort.clear();

    for (size_t encIdx = 0; encIdx < numEncoders; ++encIdx)
    {
        int64_t absolute_position =
            encs_->getPosition(encIdx) - encs_->getIndexPosition(encIdx);

        js_msg->position[encIdx] =
            absolute_position * enc_data_to_pub_[encIdx].joint_tick2rad;
        js_msg->velocity[encIdx] =
            enc_data_to_pub_[encIdx].instantaneous_speed *
            enc_data_to_pub_[encIdx].joint_tick2rad;
        enc_data_to_pub_[encIdx].instantaneous_speed = 0.0;  // Reset speed

        if (speed_filter_samples_len_ > 0)
        {
            if (!enc_data_to_pub_[encIdx].speed_buffer_updated)
            {
                if (++enc_data_to_pub_[encIdx]
                          .loops_without_update_speed_buffer >=
                    speed_filter_idle_iter_loops_before_reset_)
                {
                    auto e = std::make_unique<
                        phidgets_msgs::msg::EncoderDecimatedSpeed>();
                    e->header.stamp = this->now();
                    e->header.frame_id = frame_id_;
                    e->avr_speed = .0;
                    enc_data_to_pub_[encIdx].encoder_decimspeed_pub->publish(
                        std::move(e));
                }
            } else
            {
                enc_data_to_pub_[encIdx].loops_without_update_speed_buffer = 0;

                if (enc_data_to_pub_[encIdx].speeds_buffer.size() >=
                    static_cast<size_t>(speed_filter_samples_len_))
                {
                    const double avrg =
                        std::accumulate(
                            enc_data_to_pub_[encIdx].speeds_buffer.begin(),
                            enc_data_to_pub_[encIdx].speeds_buffer.end(), 0.0) /
                        enc_data_to_pub_[encIdx].speeds_buffer.size();
                    enc_data_to_pub_[encIdx].speeds_buffer.clear();

                    auto e = std::make_unique<
                        phidgets_msgs::msg::EncoderDecimatedSpeed>();
                    e->header.stamp = this->now();
                    e->header.frame_id = frame_id_;
                    e->avr_speed =
                        avrg * enc_data_to_pub_[encIdx].joint_tick2rad;
                    enc_data_to_pub_[encIdx].encoder_decimspeed_pub->publish(
                        std::move(e));
                }
            }
        }
    }

    encoder_pub_->publish(std::move(js_msg));
}

void HighSpeedEncoderRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    publishLatest();
}

void HighSpeedEncoderRosI::positionChangeHandler(int channel,
                                                 int position_change,
                                                 double time,
                                                 int /*index_triggered*/)
{
    if (channel >= static_cast<int>(enc_data_to_pub_.size())) return;

    {
        std::lock_guard<std::mutex> lock(encoder_mutex_);

        double instantaneous_speed = position_change / (time * 1e-3);
        enc_data_to_pub_[channel].instantaneous_speed = instantaneous_speed;
        enc_data_to_pub_[channel].speeds_buffer.push_back(instantaneous_speed);
        enc_data_to_pub_[channel].speed_buffer_updated = true;
        enc_data_to_pub_[channel].loops_without_update_speed_buffer = 0;
    }

    if (publish_rate_ <= 0) publishLatest();
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::HighSpeedEncoderRosI)
