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

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include "phidgets_magnetometer/magnetometer_ros_i.h"

namespace phidgets {

MagnetometerRosI::MagnetometerRosI(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Magnetometer");

    ROS_INFO("Opening device");
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
        // As specified in http://www.ros.org/reps/rep-0145.html
        frame_id_ = "imu_link";
    }
    if (!nh_private_.getParam("magnetic_field_stdev", magnetic_field_stdev_))
    {
        magnetic_field_stdev_ =
            0.095 * (M_PI / 180.0);  // 0.095°/s as per manual
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 8;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 0;
    }

    // compass correction params (see
    // http://www.phidgets.com/docs/1044_User_Guide)
    double cc_mag_field;
    double cc_offset0;
    double cc_offset1;
    double cc_offset2;
    double cc_gain0;
    double cc_gain1;
    double cc_gain2;
    double cc_T0;
    double cc_T1;
    double cc_T2;
    double cc_T3;
    double cc_T4;
    double cc_T5;

    bool has_compass_params =
        nh_private_.getParam("cc_mag_field", cc_mag_field) &&
        nh_private_.getParam("cc_offset0", cc_offset0) &&
        nh_private_.getParam("cc_offset1", cc_offset1) &&
        nh_private_.getParam("cc_offset2", cc_offset2) &&
        nh_private_.getParam("cc_gain0", cc_gain0) &&
        nh_private_.getParam("cc_gain1", cc_gain1) &&
        nh_private_.getParam("cc_gain2", cc_gain2) &&
        nh_private_.getParam("cc_t0", cc_T0) &&
        nh_private_.getParam("cc_t1", cc_T1) &&
        nh_private_.getParam("cc_t2", cc_T2) &&
        nh_private_.getParam("cc_t3", cc_T3) &&
        nh_private_.getParam("cc_t4", cc_T4) &&
        nh_private_.getParam("cc_t5", cc_T5);

    ROS_INFO("Connecting to Phidgets Magnetometer serial %d, hub port %d ...",
             serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(mag_mutex_);

    magnetometer_ = std::make_unique<Magnetometer>(
        serial_num, hub_port, false,
        std::bind(&MagnetometerRosI::magnetometerChangeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    ROS_INFO("Connected");

    magnetometer_->setDataInterval(data_interval_ms);

    if (has_compass_params)
    {
        magnetometer_->setCompassCorrectionParameters(
            cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
            cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
    } else
    {
        ROS_INFO("No compass correction params found.");
    }

    magnetometer_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);

    magnetometer_->getMagneticField(last_mag_x_, last_mag_y_, last_mag_z_,
                                    mag_time_zero_);
    last_mag_timestamp_ = mag_time_zero_;
    ros_time_zero_ = ros::Time::now();

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &MagnetometerRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        publishLatest();
    }
}

void MagnetometerRosI::publishLatest()
{
    std::shared_ptr<sensor_msgs::MagneticField> msg =
        std::make_shared<sensor_msgs::MagneticField>();

    msg->header.frame_id = frame_id_;

    // build covariance matrix

    double mag_field_var = magnetic_field_stdev_ * magnetic_field_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                msg->magnetic_field_covariance[idx] = mag_field_var;
            } else
            {
                msg->magnetic_field_covariance[idx] = 0.0;
            }
        }
    }

    double mag_diff_in_secs = (last_mag_timestamp_ - mag_time_zero_) / 1000.0;
    double time_in_secs = ros_time_zero_.toSec() + mag_diff_in_secs;

    msg->header.stamp = ros::Time(time_in_secs);

    // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
    msg->magnetic_field.x = last_mag_x_ * 1e-4;
    msg->magnetic_field.y = last_mag_y_ * 1e-4;
    msg->magnetic_field.z = last_mag_z_ * 1e-4;

    magnetometer_pub_.publish(*msg);
}

void MagnetometerRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(mag_mutex_);
    publishLatest();
}

void MagnetometerRosI::magnetometerChangeCallback(
    const double magnetic_field[3], double timestamp)
{
    std::lock_guard<std::mutex> lock(mag_mutex_);
    last_mag_x_ = magnetic_field[0];
    last_mag_y_ = magnetic_field[1];
    last_mag_z_ = magnetic_field[2];
    last_mag_timestamp_ = timestamp;

    if (publish_rate_ <= 0)
    {
        publishLatest();
    }
}

}  // namespace phidgets
