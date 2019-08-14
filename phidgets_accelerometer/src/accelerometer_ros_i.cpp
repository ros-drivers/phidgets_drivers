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
#include <sensor_msgs/Imu.h>

#include "phidgets_accelerometer/accelerometer_ros_i.h"

namespace phidgets {

AccelerometerRosI::AccelerometerRosI(ros::NodeHandle nh,
                                     ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Accelerometer");

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
    if (!nh_private_.getParam("linear_acceleration_stdev",
                              linear_acceleration_stdev_))
    {
        linear_acceleration_stdev_ = 300.0 * 1e-6 * G;  // 300 ug as per manual
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 250;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 5;
    }

    ROS_INFO(
        "Waiting for Phidgets Accelerometer serial %d, hub port %d to be "
        "attached...",
        serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(accel_mutex_);

    accelerometer_ = std::make_unique<Accelerometer>(
        serial_num, hub_port, false,
        std::bind(&AccelerometerRosI::accelerometerChangeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    ROS_INFO("Connected");

    accelerometer_->setDataInterval(data_interval_ms);

    accelerometer_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

    accelerometer_->getAcceleration(last_accel_x_, last_accel_y_, last_accel_z_,
                                    accel_time_zero_);
    last_accel_timestamp_ = accel_time_zero_;
    ros_time_zero_ = ros::Time::now();

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &AccelerometerRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        publishLatest();
    }
}

void AccelerometerRosI::publishLatest()
{
    std::shared_ptr<sensor_msgs::Imu> msg =
        std::make_shared<sensor_msgs::Imu>();

    msg->header.frame_id = frame_id_;

    double lin_acc_var =
        linear_acceleration_stdev_ * linear_acceleration_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                msg->linear_acceleration_covariance[idx] = lin_acc_var;
            } else
            {
                msg->linear_acceleration_covariance[idx] = 0.0;
            }
        }
    }

    double accel_diff_in_secs =
        (last_accel_timestamp_ - accel_time_zero_) / 1000.0;
    double time_in_secs = ros_time_zero_.toSec() + accel_diff_in_secs;

    msg->header.stamp = ros::Time(time_in_secs);

    // set linear acceleration
    msg->linear_acceleration.x = -last_accel_x_ * G;
    msg->linear_acceleration.y = -last_accel_y_ * G;
    msg->linear_acceleration.z = -last_accel_z_ * G;

    accelerometer_pub_.publish(*msg);
}

void AccelerometerRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(accel_mutex_);
    publishLatest();
}

void AccelerometerRosI::accelerometerChangeCallback(
    const double acceleration[3], double timestamp)
{
    std::lock_guard<std::mutex> lock(accel_mutex_);
    last_accel_x_ = acceleration[0];
    last_accel_y_ = acceleration[1];
    last_accel_z_ = acceleration[2];
    last_accel_timestamp_ = timestamp;

    if (publish_rate_ <= 0)
    {
        publishLatest();
    }
}

}  // namespace phidgets
