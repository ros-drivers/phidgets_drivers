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
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include "phidgets_gyroscope/gyroscope_ros_i.h"

namespace phidgets {

GyroscopeRosI::GyroscopeRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Gyroscope");

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
    if (!nh_private_.getParam("angular_velocity_stdev",
                              angular_velocity_stdev_))
    {
        angular_velocity_stdev_ =
            0.02 * (M_PI / 180.0);  // 0.02 deg/s resolution, as per manual
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

    ROS_INFO("Connecting to Phidgets Gyroscope serial %d, hub port %d ...",
             serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(gyro_mutex_);

    try
    {
        gyroscope_ = std::make_unique<Gyroscope>(
            serial_num, hub_port, false,
            std::bind(&GyroscopeRosI::gyroscopeChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        ROS_INFO("Connected");

        gyroscope_->setDataInterval(data_interval_ms);

        cal_publisher_ = nh_.advertise<std_msgs::Bool>("imu/is_calibrated", 5);

        calibrate();

        gyroscope_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

        cal_srv_ = nh_.advertiseService("imu/calibrate",
                                        &GyroscopeRosI::calibrateService, this);

        gyroscope_->getAngularRate(last_gyro_x_, last_gyro_y_, last_gyro_z_,
                                   gyro_time_zero_);
        last_gyro_timestamp_ = gyro_time_zero_;
        ros_time_zero_ = ros::Time::now();
    } catch (const Phidget22Error &err)
    {
        ROS_ERROR("Gyroscope: %s", err.what());
        throw;
    }

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &GyroscopeRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        publishLatest();
    }
}

void GyroscopeRosI::calibrate()
{
    ROS_INFO("Calibrating IMU...");
    gyroscope_->zero();
    ROS_INFO("Calibrating IMU done.");

    // publish message
    std_msgs::Bool is_calibrated_msg;
    is_calibrated_msg.data = true;
    cal_publisher_.publish(is_calibrated_msg);
}

bool GyroscopeRosI::calibrateService(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
    (void)req;
    (void)res;
    calibrate();
    return true;
}

void GyroscopeRosI::publishLatest()
{
    std::shared_ptr<sensor_msgs::Imu> msg =
        std::make_shared<sensor_msgs::Imu>();

    msg->header.frame_id = frame_id_;

    double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                msg->angular_velocity_covariance[idx] = ang_vel_var;
            } else
            {
                msg->angular_velocity_covariance[idx] = 0.0;
            }
        }
    }

    double gyro_diff_in_secs =
        (last_gyro_timestamp_ - gyro_time_zero_) / 1000.0;
    double time_in_secs = ros_time_zero_.toSec() + gyro_diff_in_secs;

    msg->header.stamp = ros::Time(time_in_secs);

    // set angular velocities
    msg->angular_velocity.x = last_gyro_x_ * (M_PI / 180.0);
    msg->angular_velocity.y = last_gyro_y_ * (M_PI / 180.0);
    msg->angular_velocity.z = last_gyro_z_ * (M_PI / 180.0);

    gyroscope_pub_.publish(*msg);
}

void GyroscopeRosI::timerCallback(const ros::TimerEvent & /* event */)
{
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    publishLatest();
}

void GyroscopeRosI::gyroscopeChangeCallback(const double angular_rate[3],
                                            double timestamp)
{
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    last_gyro_x_ = angular_rate[0];
    last_gyro_y_ = angular_rate[1];
    last_gyro_z_ = angular_rate[2];
    last_gyro_timestamp_ = timestamp;

    if (publish_rate_ <= 0)
    {
        publishLatest();
    }
}

}  // namespace phidgets
