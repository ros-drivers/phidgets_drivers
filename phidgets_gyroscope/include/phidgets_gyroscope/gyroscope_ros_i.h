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

#ifndef PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H
#define PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "phidgets_api/gyroscope.h"

namespace phidgets {

class GyroscopeRosI final
{
  public:
    explicit GyroscopeRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Gyroscope> gyroscope_;
    std::string frame_id_;
    double angular_velocity_variance_;
    std::mutex gyro_mutex_;
    double last_gyro_x_;
    double last_gyro_y_;
    double last_gyro_z_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher cal_publisher_;
    ros::ServiceServer cal_srv_;
    ros::Publisher gyroscope_pub_;
    void timerCallback(const ros::TimerEvent &event);
    ros::Timer timer_;
    int publish_rate_;
    std::string server_name_;
    std::string server_ip_;

    ros::Time ros_time_zero_;
    bool synchronize_timestamps_{true};
    uint64_t data_time_zero_ns_{0};
    uint64_t last_data_timestamp_ns_{0};
    uint64_t last_ros_stamp_ns_{0};
    int64_t time_resync_interval_ns_{0};
    int64_t data_interval_ns_{0};
    bool can_publish_{false};
    ros::Time last_cb_time_;
    int64_t cb_delta_epsilon_ns_{0};

    void publishLatest();

    void calibrate();

    bool calibrateService(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);

    void gyroscopeChangeCallback(const double angular_rate[3],
                                 const double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H
