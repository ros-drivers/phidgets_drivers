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

#ifndef PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
#define PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "phidgets_api/encoders.h"

namespace phidgets {

struct EncoderDataToPub {
    double instantaneous_speed = .0;
    std::vector<double> speeds_buffer;
    bool speed_buffer_updated = false;
    int loops_without_update_speed_buffer = 0;
    std::string joint_name;
    double joint_tick2rad;
    ros::Publisher encoder_decimspeed_pub;
};

class HighSpeedEncoderRosI final
{
  public:
    explicit HighSpeedEncoderRosI(ros::NodeHandle nh,
                                  ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Encoders> encs_;
    std::mutex encoder_mutex_;
    /// Size of this vector = number of found encoders.
    std::vector<EncoderDataToPub> enc_data_to_pub_;
    std::string frame_id_;
    /// (Default=10) Number of samples for the sliding window average filter of
    /// speeds.
    int speed_filter_samples_len_ = 10;
    /// (Default=1) Number of "ITERATE" loops without any new encoder tick
    /// before resetting the filtered average velocities.
    int speed_filter_idle_iter_loops_before_reset_ = 1;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher encoder_pub_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;
    std::string server_name_;
    std::string server_ip_;

    /// Publish the latest state for all encoder channels:
    void publishLatest();

    void positionChangeHandler(int channel, int position_change, double time,
                               int index_triggered);
};
}  // namespace phidgets

#endif  // PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
