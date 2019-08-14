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

#ifndef PHIDGETS_IK_IK_ROS_I_H
#define PHIDGETS_IK_IK_ROS_I_H

#include <memory>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "phidgets_api/analog_inputs.h"
#include "phidgets_api/digital_inputs.h"
#include "phidgets_api/digital_outputs.h"
#include "phidgets_msgs/SetDigitalOutput.h"

namespace phidgets {

class IKDigitalOutputSetter
{
  public:
    explicit IKDigitalOutputSetter(DigitalOutputs* dos, int index,
                                   ros::NodeHandle nh,
                                   const std::string& topicname);

  protected:
    void setMsgCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber subscription_;
    DigitalOutputs* dos_;
    int index_;
};

class IKRosI final
{
  public:
    explicit IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::mutex ik_mutex_;
    std::vector<bool> last_di_vals_;
    std::vector<double> last_ai_vals_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    // Digital inputs
    std::unique_ptr<DigitalInputs> dis_;
    std::vector<ros::Publisher> di_pubs_;
    void stateChangeCallback(int index, int input_value);
    void publishLatestDI(int index);

    // Digital outputs
    std::unique_ptr<DigitalOutputs> dos_;
    ros::ServiceServer out_srv_;
    std::vector<std::unique_ptr<IKDigitalOutputSetter>> out_subs_;

    bool setSrvCallback(phidgets_msgs::SetDigitalOutput::Request& req,
                        phidgets_msgs::SetDigitalOutput::Response& res);

    // Analog inputs
    std::unique_ptr<AnalogInputs> ais_;
    std::vector<ros::Publisher> ai_pubs_;
    void sensorChangeCallback(int index, double sensor_value);
    void publishLatestAI(int index);
};

}  // namespace phidgets

#endif  // PHIDGETS_IK_IK_ROS_I_H
