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

#ifndef PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_HPP
#define PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "phidgets_api/digital_outputs.hpp"
#include "phidgets_msgs/srv/set_digital_output.hpp"

namespace phidgets {

class DigitalOutputsRosI;

class DigitalOutputSetter final
{
  public:
    explicit DigitalOutputSetter(DigitalOutputs* dos, int index,
                                 DigitalOutputsRosI* node,
                                 const std::string& topicname);

  private:
    void setMsgCallback(const std_msgs::msg::Bool::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    DigitalOutputs* dos_;
    int index_;
};

class DigitalOutputsRosI final : public rclcpp::Node
{
  public:
    explicit DigitalOutputsRosI(const rclcpp::NodeOptions& options);

  private:
    std::unique_ptr<DigitalOutputs> dos_;
    std::vector<std::unique_ptr<DigitalOutputSetter>> out_subs_;

    rclcpp::Service<phidgets_msgs::srv::SetDigitalOutput>::SharedPtr out_srv_;
    std::string server_name_;
    std::string server_ip_;

    void setSrvCallback(
        const std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request>
            req,
        std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Response> res);
};

}  // namespace phidgets

#endif  // PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_HPP
