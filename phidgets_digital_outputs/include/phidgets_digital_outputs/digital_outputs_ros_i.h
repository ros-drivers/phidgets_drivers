#ifndef PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_H
#define PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "phidgets_api/digital_outputs.h"
#include "phidgets_msgs/SetDigitalOutput.h"

namespace phidgets {

class DigitalOutputSetter final
{
  public:
    explicit DigitalOutputSetter(DigitalOutputs* dos, int index,
                                 ros::NodeHandle nh,
                                 const std::string& topicname);

  private:
    void setMsgCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber subscription_;
    DigitalOutputs* dos_;
    int index_;
};

class DigitalOutputsRosI final
{
  public:
    explicit DigitalOutputsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<DigitalOutputs> dos_;
    std::vector<std::unique_ptr<DigitalOutputSetter>> out_subs_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::ServiceServer out_srv_;

    bool setSrvCallback(phidgets_msgs::SetDigitalOutput::Request& req,
                        phidgets_msgs::SetDigitalOutput::Response& res);
};

}  // namespace phidgets

#endif  // PHIDGETS_DIGITAL_OUTPUTS_DIGITAL_OUTPUTS_ROS_I_H
