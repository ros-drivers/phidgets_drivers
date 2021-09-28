#include <memory>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_api/analog_outputs.h"
#include "phidgets_analog_outputs/analog_outputs_ros_i.h"
#include "phidgets_msgs/SetAnalogOutput.h"

namespace phidgets {

AnalogOutputsRosI::AnalogOutputsRosI(ros::NodeHandle nh,
                                     ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Analog Outputs");

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
    bool is_hub_port_device;
    if (!nh_private.getParam("is_hub_port_device", is_hub_port_device))
    {
        // only used if the device is on a VINT hub_port
        is_hub_port_device = false;
    }

    ROS_INFO("Connecting to Phidgets AnalogOutputs serial %d, hub port %d ...",
             serial_num, hub_port);

    try
    {
        aos_ = std::make_unique<AnalogOutputs>(serial_num, hub_port,
                                               is_hub_port_device);

    } catch (const Phidget22Error& err)
    {
        ROS_ERROR("AnalogOutputs: %s", err.what());
        throw;
    }

    int n_out = aos_->getOutputCount();
    ROS_INFO("Connected %d outputs", n_out);
    out_subs_.resize(n_out);
    for (int i = 0; i < n_out; i++)
    {
        char topicname[] = "analog_output00";
        snprintf(topicname, sizeof(topicname), "analog_output%02d", i);
        out_subs_[i] =
            std::make_unique<AnalogOutputSetter>(aos_.get(), i, nh, topicname);
    }
    out_srv_ = nh_.advertiseService("set_analog_output",
                                    &AnalogOutputsRosI::setSrvCallback, this);
}

bool AnalogOutputsRosI::setSrvCallback(
    phidgets_msgs::SetAnalogOutput::Request& req,
    phidgets_msgs::SetAnalogOutput::Response& res)
{
    aos_->setOutputVoltage(req.index, req.voltage);
    res.success = true;
    return true;
}

AnalogOutputSetter::AnalogOutputSetter(AnalogOutputs* aos, int index,
                                       ros::NodeHandle nh,
                                       const std::string& topicname)
    : aos_(aos), index_(index)
{
    subscription_ =
        nh.subscribe(topicname, 1, &AnalogOutputSetter::setMsgCallback, this);
}

void AnalogOutputSetter::setMsgCallback(const std_msgs::Float64::ConstPtr& msg)
{
    aos_->setOutputVoltage(index_, msg->data);
}

}  // namespace phidgets
