#include <memory>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "phidgets_api/digital_outputs.h"
#include "phidgets_digital_outputs/digital_outputs_ros_i.h"
#include "phidgets_msgs/SetDigitalOutput.h"

namespace phidgets {

DigitalOutputsRosI::DigitalOutputsRosI(ros::NodeHandle nh,
                                       ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Digital Outputs");

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

    ROS_INFO("Connecting to Phidgets DigitalOutputs serial %d, hub port %d ...",
             serial_num, hub_port);

    try
    {
        dos_ = std::make_unique<DigitalOutputs>(serial_num, hub_port,
                                                is_hub_port_device);

    } catch (const Phidget22Error& err)
    {
        ROS_ERROR("DigitalOutputs: %s", err.what());
        throw;
    }

    int n_out = dos_->getOutputCount();
    ROS_INFO("Connected %d outputs", n_out);
    out_subs_.resize(n_out);
    for (int i = 0; i < n_out; i++)
    {
        char topicname[] = "digital_output00";
        snprintf(topicname, sizeof(topicname), "digital_output%02d", i);
        out_subs_[i] =
            std::make_unique<DigitalOutputSetter>(dos_.get(), i, nh, topicname);
    }
    out_srv_ = nh_.advertiseService("set_digital_output",
                                    &DigitalOutputsRosI::setSrvCallback, this);
}

bool DigitalOutputsRosI::setSrvCallback(
    phidgets_msgs::SetDigitalOutput::Request& req,
    phidgets_msgs::SetDigitalOutput::Response& res)
{
    dos_->setOutputState(req.index, req.state);
    res.success = true;
    return true;
}

DigitalOutputSetter::DigitalOutputSetter(DigitalOutputs* dos, int index,
                                         ros::NodeHandle nh,
                                         const std::string& topicname)
    : dos_(dos), index_(index)
{
    subscription_ =
        nh.subscribe(topicname, 1, &DigitalOutputSetter::setMsgCallback, this);
}

void DigitalOutputSetter::setMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
    dos_->setOutputState(index_, msg->data);
}

}  // namespace phidgets
