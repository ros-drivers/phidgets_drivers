#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_analog_outputs/analog_outputs_ros_i.h"
#include "phidgets_analog_outputs/phidgets_analog_outputs_nodelet.h"

typedef phidgets::PhidgetsAnalogOutputsNodelet PhidgetsAnalogOutputsNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsAnalogOutputsNodelet, nodelet::Nodelet)

void PhidgetsAnalogOutputsNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Analog Outputs Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    aos_ = std::make_unique<AnalogOutputsRosI>(nh, nh_private);
}
