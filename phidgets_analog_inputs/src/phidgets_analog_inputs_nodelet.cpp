#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_analog_inputs/analog_inputs_ros_i.h"
#include "phidgets_analog_inputs/phidgets_analog_inputs_nodelet.h"

typedef phidgets::PhidgetsAnalogInputsNodelet PhidgetsAnalogInputsNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsAnalogInputsNodelet, nodelet::Nodelet)

void PhidgetsAnalogInputsNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Analog Inputs Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    ais_ = std::make_unique<AnalogInputsRosI>(nh, nh_private);
}
