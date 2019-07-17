#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_digital_outputs/digital_outputs_ros_i.h"
#include "phidgets_digital_outputs/phidgets_digital_outputs_nodelet.h"

typedef phidgets::PhidgetsDigitalOutputsNodelet PhidgetsDigitalOutputsNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsDigitalOutputsNodelet, nodelet::Nodelet)

void PhidgetsDigitalOutputsNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Digital Outputs Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    dos_ = std::make_unique<DigitalOutputsRosI>(nh, nh_private);
}
