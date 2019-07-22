#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_digital_inputs/digital_inputs_ros_i.h"
#include "phidgets_digital_inputs/phidgets_digital_inputs_nodelet.h"

typedef phidgets::PhidgetsDigitalInputsNodelet PhidgetsDigitalInputsNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsDigitalInputsNodelet, nodelet::Nodelet)

void PhidgetsDigitalInputsNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Digital Inputs Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    dis_ = std::make_unique<DigitalInputsRosI>(nh, nh_private);
}
