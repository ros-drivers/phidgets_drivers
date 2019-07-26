#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_temperature/phidgets_temperature_nodelet.h"
#include "phidgets_temperature/temperature_ros_i.h"

typedef phidgets::PhidgetsTemperatureNodelet PhidgetsTemperatureNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsTemperatureNodelet, nodelet::Nodelet)

void PhidgetsTemperatureNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Temperature Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    temperature_ = std::make_unique<TemperatureRosI>(nh, nh_private);
}
