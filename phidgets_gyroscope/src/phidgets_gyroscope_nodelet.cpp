#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_gyroscope/gyroscope_ros_i.h"
#include "phidgets_gyroscope/phidgets_gyroscope_nodelet.h"

typedef phidgets::PhidgetsGyroscopeNodelet PhidgetsGyroscopeNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsGyroscopeNodelet, nodelet::Nodelet)

void PhidgetsGyroscopeNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Gyroscope Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    gyroscope_ = std::make_unique<GyroscopeRosI>(nh, nh_private);
}
