#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_magnetometer/magnetometer_ros_i.h"
#include "phidgets_magnetometer/phidgets_magnetometer_nodelet.h"

typedef phidgets::PhidgetsMagnetometerNodelet PhidgetsMagnetometerNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsMagnetometerNodelet, nodelet::Nodelet)

void PhidgetsMagnetometerNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Magnetometer Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    magnetometer_ = std::make_unique<MagnetometerRosI>(nh, nh_private);
}
