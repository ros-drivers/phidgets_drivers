#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_accelerometer/accelerometer_ros_i.h"
#include "phidgets_accelerometer/phidgets_accelerometer_nodelet.h"

typedef phidgets::PhidgetsAccelerometerNodelet PhidgetsAccelerometerNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsAccelerometerNodelet, nodelet::Nodelet)

void PhidgetsAccelerometerNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Accelerometer Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    accelerometer_ = std::make_unique<AccelerometerRosI>(nh, nh_private);
}
