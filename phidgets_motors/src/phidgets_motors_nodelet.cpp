#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_motors/motors_ros_i.h"
#include "phidgets_motors/phidgets_motors_nodelet.h"

typedef phidgets::PhidgetsMotorsNodelet PhidgetsMotorsNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsMotorsNodelet, nodelet::Nodelet)

void PhidgetsMotorsNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets Motors Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    motors_ = std::make_unique<MotorsRosI>(nh, nh_private);
}
