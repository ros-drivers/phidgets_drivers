#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_ik/ik_ros_i.h"
#include "phidgets_ik/phidgets_ik_nodelet.h"

typedef phidgets::PhidgetsIKNodelet PhidgetsIKNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsIKNodelet, nodelet::Nodelet)

void PhidgetsIKNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets IK Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    ik_ = std::make_unique<IKRosI>(nh, nh_private);
}
