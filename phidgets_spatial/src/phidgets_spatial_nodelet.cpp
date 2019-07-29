#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_spatial/phidgets_spatial_nodelet.h"
#include "phidgets_spatial/spatial_ros_i.h"

typedef phidgets::PhidgetsSpatialNodelet PhidgetsSpatialNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsSpatialNodelet, nodelet::Nodelet)

void PhidgetsSpatialNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets SPATIAL Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    spatial_ = std::make_unique<SpatialRosI>(nh, nh_private);
}
