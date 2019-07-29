#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "phidgets_high_speed_encoder/high_speed_encoder_ros_i.h"
#include "phidgets_high_speed_encoder/phidgets_high_speed_encoder_nodelet.h"

typedef phidgets::PhidgetsHighSpeedEncoderNodelet
    PhidgetsHighSpeedEncoderNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsHighSpeedEncoderNodelet, nodelet::Nodelet)

void PhidgetsHighSpeedEncoderNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets High Speed Encoder Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    enc_ = std::make_unique<HighSpeedEncoderRosI>(nh, nh_private);
}
