#include "phidgets_ik/phidgets_ik_nodelet.h"

typedef phidgets::PhidgetsIKNodelet PhidgetsIKNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsIKNodelet, nodelet::Nodelet)

void PhidgetsIKNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets IK Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    ik_ = new IKRosI(nh, nh_private);
}
