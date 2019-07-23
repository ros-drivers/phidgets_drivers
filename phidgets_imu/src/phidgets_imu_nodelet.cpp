#include "phidgets_imu/phidgets_imu_nodelet.h"

typedef phidgets::PhidgetsImuNodelet PhidgetsImuNodelet;

PLUGINLIB_EXPORT_CLASS(PhidgetsImuNodelet, nodelet::Nodelet)

void PhidgetsImuNodelet::onInit()
{
    NODELET_INFO("Initializing Phidgets IMU Nodelet");

    // TODO: Do we want the single threaded or multithreaded NH?
    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    imu_ = new ImuRosI(nh, nh_private);
}
