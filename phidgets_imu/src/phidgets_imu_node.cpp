#include "phidgets_imu/phidgets_imu.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsImu");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::PhidgetsImu imu(nh, nh_private);
  ros::spin();
  return 0;
}
