#include "phidgets_ik/ik_ros_i.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "PhidgetsIK");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  phidgets::IKRosI ik(nh, nh_private);
  ros::spin();
  return 0;
}
