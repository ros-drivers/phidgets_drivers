#include <ros/ros.h>

#include "phidgets_gyroscope/gyroscope_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsGyroscope");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::GyroscopeRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
