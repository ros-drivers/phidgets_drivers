#include <ros/ros.h>

#include "phidgets_motors/motors_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsMotors");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::MotorsRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
