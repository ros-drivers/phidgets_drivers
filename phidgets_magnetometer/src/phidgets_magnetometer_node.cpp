#include <ros/ros.h>

#include "phidgets_magnetometer/magnetometer_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsMagnetometer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::MagnetometerRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
