#include <ros/ros.h>

#include "phidgets_accelerometer/accelerometer_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsAccelerometer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::AccelerometerRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
