#include <ros/ros.h>

#include "phidgets_temperature/temperature_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsTemperature");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::TemperatureRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
