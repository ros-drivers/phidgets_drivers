#include <ros/ros.h>

#include "phidgets_digital_outputs/digital_outputs_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsDigitalOutputs");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::DigitalOutputsRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
