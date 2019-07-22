#include <ros/ros.h>

#include "phidgets_digital_inputs/digital_inputs_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsDigitalInputs");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::DigitalInputsRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
