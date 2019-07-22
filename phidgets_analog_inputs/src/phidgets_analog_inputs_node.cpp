#include <ros/ros.h>

#include "phidgets_analog_inputs/analog_inputs_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsAnalogInputs");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::AnalogInputsRosI ik(nh, nh_private);
    ros::spin();
    return 0;
}
