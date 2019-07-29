#include "phidgets_spatial/spatial_ros_i.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PhidgetsSpatial");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    phidgets::SpatialRosI spatial(nh, nh_private);
    ros::spin();
    return 0;
}
