#ifndef PHIDGETS_IMU_PHIDGETS_IMU
#define PHIDGETS_IMU_PHIDGETS_IMU

#include <phidgets_api/phidget21.h>

#include <ros/ros.h>
#include <ros/service_server.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace phidgets {

const float G = 9.81;

class PhidgetsImu
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef geometry_msgs::Vector3Stamped MagMsg;

  public:

    PhidgetsImu(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~PhidgetsImu();

    // needs to be public because of the C API of phidgets
    void processImuData(CPhidgetSpatial_SpatialEventDataHandle * data, int count);

    // **** services

    bool calibrate(std_srvs::Empty::Request  &req,
                   std_srvs::Empty::Response &res);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher  imu_publisher_;
    ros::Publisher  mag_publisher_;
    ros::Publisher  cal_publisher_;
    ros::ServiceServer cal_srv_;

    // **** variables

    bool initialized_;
    boost::mutex mutex_;
    tf::Quaternion orientation_;
    ros::Time last_imu_time_;
	  CPhidgetSpatialHandle handle_id_;
  
    ImuMsg imu_msg_;

    // **** paramaters

    int rate_;  // rate in ms

    std::string frame_id_;

    double angular_velocity_stdev_;
    double linear_acceleration_stdev_;
    
    // **** methods

    void initDevice();
    void zero();
};

//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);

} //namespace phidgets

#endif // PHIDGETS_IMU_PHIDGETS_IMU
