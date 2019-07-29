#ifndef PHIDGETS_SPATIAL_SPATIAL_ROS_I_H
#define PHIDGETS_SPATIAL_SPATIAL_ROS_I_H

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

#include "phidgets_api/spatial.h"

namespace phidgets {

const float G = 9.81;
const float MAX_TIMEDIFF_SECONDS = 0.1;

class SpatialRosI final
{
  public:
    explicit SpatialRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Time ros_time_zero_;
    std::string frame_id_;
    std::mutex spatial_mutex_;
    void timerCallback(const ros::TimerEvent &event);
    ros::Timer timer_;
    int publish_rate_;
    ros::Publisher cal_publisher_;
    ros::ServiceServer cal_srv_;

    ros::Publisher imu_pub_;
    bool got_first_data_;
    double data_time_zero_;
    double last_data_timestamp_;

    ros::Publisher magnetic_field_pub_;

    void calibrate();

    bool calibrateService(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);

    std::unique_ptr<Spatial> spatial_;

    // Accelerometer
    double linear_acceleration_stdev_;
    double last_accel_x_;
    double last_accel_y_;
    double last_accel_z_;

    // Gyroscope
    double angular_velocity_stdev_;
    double last_gyro_x_;
    double last_gyro_y_;
    double last_gyro_z_;

    // Magnetometer
    double magnetic_field_stdev_;
    double last_mag_x_;
    double last_mag_y_;
    double last_mag_z_;

    void publishLatest();

    void spatialDataCallback(const double acceleration[3],
                             const double angular_rate[3],
                             const double magnetic_field[3], double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_SPATIAL_SPATIAL_ROS_I_H
