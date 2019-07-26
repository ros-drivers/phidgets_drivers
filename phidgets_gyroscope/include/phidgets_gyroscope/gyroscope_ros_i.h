#ifndef PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H
#define PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "phidgets_api/gyroscope.h"

namespace phidgets {

const double MAX_TIMEDIFF_SECONDS = 0.1;

class GyroscopeRosI final
{
  public:
    explicit GyroscopeRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Gyroscope> gyroscope_;
    std::string frame_id_;
    double angular_velocity_stdev_;
    std::mutex gyro_mutex_;
    double last_gyro_x_;
    double last_gyro_y_;
    double last_gyro_z_;
    double last_gyro_timestamp_;
    double gyro_time_zero_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher cal_publisher_;
    ros::ServiceServer cal_srv_;
    ros::Publisher gyroscope_pub_;
    ros::Time ros_time_zero_;
    void timerCallback(const ros::TimerEvent &event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest();

    void calibrate();

    bool calibrateService(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);

    void gyroscopeChangeCallback(const double angular_rate[3],
                                 const double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_GYROSCOPE_GYROSCOPE_ROS_I_H
