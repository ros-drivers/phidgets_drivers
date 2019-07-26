#ifndef PHIDGETS_ACCELEROMETER_ACCELEROMETER_ROS_I_H
#define PHIDGETS_ACCELEROMETER_ACCELEROMETER_ROS_I_H

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "phidgets_api/accelerometer.h"

namespace phidgets {

const double G = 9.81;
const double MAX_TIMEDIFF_SECONDS = 0.1;

class AccelerometerRosI final
{
  public:
    explicit AccelerometerRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Accelerometer> accelerometer_;
    std::string frame_id_;
    double linear_acceleration_stdev_;
    std::mutex accel_mutex_;
    double last_accel_x_;
    double last_accel_y_;
    double last_accel_z_;
    double last_accel_timestamp_;
    double accel_time_zero_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher accelerometer_pub_;
    ros::Time ros_time_zero_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest();

    void accelerometerChangeCallback(const double acceleration[3],
                                     const double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_ACCELEROMETER_ACCELEROMETER_ROS_I_H
