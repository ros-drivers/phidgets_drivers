#ifndef PHIDGETS_MAGNETOMETER_MAGNETOMETER_ROS_I_H
#define PHIDGETS_MAGNETOMETER_MAGNETOMETER_ROS_I_H

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "phidgets_api/magnetometer.h"

namespace phidgets {

const double MAX_TIMEDIFF_SECONDS = 0.1;

class MagnetometerRosI final
{
  public:
    explicit MagnetometerRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Magnetometer> magnetometer_;
    std::string frame_id_;
    double magnetic_field_stdev_;
    std::mutex mag_mutex_;
    double last_mag_x_;
    double last_mag_y_;
    double last_mag_z_;
    double last_mag_timestamp_;
    double mag_time_zero_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher magnetometer_pub_;
    ros::Time ros_time_zero_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest();

    void magnetometerChangeCallback(const double magnetic_field[3],
                                    const double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_MAGNETOMETER_MAGNETOMETER_ROS_I_H
