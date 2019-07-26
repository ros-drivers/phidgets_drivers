#ifndef PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_H
#define PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_H

#include <memory>
#include <mutex>

#include <ros/ros.h>

#include "phidgets_api/temperature.h"

namespace phidgets {

class TemperatureRosI final
{
  public:
    explicit TemperatureRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Temperature> temperature_;
    std::mutex temperature_mutex_;
    double last_temperature_reading_;
    bool got_first_data_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher temperature_pub_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest();

    void temperatureChangeCallback(double temperature);
};

}  // namespace phidgets

#endif  // PHIDGETS_TEMPERATURE_TEMPERATURE_ROS_I_H
