#ifndef PHIDGETS_ANALOG_INPUTS_ANALOG_INPUTS_ROS_I_H
#define PHIDGETS_ANALOG_INPUTS_ANALOG_INPUTS_ROS_I_H

#include <memory>
#include <mutex>
#include <vector>

#include <ros/ros.h>

#include "phidgets_api/analog_inputs.h"

namespace phidgets {

struct ValToPub {
    ros::Publisher pub;
    double last_val;
};

class AnalogInputsRosI final
{
  public:
    explicit AnalogInputsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<AnalogInputs> ais_;
    std::mutex ai_mutex_;
    std::vector<ValToPub> val_to_pubs_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest(int index);

    void sensorChangeCallback(int index, double sensor_value);
};

}  // namespace phidgets

#endif  // PHIDGETS_ANALOG_INPUTS_ANALOG_INPUTS_ROS_I_H
