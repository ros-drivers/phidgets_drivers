#ifndef PHIDGETS_DIGITAL_INPUTS_DIGITAL_INPUTS_ROS_I_H
#define PHIDGETS_DIGITAL_INPUTS_DIGITAL_INPUTS_ROS_I_H

#include <memory>
#include <mutex>
#include <vector>

#include <ros/ros.h>

#include "phidgets_api/digital_inputs.h"

namespace phidgets {

struct ValToPub {
    ros::Publisher pub;
    bool last_val;
};

class DigitalInputsRosI final
{
  public:
    explicit DigitalInputsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<DigitalInputs> dis_;
    std::mutex di_mutex_;
    std::vector<ValToPub> val_to_pubs_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest(int index);

    void stateChangeCallback(int index, int input_value);
};

}  // namespace phidgets

#endif  // PHIDGETS_DIGITAL_INPUTS_DIGITAL_INPUTS_ROS_I_H
