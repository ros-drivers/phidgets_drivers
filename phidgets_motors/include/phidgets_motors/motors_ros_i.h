#ifndef PHIDGETS_MOTORS_MOTORS_ROS_I_H
#define PHIDGETS_MOTORS_MOTORS_ROS_I_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_api/motors.h"

namespace phidgets {

class DutyCycleSetter final
{
  public:
    explicit DutyCycleSetter(Motors* motors, int index, ros::NodeHandle nh,
                             const std::string& topicname);

  private:
    void setMsgCallback(const std_msgs::Float64::ConstPtr& msg);
    ros::Subscriber subscription_;
    Motors* motors_;
    int index_;
};

struct MotorVals {
    std::unique_ptr<DutyCycleSetter> duty_cycle_sub;
    double last_duty_cycle_val;
    double last_back_emf_val;
    ros::Publisher duty_cycle_pub;
    ros::Publisher back_emf_pub;
};

class MotorsRosI final
{
  public:
    explicit MotorsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Motors> motors_;
    std::mutex motor_mutex_;
    std::vector<MotorVals> motor_vals_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatestDutyCycle(int index);
    void publishLatestBackEMF(int index);

    void dutyCycleChangeCallback(int channel, double duty_cycle);

    void backEMFChangeCallback(int channel, double back_emf);
};

}  // namespace phidgets

#endif  // PHIDGETS_MOTORS_MOTORS_ROS_I_H
