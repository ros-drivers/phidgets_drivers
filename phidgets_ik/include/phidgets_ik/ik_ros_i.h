#ifndef PHIDGETS_IK_IK_ROS_I_H
#define PHIDGETS_IK_IK_ROS_I_H

#include <memory>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "phidgets_api/analog_inputs.h"
#include "phidgets_api/digital_inputs.h"
#include "phidgets_api/digital_outputs.h"
#include "phidgets_msgs/SetDigitalOutput.h"

namespace phidgets {

class IKDigitalOutputSetter
{
  public:
    explicit IKDigitalOutputSetter(DigitalOutputs* dos, int index,
                                   ros::NodeHandle nh,
                                   const std::string& topicname);

  protected:
    void setMsgCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber subscription_;
    DigitalOutputs* dos_;
    int index_;
};

class IKRosI final
{
  public:
    explicit IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    std::mutex ik_mutex_;
    std::vector<bool> last_di_vals_;
    std::vector<double> last_ai_vals_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    // Digital inputs
    std::unique_ptr<DigitalInputs> dis_;
    std::vector<ros::Publisher> di_pubs_;
    void stateChangeCallback(int index, int input_value);
    void publishLatestDI(int index);

    // Digital outputs
    std::unique_ptr<DigitalOutputs> dos_;
    ros::ServiceServer out_srv_;
    std::vector<std::unique_ptr<IKDigitalOutputSetter>> out_subs_;

    bool setSrvCallback(phidgets_msgs::SetDigitalOutput::Request& req,
                        phidgets_msgs::SetDigitalOutput::Response& res);

    // Analog inputs
    std::unique_ptr<AnalogInputs> ais_;
    std::vector<ros::Publisher> ai_pubs_;
    void sensorChangeCallback(int index, double sensor_value);
    void publishLatestAI(int index);
};

}  // namespace phidgets

#endif  // PHIDGETS_IK_IK_ROS_I_H
