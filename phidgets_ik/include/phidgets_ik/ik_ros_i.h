#ifndef PHIDGETS_IK_IK_ROS_I_H
#define PHIDGETS_IK_IK_ROS_I_H

#include <phidgets_api/ik.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "phidgets_msgs/SetDigitalOutput.h"

#include <memory>
#include <vector>

namespace phidgets {

class OutputSetter
{
  public:
    explicit OutputSetter(IK* ik, int index);
    void set_msg_callback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber subscription;

  protected:
    IK* ik_;
    int index_;
};

class IKRosI final : public IK
{
  public:
    explicit IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  private:
    int n_in;
    int n_out;
    int n_sensors;
    std::vector<ros::Publisher> in_pubs_;
    std::vector<ros::Publisher> sensor_pubs_;
    ros::ServiceServer out_srv_;
    std::vector<std::shared_ptr<OutputSetter> > out_subs_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    const float VREF;

    void initDevice();
    void sensorHandler(int index, int sensorValue) override;
    void inputHandler(int index, int inputValue) override;

    bool set_srv_callback(phidgets_msgs::SetDigitalOutput::Request& req,
                          phidgets_msgs::SetDigitalOutput::Response& res);
};

}  // namespace phidgets

#endif  // PHIDGETS_IK_IK_ROS_I_H
