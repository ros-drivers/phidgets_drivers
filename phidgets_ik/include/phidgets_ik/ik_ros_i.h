#ifndef PHIDGETS_IK_IK_ROS_I_H
#define PHIDGETS_IK_IK_ROS_I_H

#include <ros/ros.h>
#include <phidgets_api/ik.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "phidgets_ik/SetDigitalOutput.h"

#include <vector>
#include <boost/shared_ptr.hpp>

namespace phidgets {

class OutputSetter {
  public:
    OutputSetter(CPhidgetInterfaceKitHandle ik_handle, int index);
    virtual void set_msg_callback(const std_msgs::Bool::ConstPtr& msg);
    ros::Subscriber subscription;
  protected:
    int index;
    CPhidgetInterfaceKitHandle ik_handle_;
};

class IKRosI : public IK
{

  public:

    IKRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

  protected:

    int n_in;
    int n_out;
    int n_sensors;
    std::vector<ros::Publisher> in_pubs_;
    std::vector<ros::Publisher> sensor_pubs_;
    ros::ServiceServer out_srv_;
    std::vector<boost::shared_ptr<OutputSetter> > out_subs_;

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    const float VREF;

    void initDevice();
    void sensorHandler(int index, int sensorValue);
    void inputHandler(int index, int inputValue);

    bool set_srv_callback(phidgets_ik::SetDigitalOutput::Request& req, phidgets_ik::SetDigitalOutput::Response &res);
};

} //namespace phidgets

#endif // PHIDGETS_IK_IK_ROS_I_H
