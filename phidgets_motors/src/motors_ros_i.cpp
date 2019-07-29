#include <functional>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_api/motors.h"
#include "phidgets_motors/motors_ros_i.h"

namespace phidgets {

MotorsRosI::MotorsRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Motors");

    int serial_num;
    if (!nh_private_.getParam("serial", serial_num))
    {
        serial_num = -1;  // default open any device
    }
    int hub_port;
    if (!nh_private.getParam("hub_port", hub_port))
    {
        hub_port = 0;  // only used if the device is on a VINT hub_port
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 256;
    }
    double braking_strength;
    if (!nh_private.getParam("braking_strength", braking_strength))
    {
        braking_strength = 0.0;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 5;
    }

    ROS_INFO(
        "Waiting for Phidgets Motors serial %d, hub port %d to be "
        "attached...",
        serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(motor_mutex_);

    motors_ = std::make_unique<Motors>(
        serial_num, hub_port, false,
        std::bind(&MotorsRosI::dutyCycleChangeCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotorsRosI::backEMFChangeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    ROS_INFO("Connected");

    int n_motors = motors_->getMotorCount();
    motor_vals_.resize(n_motors);
    for (int i = 0; i < n_motors; i++)
    {
        char topicname[] = "set_motor_duty_cycle00";
        snprintf(topicname, sizeof(topicname), "set_motor_duty_cycle%02d", i);
        motor_vals_[i].duty_cycle_sub =
            std::make_unique<DutyCycleSetter>(motors_.get(), i, nh, topicname);

        char pubtopic[] = "motor_duty_cycle00";
        snprintf(pubtopic, sizeof(pubtopic), "motor_duty_cycle%02d", i);
        motor_vals_[i].duty_cycle_pub =
            nh_.advertise<std_msgs::Float64>(pubtopic, 1);

        char backemftopic[] = "motor_back_emf00";
        snprintf(backemftopic, sizeof(backemftopic), "motor_back_emf%02d", i);
        motor_vals_[i].back_emf_pub =
            nh_.advertise<std_msgs::Float64>(backemftopic, 1);

        motor_vals_[i].last_duty_cycle_val = motors_->getDutyCycle(i);
        motor_vals_[i].last_back_emf_val = motors_->getBackEMF(i);

        motors_->setDataInterval(i, data_interval_ms);
        motors_->setBraking(i, braking_strength);
    }

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &MotorsRosI::timerCallback, this);
    } else
    {
        // If we are *not* publishing periodically, then we are event driven and
        // will only publish when something changes (where "changes" is defined
        // by the libphidget22 library).  In that case, make sure to publish
        // once at the beginning to make sure there is *some* data.
        for (int i = 0; i < n_motors; ++i)
        {
            publishLatestDutyCycle(i);
            publishLatestBackEMF(i);
        }
    }
}

void MotorsRosI::publishLatestDutyCycle(int index)
{
    std_msgs::Float64 msg;
    msg.data = motor_vals_[index].last_duty_cycle_val;
    motor_vals_[index].duty_cycle_pub.publish(msg);
}

void MotorsRosI::publishLatestBackEMF(int index)
{
    std_msgs::Float64 backemf_msg;
    backemf_msg.data = motor_vals_[index].last_back_emf_val;
    motor_vals_[index].back_emf_pub.publish(backemf_msg);
}

void MotorsRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (int i = 0; i < static_cast<int>(motor_vals_.size()); ++i)
    {
        publishLatestDutyCycle(i);
        publishLatestBackEMF(i);
    }
}

DutyCycleSetter::DutyCycleSetter(Motors* motors, int index, ros::NodeHandle nh,
                                 const std::string& topicname)
    : motors_(motors), index_(index)
{
    subscription_ =
        nh.subscribe(topicname, 1, &DutyCycleSetter::setMsgCallback, this);
}

void DutyCycleSetter::setMsgCallback(const std_msgs::Float64::ConstPtr& msg)
{
    try
    {
        motors_->setDutyCycle(index_, msg->data);
    } catch (const phidgets::Phidget22Error& err)
    {
        // If the data was wrong, the lower layers will throw an exception; just
        // catch and ignore here so we don't crash the node.
    }
}

void MotorsRosI::dutyCycleChangeCallback(int channel, double duty_cycle)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_duty_cycle_val = duty_cycle;

        if (publish_rate_ <= 0)
        {
            publishLatestDutyCycle(channel);
        }
    }
}

void MotorsRosI::backEMFChangeCallback(int channel, double back_emf)
{
    if (static_cast<int>(motor_vals_.size()) > channel)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motor_vals_[channel].last_back_emf_val = back_emf;

        if (publish_rate_ <= 0)
        {
            publishLatestBackEMF(channel);
        }
    }
}

}  // namespace phidgets
