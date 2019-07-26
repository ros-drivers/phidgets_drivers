#include <functional>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "phidgets_temperature/temperature_ros_i.h"

namespace phidgets {

TemperatureRosI::TemperatureRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets Temperature");

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
    int thermocouple_type;
    if (!nh_private.getParam("thermocouple_type", thermocouple_type))
    {
        thermocouple_type = 0;
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 500;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 5;
    }

    ROS_INFO(
        "Waiting for Phidgets Temperature serial %d, hub port %d, thermocouple "
        "type %d to be attached...",
        serial_num, hub_port, thermocouple_type);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(temperature_mutex_);

    temperature_ = std::make_unique<Temperature>(
        serial_num, hub_port, false,
        std::bind(&TemperatureRosI::temperatureChangeCallback, this,
                  std::placeholders::_1));

    ROS_INFO("Connected");

    temperature_->setDataInterval(data_interval_ms);

    if (thermocouple_type != 0)
    {
        temperature_->setThermocoupleType(
            static_cast<ThermocoupleType>(thermocouple_type));
    }

    temperature_pub_ = nh_.advertise<std_msgs::Float64>("temperature", 1);

    got_first_data_ = false;

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &TemperatureRosI::timerCallback, this);
    } else
    {
        // We'd like to get the temperature on startup here, but it can take
        // some some time for the setThermocoupleType() call above to complete
        // down on the sensor.  Instead, we wait for the first change callback
        // before we start publishing.
    }
}

void TemperatureRosI::publishLatest()
{
    std_msgs::Float64 msg;
    msg.data = last_temperature_reading_;
    temperature_pub_.publish(msg);
}

void TemperatureRosI::timerCallback(const ros::TimerEvent& /* event */)
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    if (got_first_data_)
    {
        publishLatest();
    }
}

void TemperatureRosI::temperatureChangeCallback(double temperature)
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    last_temperature_reading_ = temperature;

    if (!got_first_data_)
    {
        got_first_data_ = true;
    }

    if (publish_rate_ <= 0)
    {
        publishLatest();
    }
}

}  // namespace phidgets
