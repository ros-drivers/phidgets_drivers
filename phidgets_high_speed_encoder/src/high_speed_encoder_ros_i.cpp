#include <numeric>
#include <string>
#include <vector>

#include <sensor_msgs/JointState.h>

#include "phidgets_high_speed_encoder/high_speed_encoder_ros_i.h"
#include "phidgets_msgs/EncoderDecimatedSpeed.h"

namespace phidgets {

HighSpeedEncoderRosI::HighSpeedEncoderRosI(ros::NodeHandle nh,
                                           ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    joint_names_ = {"joint0", "joint1", "joint2", "joint3"};
    joint_tick2rad_ = {1.0, 1.0, 1.0, 1.0};
    for (unsigned int i = 0; i < joint_names_.size(); i++)
    {
        char str[100];
        sprintf(str, "joint%u_name", i);
        nh_private_.getParam(str, joint_names_[i]);

        sprintf(str, "joint%u_tick2rad", i);
        nh_private_.getParam(str, joint_tick2rad_[i]);

        ROS_INFO("Channel %u Name: '%s', Tick2rad: '%f'", i,
                 joint_names_[i].c_str(), joint_tick2rad_[i]);
    }

    int serial_number = -1;
    if (serial_number == -1)
    {
        nh_private_.getParam("serial_number", serial_number);
    }

    nh_private_.getParam("frame_id", frame_id_);
    ROS_INFO("frame_id = '%s'", frame_id_.c_str());
    int publish_rate;
    nh_private_.param("PUBLISH_RATE", publish_rate, 20);
    nh_private_.param("SPEED_FILTER_SAMPLES_LEN", speed_filter_samples_len_,
                      10);
    nh_private_.param("SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET",
                      speed_filter_idle_iter_loops_before_reset_, 1);

    // First time: create publishers:
    const std::string topic_path = "joint_states";
    ROS_INFO("Publishing state to topic: %s", topic_path.c_str());

    encoder_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_path, 100);

    // get the program to wait for an encoder device to be attached
    if (serial_number == -1)
    {
        ROS_INFO("Waiting for any High Speed Encoder Phidget to be attached.");
    } else
    {
        ROS_INFO("Waiting for High Speed Encoder Phidget %d to be attached....",
                 serial_number);
    }

    // open the device:
    const int result = openAndWaitForAttachment(serial_number, 10000);

    if (result != 0)
    {
        std::string sError = Phidget::getErrorDescription(result);
        ROS_FATAL("Problem waiting for attachment: %s", sError.c_str());
        ros::shutdown();
        return;
    }

    display_properties();

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate),
                             &HighSpeedEncoderRosI::timerCallback, this);
}

void HighSpeedEncoderRosI::timerCallback(const ros::TimerEvent & /* event */)
{
    size_t N;
    {
        std::lock_guard<std::mutex> lock(encoder_states_mutex_);
        N = encoder_states_.size();
        if (N == 0)
        {
            return;
        }
    }

    sensor_msgs::JointState js_msg;
    static uint32_t seq_cnt = 0;
    js_msg.header.seq = (seq_cnt++);
    js_msg.header.stamp = ros::Time::now();
    js_msg.header.frame_id = frame_id_;

    js_msg.name.resize(N);
    for (unsigned int i = 0; i < std::min<size_t>(joint_names_.size(), N); i++)
    {
        js_msg.name[i] = joint_names_[i];
    }

    js_msg.position.resize(N);
    js_msg.velocity.resize(N);
    js_msg.effort.clear();

    for (unsigned int i = 0; i < N; i++)
    {
        TStatePerChannel &spc = encoder_states_[i];

        js_msg.position[i] = spc.tickPos * joint_tick2rad_[i];
        js_msg.velocity[i] = spc.instantaneousSpeed * joint_tick2rad_[i];

        spc.instantaneousSpeed = 0;  // Reset speed

        if (speed_filter_samples_len_ > 0)
        {
            if (!spc.speed_buffer_updated)
            {
                if (++spc.loops_without_update_speed_buffer >=
                    speed_filter_idle_iter_loops_before_reset_)
                {
                    phidgets_msgs::EncoderDecimatedSpeed e;
                    e.header.stamp = ros::Time::now();
                    e.header.frame_id = frame_id_;
                    e.avr_speed = .0;
                    encoder_decimspeed_pubs_[i].publish(e);
                }
            } else
            {
                spc.loops_without_update_speed_buffer = 0;

                if (spc.speeds_buffer.size() >=
                    static_cast<size_t>(speed_filter_samples_len_))
                {
                    const double avrg =
                        std::accumulate(spc.speeds_buffer.begin(),
                                        spc.speeds_buffer.end(), 0.0) /
                        spc.speeds_buffer.size();
                    spc.speeds_buffer.clear();

                    phidgets_msgs::EncoderDecimatedSpeed e;
                    e.header.stamp = ros::Time::now();
                    e.header.frame_id = frame_id_;
                    e.avr_speed = avrg * joint_tick2rad_[i];
                    encoder_decimspeed_pubs_[i].publish(e);
                }
            }
        }
    }

    encoder_pub_.publish(js_msg);
}

void HighSpeedEncoderRosI::display_properties()
{
    const int serial_number = Phidget::getDeviceSerialNumber();
    const int version = Phidget::getDeviceVersion();
    const int num_encoders = Encoder::getEncoderCount();
    const std::string dev_type = Phidget::getDeviceType();
    const int num_inputs = Encoder::getInputCount();

    ROS_INFO("Device type       : %s", dev_type.c_str());
    ROS_INFO("Device serial     : %d", serial_number);
    ROS_INFO("Device version    : %d", version);
    ROS_INFO("Number of encoders: %d", num_encoders);
    ROS_INFO("Number of inputs  : %d", num_inputs);
}

void HighSpeedEncoderRosI::attachHandler()
{
    const int serial_number = Phidget::getDeviceSerialNumber();
    const std::string name = Phidget::getDeviceName();
    const int inputcount = Encoder::getEncoderCount();
    const std::string topic_path = "joint_states";

    {
        std::lock_guard<std::mutex> lock(encoder_states_mutex_);
        encoder_states_.resize(inputcount);
        encoder_decimspeed_pubs_.resize(inputcount);
        for (int i = 0; i < inputcount; i++)
        {
            std::string s = topic_path;
            char buf[100];
            sprintf(buf, "_ch%u_decim_speed", i);
            s += buf;
            ROS_INFO("Publishing decimated speed of channel %u to topic: %s", i,
                     s.c_str());
            encoder_decimspeed_pubs_[i] =
                nh_.advertise<phidgets_msgs::EncoderDecimatedSpeed>(s, 10);
        }
    }
    ROS_INFO("%s Serial number %d attached with %d encoders!", name.c_str(),
             serial_number, inputcount);

    // the 1047 requires enabling of the encoder inputs, so enable them if this
    // is a 1047
    for (int i = 0; i < inputcount; i++)
    {
        Encoder::setEnabled(i, true);
    }
}

void HighSpeedEncoderRosI::detachHandler()
{
    const int serial_number = Phidget::getDeviceSerialNumber();
    const std::string name = Phidget::getDeviceName();
    ROS_INFO("%s Serial number %d detached!", name.c_str(), serial_number);

    std::lock_guard<std::mutex> lock(encoder_states_mutex_);
    encoder_states_.resize(0);
    encoder_decimspeed_pubs_.resize(0);
}

void HighSpeedEncoderRosI::errorHandler(int errorCode)
{
    ROS_ERROR("Error handled. %d - %s", errorCode,
              Phidget::getErrorDescription(errorCode).c_str());
}

void HighSpeedEncoderRosI::positionChangeHandler(int index, int time,
                                                 int positionChange)
{
    const int Position = Encoder::getPosition(index);
    ROS_DEBUG("Encoder %d Count %d", index, Position);

    std::lock_guard<std::mutex> lock(encoder_states_mutex_);
    if (index < (int)encoder_states_.size())
    {
        TStatePerChannel &spc = encoder_states_[index];
        spc.tickPos = Position;
        spc.instantaneousSpeed = positionChange / (time * 1e-6);
        spc.speeds_buffer.push_back(spc.instantaneousSpeed);
        spc.speed_buffer_updated = true;
        spc.loops_without_update_speed_buffer = 0;
    }
}

}  // namespace phidgets
