/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets high speed encoder
 *  Copyright (c) 2010, Bob Mottram
 *  Copyright (c) 2017, Jose Luis Blanco Claraco
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <std_msgs/String.h>
#include <vector>
#include <mutex>
#include <numeric> // std::accumulate()
#include "phidgets_api/encoder.h"

#include <sensor_msgs/JointState.h>
#include "phidgets_high_speed_encoder/EncoderDecimatedSpeed.h"

// (Default=20 Hz) Rate for publishing encoder states
double PUBLISH_RATE = 20; // [Hz]
// (Default=10) Number of samples for the sliding window average filter of speeds.
int SPEED_FILTER_SAMPLES_LEN = 10;
// (Default=1) Number of "ITERATE" loops without any new encoder tick before resetting the filtered average velocities.
int SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET = 1;

struct TStatePerChannel
{
  int tickPos = .0;
  double instantaneousSpeed = .0;
  std::vector<double> speeds_buffer;
  bool speed_buffer_updated = false;
  unsigned int  loops_without_update_speed_buffer = 0;
};
std::vector<TStatePerChannel>  encoder_states;
std::mutex  encoder_states_mux;

/** Derived type from Encoder so we can override virtual event handlers. */
class EncoderNode : public phidgets::Encoder
{
public:
  EncoderNode() : phidgets::Encoder()
  {
  }

  void display_properties()
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

protected:
  void attachHandler() override
  {
    const int serial_number = Phidget::getDeviceSerialNumber();
    const std::string name = Phidget::getDeviceName();
    const int inputcount = Encoder::getEncoderCount();

    {
      std::lock_guard<std::mutex> lock(encoder_states_mux);
      encoder_states.resize(inputcount);
    }
    ROS_INFO("%s Serial number %d attached with %d encoders!",
             name.c_str(), serial_number, inputcount);

    //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047
    for (int i = 0 ; i < inputcount ; i++)
      Encoder::setEnabled(i, true);
  }

  void detachHandler() override
  {
    const int serial_number = Phidget::getDeviceSerialNumber();
    const std::string name = Phidget::getDeviceName();
    ROS_INFO("%s Serial number %d detached!",
             name.c_str(), serial_number);
  }

  void errorHandler(int ErrorCode) override
  {
    ROS_ERROR("Error handled. %d - %s", ErrorCode,
              Phidget::getErrorDescription(ErrorCode).c_str());
  }

  void positionChangeHandler(int index, int time, int positionChange) override
  {
    const int Position = Encoder::getPosition(index);
    ROS_DEBUG("Encoder %d Count %d", index, Position);

    std::lock_guard<std::mutex> lock(encoder_states_mux);
    if (index < (int)encoder_states.size())
    {
      TStatePerChannel &spc = encoder_states[index];
      spc.tickPos = Position;
      spc.instantaneousSpeed = positionChange / (time * 1e-6);
      spc.speeds_buffer.push_back(spc.instantaneousSpeed);
      spc.speed_buffer_updated = true;
      spc.loops_without_update_speed_buffer = 0;
    }
  }

  void indexHandler(int index, int indexPosition) override
  {

  }


}; // end EncoderNode

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "phidgets_high_speed_encoder");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::vector<std::string> joint_names = { "joint0", "joint1", "joint2", "joint3"};
  std::vector<double>      joint_tick2rad = { 4, 1.0};
  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    char str[100];
    sprintf(str, "joint%u_name", i);
    nh.getParam(str, joint_names[i]);

    sprintf(str, "joint%u_tick2rad", i);
    nh.getParam(str, joint_tick2rad[i]);

    ROS_INFO("Channel %u: '%s'='%s'", i, str, joint_names[i].c_str());
  }

  int serial_number = -1;
  if (serial_number == -1)
  {
    nh.getParam("serial_number", serial_number);
  }

  std::string frame_id;
  nh.getParam("frame_id", frame_id);
  ROS_INFO("frame_id = '%s'", frame_id.c_str());
  nh.getParam("PUBLISH_RATE", PUBLISH_RATE);
  nh.getParam("SPEED_FILTER_SAMPLES_LEN", SPEED_FILTER_SAMPLES_LEN);
  nh.getParam("SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET", SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET);

  // First time: create publishers:
  const std::string topic_path = "joint_states";
  ROS_INFO("Publishing state to topic: %s", topic_path.c_str());

  ros::Publisher encoder_pub = n.advertise<sensor_msgs::JointState>(topic_path, 100);
  std::vector<ros::Publisher> encoder_decimspeed_pubs;

  // The encoder object instance:
  EncoderNode  encoder_node;

  //open the device:
  encoder_node.open(serial_number);

  // get the program to wait for an encoder device to be attached
  if (serial_number == -1)
  {
    ROS_INFO("Waiting for any High Speed Encoder Phidget to be attached.");
  }
  else
  {
    ROS_INFO("Waiting for High Speed Encoder Phidget %d to be attached....",
             serial_number);
  }

  const int result = encoder_node.waitForAttachment(10000);

  if (result != 0)
  {
    const auto sError = encoder_node.getErrorDescription(result);
    ROS_ERROR("Problem waiting for attachment: %s", sError.c_str());
    return 1; // !=0 exit status means error in main()
  }

  encoder_node.display_properties();

  ros::Rate rate(PUBLISH_RATE);
  ROS_INFO("Publishing encoder states at %.03f Hz", PUBLISH_RATE);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    // Publish:
    {
      std::lock_guard<std::mutex> lock(encoder_states_mux);
      const unsigned int N = encoder_states.size();

      // First time? We need to create decimated speed publisher here,
      // once we know how many channel we have. Also, they must be independent
      // for each channel due to the unsynchronous nature of the filtering
      // algorithm:
      if (encoder_decimspeed_pubs.size() != N)
      {
        encoder_decimspeed_pubs.resize(N);
        for (unsigned int i = 0; i < N; i++)
        {
          std::string s = topic_path;
          char buf[100];
          sprintf(buf, "_ch%u_decim_speed", i);
          s += buf;
          ROS_INFO("Publishing decimated speed of channel %u to topic: %s", i, s.c_str());
          encoder_decimspeed_pubs[i] =
            n.advertise<phidgets_high_speed_encoder::EncoderDecimatedSpeed>(
              s,
              10);
        }
      }

      sensor_msgs::JointState js_msg;
      static uint32_t seq_cnt = 0;
      js_msg.header.seq = (seq_cnt++);
      js_msg.header.stamp = ros::Time::now();
      js_msg.header.frame_id = frame_id;

      js_msg.name.resize(N);
      for (unsigned int i = 0; i < std::min<size_t>(joint_names.size(), N); i++)
        js_msg.name[i] = joint_names[i];

      js_msg.position.resize(N);
      js_msg.velocity.resize(N);
      js_msg.effort.clear();

      for (unsigned int i = 0; i < N; i++)
      {
        TStatePerChannel &spc = encoder_states[i];

        js_msg.position[i] = spc.tickPos * joint_tick2rad[i];
        js_msg.velocity[i] = spc.instantaneousSpeed * joint_tick2rad[i];

        spc.instantaneousSpeed = 0; // Reset speed

        if (SPEED_FILTER_SAMPLES_LEN > 0)
        {
          if (!spc.speed_buffer_updated)
          {
            if (int(++spc.loops_without_update_speed_buffer) >= SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET)
            {
              phidgets_high_speed_encoder::EncoderDecimatedSpeed e;
              e.header.stamp = ros::Time::now();
              e.header.frame_id = frame_id;
              e.avr_speed = .0;
              encoder_decimspeed_pubs[i].publish(e);
            }
          }
          else
          {
            spc.loops_without_update_speed_buffer = 0;

            if (int(spc.speeds_buffer.size()) >= SPEED_FILTER_SAMPLES_LEN)
            {
              const double avrg = std::accumulate(spc.speeds_buffer.begin(), spc.speeds_buffer.end(), 0.0) / spc.speeds_buffer.size();
              spc.speeds_buffer.clear();

              phidgets_high_speed_encoder::EncoderDecimatedSpeed e;
              e.header.stamp = ros::Time::now();
              e.header.frame_id = frame_id;
              e.avr_speed = avrg * joint_tick2rad[i];
              encoder_decimspeed_pubs[i].publish(e);
            }
          }
        }
      }

      encoder_pub.publish(js_msg);

    } // end lock guard
  } // end while ros::ok()

  return 0;
}
