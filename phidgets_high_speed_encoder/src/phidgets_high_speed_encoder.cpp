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
#include <libphidgets/phidget21.h>
#include <std_msgs/String.h>
#include <vector>
#include <mutex>
#include <numeric> // std::accumulate()
#include "phidgets_high_speed_encoder/EncoderParams.h"
#include "phidgets_high_speed_encoder/EncoderDecimatedSpeed.h"

static CPhidgetEncoderHandle phid;
std::vector<ros::Publisher> encoder_pubs, encoder_decimspeed_pubs;
static std::string frame_id;
static bool inverted = false;
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


int AttachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName(phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);

  CPhidget_DeviceID deviceID;
  //Retrieve the device ID and number of encoders so that we can set the enables if needed
  CPhidget_getDeviceID(phid, &deviceID);
  int inputcount = 0;
  CPhidgetEncoder_getEncoderCount((CPhidgetEncoderHandle)phid, &inputcount);

  {
    std::lock_guard<std::mutex> lock(encoder_states_mux);
    encoder_states.resize(inputcount);
  }
  ROS_INFO("%s Serial number %d attached with %d encoders!",
           name, serial_number, inputcount);

  //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047
  if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
  {
    for (int i = 0 ; i < inputcount ; i++)
      CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)phid, i, 1);
  }

  return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName(phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d detached!",
           name, serial_number);

  return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
                 int ErrorCode, const char *Description)
{
  ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
  return 0;
}

int InputChangeHandler(CPhidgetEncoderHandle ENC,
                       void *usrptr, int Index, int State)
{
  return 0;
}

int PositionChangeHandler(CPhidgetEncoderHandle ENC,
                          void *usrptr, int Index,
                          int Time, int RelativePosition)
{
  int Position;
  CPhidgetEncoder_getPosition(ENC, Index, &Position);
  ROS_DEBUG("Encoder %d Count %d", Index, Position);

  std::lock_guard<std::mutex> lock(encoder_states_mux);
  if (Index < encoder_states.size())
  {
    TStatePerChannel &spc = encoder_states[Index];
    spc.tickPos = Position;
    spc.instantaneousSpeed = RelativePosition / (Time * 1e-6);
    spc.speeds_buffer.push_back(spc.instantaneousSpeed);
    spc.speed_buffer_updated = true;
    spc.loops_without_update_speed_buffer = 0;
  }

  return 0;
}

int display_properties(CPhidgetEncoderHandle phid)
{
  int serial_number, version, num_encoders, num_inputs;
  const char *ptr;

  CPhidget_getDeviceType((CPhidgetHandle) phid, &ptr);
  CPhidget_getSerialNumber((CPhidgetHandle) phid,
                           &serial_number);
  CPhidget_getDeviceVersion((CPhidgetHandle) phid, &version);

  CPhidgetEncoder_getInputCount(phid, &num_inputs);
  CPhidgetEncoder_getEncoderCount(phid, &num_encoders);

  ROS_INFO("%s", ptr);
  ROS_INFO("Serial Number: %d", serial_number);
  ROS_INFO("Version: %d", version);
  ROS_INFO("Number of encoders %d", num_encoders);

  return 0;
}

bool attach(
  CPhidgetEncoderHandle &phid,
  int serial_number)
{
  // create the object
  CPhidgetEncoder_create(&phid);

  // Set the handlers to be run when the device is
  // plugged in or opened from software, unplugged or
  // closed from software, or generates an error.
  CPhidget_set_OnAttach_Handler((CPhidgetHandle) phid,
                                AttachHandler, NULL);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle) phid,
                                DetachHandler, NULL);
  CPhidget_set_OnError_Handler((CPhidgetHandle) phid,
                               ErrorHandler, NULL);

  // Registers a callback that will run if an input changes.
  // Requires the handle for the Phidget, the function
  // that will be called, and an arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetEncoder_set_OnInputChange_Handler(phid,
      InputChangeHandler,
      NULL);

  // Registers a callback that will run if the
  // encoder changes.
  // Requires the handle for the Encoder, the function
  // that will be called, and an arbitrary pointer that
  // will be supplied to the callback function (may be NULL).
  CPhidgetEncoder_set_OnPositionChange_Handler(phid,
      PositionChangeHandler,
      NULL);

  //open the device for connections
  CPhidget_open((CPhidgetHandle) phid, serial_number);

  // get the program to wait for an encoder device
  // to be attached
  if (serial_number == -1)
  {
    ROS_INFO("Waiting for High Speed Encoder Phidget " \
             "to be attached....");
  }
  else
  {
    ROS_INFO("Waiting for High Speed Encoder Phidget " \
             "%d to be attached....", serial_number);
  }
  int result;
  if ((result =
         CPhidget_waitForAttachment((CPhidgetHandle) phid,
                                    10000)))
  {
    const char *err;
    CPhidget_getErrorDescription(result, &err);
    ROS_ERROR("Problem waiting for attachment: %s",
              err);
    return false;
  }
  else return true;
}

/*!
 * \brief disconnect the encoder
 */
void disconnect(CPhidgetEncoderHandle &phid)
{
  ROS_INFO("Closing...");
  CPhidget_close((CPhidgetHandle) phid);
  CPhidget_delete((CPhidgetHandle) phid);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "phidgets_high_speed_encoder");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  int serial_number = -1;
  nh.getParam("serial", serial_number);
  std::string name = "encoder";
  nh.getParam("name", name);
  if (serial_number == -1)
  {
    nh.getParam("serial_number", serial_number);
  }
  if (inverted)
  {
    ROS_INFO("values are inverted");
  }

  std::string topic_path = "phidgets/";
  nh.getParam("topic_path", topic_path);
  nh.getParam("frame_id", frame_id);
  ROS_INFO("frame_id = %s", frame_id.c_str());
  nh.getParam("inverted", inverted);
  nh.getParam("PUBLISH_RATE", PUBLISH_RATE);
  nh.getParam("SPEED_FILTER_SAMPLES_LEN", SPEED_FILTER_SAMPLES_LEN);
  nh.getParam("SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET", SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET);

  std::string topic_name = topic_path + name;
  if (serial_number > -1)
  {
    char ser[10];
    sprintf(ser, "%d", serial_number);
    topic_name += "/";
    topic_name += ser;
  }

  if (attach(phid, serial_number))
  {
    display_properties(phid);

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

        // First time: create publishers:
        ROS_ASSERT(encoder_decimspeed_pubs.size() == encoder_pubs.size());
        if (encoder_pubs.size() != N)
        {
          encoder_pubs.resize(N);
          encoder_decimspeed_pubs.resize(N);
          for (unsigned int i = 0; i < N; i++)
          {
            std::string pub_name = topic_name;
            char ch[10];
            sprintf(ch, "/channel%u", i);
            pub_name += ch;
            ROS_INFO("Publishing state to topic: %s", pub_name.c_str());
            encoder_pubs[i] =
              n.advertise<phidgets_high_speed_encoder::EncoderParams>(
                pub_name,
                100);

            pub_name += "_decim_speed";
            ROS_INFO("Publishing decimated speed to topic: %s", pub_name.c_str());
            encoder_decimspeed_pubs[i] =
              n.advertise<phidgets_high_speed_encoder::EncoderDecimatedSpeed>(
                pub_name,
                10);
          }
        }

        for (unsigned int i = 0; i < N; i++)
        {
          TStatePerChannel &spc = encoder_states[i];

          phidgets_high_speed_encoder::EncoderParams e;
          e.header.stamp = ros::Time::now();
          e.header.frame_id = frame_id;
          e.count = (inverted ? -spc.tickPos : spc.tickPos);
          e.inst_vel = spc.instantaneousSpeed;
          encoder_pubs[i].publish(e);

          if (SPEED_FILTER_SAMPLES_LEN > 0)
          {
            if (!spc.speed_buffer_updated)
            {
              if (++spc.loops_without_update_speed_buffer >= SPEED_FILTER_IDLE_ITER_LOOPS_BEFORE_RESET)
              {
                phidgets_high_speed_encoder::EncoderDecimatedSpeed e;
                e.header.stamp = ros::Time::now();
                e.header.frame_id = frame_id;
                e.speed = .0;
                encoder_decimspeed_pubs[i].publish(e);
              }
            }
            else
            {
              spc.loops_without_update_speed_buffer = 0;

              if (spc.speeds_buffer.size() >= SPEED_FILTER_SAMPLES_LEN)
              {
                const double avrg = std::accumulate(spc.speeds_buffer.begin(), spc.speeds_buffer.end(), 0.0) / spc.speeds_buffer.size();
                spc.speeds_buffer.clear();

                phidgets_high_speed_encoder::EncoderDecimatedSpeed e;
                e.header.stamp = ros::Time::now();
                e.header.frame_id = frame_id;
                e.speed = avrg;
                encoder_decimspeed_pubs[i].publish(e);
              }
            }
          }
        }
      } // end lock guard
    } // end while ros::ok()
  } // end if attached()

  disconnect(phid);
  return 0;
}
