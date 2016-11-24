/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets high speed encoder
 *  Copyright (c) 2010, Bob Mottram
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
#include <stdlib.h>
#include <sstream>
#include <libphidgets/phidget21.h>
#include <std_msgs/String.h>
#include "phidgets_high_speed_encoder/EncoderParams.h"

static CPhidgetEncoderHandle phid;
static ros::Publisher encoder_pub;
static bool initialised = false;
static std::string frame_id;
static bool inverted = false;

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
  int serial_number;
  const char *name;

  CPhidget_getDeviceName(phid, &name);
  CPhidget_getSerialNumber(phid, &serial_number);
  ROS_INFO("%s Serial number %d attached!",
           name, serial_number);

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
                          int Time, int)
{
  static uint32_t sequence_number = 0U;
  int Position;
  CPhidgetEncoder_getPosition(ENC, Index, &Position);

  phidgets_high_speed_encoder::EncoderParams e;
  e.header.stamp = ros::Time::now();
  e.header.frame_id = frame_id;
  e.index = Index;
  e.count = (inverted ? -Position : Position);
  if (initialised) encoder_pub.publish(e);
  ROS_DEBUG("Encoder %d Count %d", Index, Position);
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

  if (attach(phid, serial_number))
  {
    display_properties(phid);

    const int buffer_length = 100;
    std::string topic_name = topic_path + name;
    if (serial_number > -1)
    {
      char ser[10];
      sprintf(ser, "%d", serial_number);
      topic_name += "/";
      topic_name += ser;
    }
    encoder_pub =
        n.advertise<phidgets_high_speed_encoder::EncoderParams>(topic_name,
                                                                 buffer_length);

    initialised = true;

    ros::spin();

    disconnect(phid);
  }
  return 0;
}
