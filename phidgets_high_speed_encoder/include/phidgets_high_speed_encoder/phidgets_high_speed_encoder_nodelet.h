#ifndef PHIDGETS_HIGH_SPEED_ENCODER_PHIDGETS_HIGH_SPEED_ENCODER_NODELET_H
#define PHIDGETS_HIGH_SPEED_ENCODER_PHIDGETS_HIGH_SPEED_ENCODER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "phidgets_high_speed_encoder/high_speed_encoder_ros_i.h"

namespace phidgets {

class PhidgetsHighSpeedEncoderNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    Encoder* enc_;  // FIXME: change to smart pointer
};

}  // namespace phidgets

#endif  // PHIDGETS_HIGH_SPEED_ENCODER_PHIDGETS_HIGH_SPEED_ENCODER_NODELET_H
