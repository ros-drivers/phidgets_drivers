#ifndef PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H
#define PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "phidgets_imu/phidgets_imu.h"

namespace phidgets {

class PhidgetsImuNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    PhidgetsImu * imu_;  // FIXME: change to smart pointer
};

} // namespace phidgets

#endif // PHIDGETS_IMU_PHIDGETS_IMU_NODELET_H
