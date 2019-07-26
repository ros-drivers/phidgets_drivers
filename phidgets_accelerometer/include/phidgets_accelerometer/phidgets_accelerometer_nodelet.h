#ifndef PHIDGETS_ACCELEROMETER_PHIDGETS_ACCELEROMETER_NODELET_H
#define PHIDGETS_ACCELEROMETER_PHIDGETS_ACCELEROMETER_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_accelerometer/accelerometer_ros_i.h"

namespace phidgets {

class PhidgetsAccelerometerNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<AccelerometerRosI> accelerometer_;
};

}  // namespace phidgets

#endif  // PHIDGETS_ACCELEROMETER_PHIDGETS_ACCELEROMETER_NODELET_H
