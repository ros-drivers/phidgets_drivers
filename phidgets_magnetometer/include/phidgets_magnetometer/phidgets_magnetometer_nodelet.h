#ifndef PHIDGETS_MAGNETOMETER_PHIDGETS_MAGNETOMETER_NODELET_H
#define PHIDGETS_MAGNETOMETER_PHIDGETS_MAGNETOMETER_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_magnetometer/magnetometer_ros_i.h"

namespace phidgets {

class PhidgetsMagnetometerNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<MagnetometerRosI> magnetometer_;
};

}  // namespace phidgets

#endif  // PHIDGETS_MAGNETOMETER_PHIDGETS_MAGNETOMETER_NODELET_H
