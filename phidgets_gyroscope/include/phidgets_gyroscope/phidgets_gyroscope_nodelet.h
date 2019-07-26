#ifndef PHIDGETS_GYROSCOPE_PHIDGETS_GYROSCOPE_NODELET_H
#define PHIDGETS_GYROSCOPE_PHIDGETS_GYROSCOPE_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_gyroscope/gyroscope_ros_i.h"

namespace phidgets {

class PhidgetsGyroscopeNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<GyroscopeRosI> gyroscope_;
};

}  // namespace phidgets

#endif  // PHIDGETS_GYROSCOPE_PHIDGETS_GYROSCOPE_NODELET_H
