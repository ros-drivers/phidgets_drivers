#ifndef PHIDGETS_MOTORS_PHIDGETS_MOTORS_NODELET_H
#define PHIDGETS_MOTORS_PHIDGETS_MOTORS_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_motors/motors_ros_i.h"

namespace phidgets {

class PhidgetsMotorsNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<MotorsRosI> motors_;
};

}  // namespace phidgets

#endif  // PHIDGETS_MOTORS_PHIDGETS_MOTORS_NODELET_H
