#ifndef PHIDGETS_IK_PHIDGETS_IK_NODELET_H
#define PHIDGETS_IK_PHIDGETS_IK_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_ik/ik_ros_i.h"

namespace phidgets {

class PhidgetsIKNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<IKRosI> ik_;
};

}  // namespace phidgets

#endif  // PHIDGETS_IK_PHIDGETS_IK_NODELET_H
