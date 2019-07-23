#ifndef PHIDGETS_IK_PHIDGETS_IK_NODELET_H
#define PHIDGETS_IK_PHIDGETS_IK_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "phidgets_ik/ik_ros_i.h"

namespace phidgets {

class PhidgetsIKNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    IK* ik_;  // FIXME: change to smart pointer
};

}  // namespace phidgets

#endif  // PHIDGETS_IK_PHIDGETS_IK_NODELET_H
