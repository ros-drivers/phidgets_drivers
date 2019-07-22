#ifndef PHIDGETS_DIGITAL_INPUTS_PHIDGETS_DIGITAL_INPUTS_NODELET_H
#define PHIDGETS_DIGITAL_INPUTS_PHIDGETS_DIGITAL_INPUTS_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_digital_inputs/digital_inputs_ros_i.h"

namespace phidgets {

class PhidgetsDigitalInputsNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<DigitalInputsRosI> dis_;
};

}  // namespace phidgets

#endif  // PHIDGETS_DIGITAL_INPUTS_PHIDGETS_DIGITAL_INPUTS_NODELET_H
