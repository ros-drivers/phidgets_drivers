#ifndef PHIDGETS_DIGITAL_OUTPUTS_PHIDGETS_DIGITAL_OUTPUTS_NODELET_H
#define PHIDGETS_DIGITAL_OUTPUTS_PHIDGETS_DIGITAL_OUTPUTS_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_digital_outputs/digital_outputs_ros_i.h"

namespace phidgets {

class PhidgetsDigitalOutputsNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<DigitalOutputsRosI> dos_;
};

}  // namespace phidgets

#endif  // PHIDGETS_DIGITAL_OUTPUTS_PHIDGETS_DIGITAL_OUTPUTS_NODELET_H
