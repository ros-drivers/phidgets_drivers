#ifndef PHIDGETS_ANALOG_INPUTS_PHIDGETS_ANALOG_INPUTS_NODELET_H
#define PHIDGETS_ANALOG_INPUTS_PHIDGETS_ANALOG_INPUTS_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_analog_inputs/analog_inputs_ros_i.h"

namespace phidgets {

class PhidgetsAnalogInputsNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<AnalogInputsRosI> ais_;
};

}  // namespace phidgets

#endif  // PHIDGETS_ANALOG_INPUTS_PHIDGETS_ANALOG_INPUTS_NODELET_H
