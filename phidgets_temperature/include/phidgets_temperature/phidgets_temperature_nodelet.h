#ifndef PHIDGETS_TEMPERATURE_PHIDGETS_TEMPERATURE_NODELET_H
#define PHIDGETS_TEMPERATURE_PHIDGETS_TEMPERATURE_NODELET_H

#include <memory>

#include <nodelet/nodelet.h>

#include "phidgets_temperature/temperature_ros_i.h"

namespace phidgets {

class PhidgetsTemperatureNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    std::unique_ptr<TemperatureRosI> temperature_;
};

}  // namespace phidgets

#endif  // PHIDGETS_TEMPERATURE_PHIDGETS_TEMPERATURE_NODELET_H
