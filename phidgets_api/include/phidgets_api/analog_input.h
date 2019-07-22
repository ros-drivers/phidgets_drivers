#ifndef PHIDGETS_API_ANALOG_INPUT_H
#define PHIDGETS_API_ANALOG_INPUT_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class AnalogInput final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(AnalogInput)

    explicit AnalogInput(int32_t serial_number, int hub_port,
                         bool is_hub_port_device, int channel,
                         std::function<void(int, double)> input_handler);

    ~AnalogInput();

    double getSensorValue() const;

    void voltageChangeHandler(double sensorValue) const;

  private:
    int channel_;
    std::function<void(int, double)> input_handler_;
    PhidgetVoltageInputHandle ai_handle_;

    static void VoltageChangeHandler(PhidgetVoltageInputHandle input_handle,
                                     void *ctx, double sensorValue);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ANALOG_INPUT_H
