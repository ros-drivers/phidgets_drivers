#ifndef PHIDGETS_API_DIGITAL_OUTPUT_H
#define PHIDGETS_API_DIGITAL_OUTPUT_H

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class DigitalOutput final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(DigitalOutput)

    explicit DigitalOutput(int32_t serial_number, int hub_port,
                           bool is_hub_port_device, int channel);

    ~DigitalOutput();

    void setOutputState(bool state) const;

  private:
    PhidgetDigitalOutputHandle do_handle_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_DIGITAL_OUTPUT_H
