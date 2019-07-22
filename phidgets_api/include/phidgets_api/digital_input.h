#ifndef PHIDGETS_API_DIGITAL_INPUT_H
#define PHIDGETS_API_DIGITAL_INPUT_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class DigitalInput
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(DigitalInput)

    explicit DigitalInput(int32_t serial_number, int hub_port,
                          bool is_hub_port_device, int channel,
                          std::function<void(int, int)> input_handler);

    ~DigitalInput();

    bool getInputValue() const;

    void stateChangeHandler(int state) const;

  private:
    int channel_;
    std::function<void(int, int)> input_handler_;
    PhidgetDigitalInputHandle di_handle_;

    static void StateChangeHandler(PhidgetDigitalInputHandle input_handle,
                                   void *ctx, int state);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_DIGITAL_INPUT_H
