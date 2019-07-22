#ifndef PHIDGETS_API_DIGITAL_INPUTS_H
#define PHIDGETS_API_DIGITAL_INPUTS_H

#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/digital_input.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class DigitalInputs
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(DigitalInputs)

    explicit DigitalInputs(int32_t serial_number, int hub_port,
                           bool is_hub_port_device,
                           std::function<void(int, int)> input_handler);

    ~DigitalInputs();

    uint32_t getInputCount() const noexcept;

    bool getInputValue(int index) const;

  private:
    uint32_t input_count_;
    std::vector<std::unique_ptr<DigitalInput>> dis_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_DIGITAL_INPUTS_H
