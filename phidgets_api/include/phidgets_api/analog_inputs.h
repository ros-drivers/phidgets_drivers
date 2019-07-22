#ifndef PHIDGETS_API_ANALOG_INPUTS_H
#define PHIDGETS_API_ANALOG_INPUTS_H

#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/analog_input.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class AnalogInputs final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(AnalogInputs)

    explicit AnalogInputs(int32_t serial_number, int hub_port,
                          bool is_hub_port_device,
                          std::function<void(int, double)> input_handler);

    ~AnalogInputs();

    uint32_t getInputCount() const noexcept;

    double getSensorValue(int index) const;

  private:
    uint32_t input_count_;
    std::vector<std::unique_ptr<AnalogInput>> ais_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ANALOG_INPUTS_H
