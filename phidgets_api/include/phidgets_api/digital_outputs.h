#ifndef PHIDGETS_API_DIGITAL_OUTPUTS_H
#define PHIDGETS_API_DIGITAL_OUTPUTS_H

#include <memory>
#include <vector>

#include "phidgets_api/digital_output.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class DigitalOutputs final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(DigitalOutputs)

    explicit DigitalOutputs(int32_t serial_number, int hub_port,
                            bool is_hub_port_device);

    ~DigitalOutputs();

    uint32_t getOutputCount() const noexcept;

    void setOutputState(int index, bool state) const;

  private:
    uint32_t output_count_;
    std::vector<std::unique_ptr<DigitalOutput>> dos_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_DIGITAL_OUTPUTS_H
