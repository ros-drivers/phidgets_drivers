#ifndef PHIDGETS_API_TEMPERATURE_H
#define PHIDGETS_API_TEMPERATURE_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

enum class ThermocoupleType {
    J_TYPE = 1,
    K_TYPE = 2,
    E_TYPE = 3,
    T_TYPE = 4,
};

class Temperature final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Temperature)

    explicit Temperature(int32_t serial_number, int hub_port,
                         bool is_hub_port_device,
                         std::function<void(double)> temperature_handler);

    ~Temperature();

    void setThermocoupleType(ThermocoupleType type);

    double getTemperature() const;

    void setDataInterval(uint32_t interval_ms) const;

    void temperatureChangeHandler(double temperature) const;

  private:
    std::function<void(double)> temperature_handler_;
    PhidgetTemperatureSensorHandle temperature_handle_;
    static void TemperatureChangeHandler(
        PhidgetTemperatureSensorHandle temperature_handle, void *ctx,
        double temperature);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_TEMPERATURE_H
