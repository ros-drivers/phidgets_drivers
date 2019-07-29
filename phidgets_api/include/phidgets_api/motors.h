#ifndef PHIDGETS_API_MOTORS_H
#define PHIDGETS_API_MOTORS_H

#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/motor.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class Motors final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Motors)

    explicit Motors(int32_t serial_number, int hub_port,
                    bool is_hub_port_device,
                    std::function<void(int, double)> duty_cycle_change_handler,
                    std::function<void(int, double)> back_emf_change_handler);

    ~Motors();

    uint32_t getMotorCount() const noexcept;
    double getDutyCycle(int index) const;
    void setDutyCycle(int index, double duty_cycle) const;
    double getAcceleration(int index) const;
    void setAcceleration(int index, double acceleration) const;
    double getBackEMF(int index) const;
    void setDataInterval(int index, uint32_t data_interval_ms) const;

    double getBraking(int index) const;
    void setBraking(int index, double braking) const;

  private:
    uint32_t motor_count_;
    std::vector<std::unique_ptr<Motor>> motors_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_MOTORS_H
