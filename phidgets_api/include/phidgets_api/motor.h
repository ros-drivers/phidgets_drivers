#ifndef PHIDGETS_API_MOTOR_H
#define PHIDGETS_API_MOTOR_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Motor final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Motor)

    explicit Motor(int32_t serial_number, int hub_port, bool is_hub_port_device,
                   int channel,
                   std::function<void(int, double)> duty_cycle_change_handler,
                   std::function<void(int, double)> back_emf_change_handler);

    ~Motor();

    double getDutyCycle() const;
    void setDutyCycle(double duty_cycle) const;
    double getAcceleration() const;
    void setAcceleration(double acceleration) const;
    double getBackEMF() const;
    void setDataInterval(uint32_t data_interval_ms) const;

    double getBraking() const;
    void setBraking(double braking) const;

    void dutyCycleChangeHandler(double duty_cycle) const;

    void backEMFChangeHandler(double back_emf) const;

  private:
    int channel_;
    std::function<void(int, double)> duty_cycle_change_handler_;
    std::function<void(int, double)> back_emf_change_handler_;
    PhidgetDCMotorHandle motor_handle_;

    static void DutyCycleChangeHandler(PhidgetDCMotorHandle motor_handle,
                                       void *ctx, double duty_cycle);
    static void BackEMFChangeHandler(PhidgetDCMotorHandle motor_handle,
                                     void *ctx, double back_emf);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_MOTOR_H
