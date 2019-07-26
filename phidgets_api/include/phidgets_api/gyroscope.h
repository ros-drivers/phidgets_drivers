#ifndef PHIDGETS_API_GYROSCOPE_H
#define PHIDGETS_API_GYROSCOPE_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Gyroscope final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Gyroscope)

    explicit Gyroscope(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        std::function<void(const double[3], double)> data_handler);

    ~Gyroscope();

    void dataHandler(const double angular_rate[3], double timestamp) const;

    void getAngularRate(double &x, double &y, double &z,
                        double &timestamp) const;

    void setDataInterval(uint32_t interval_ms) const;

    void zero() const;

  private:
    std::function<void(const double[3], double)> data_handler_;
    PhidgetGyroscopeHandle gyro_handle_;

    static void DataHandler(PhidgetGyroscopeHandle input_handle, void *ctx,
                            const double angular_rate[3], double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_GYROSCOPE_H
