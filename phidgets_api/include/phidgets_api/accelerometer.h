#ifndef PHIDGETS_API_ACCELEROMETER_H
#define PHIDGETS_API_ACCELEROMETER_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Accelerometer final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Accelerometer)

    explicit Accelerometer(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        std::function<void(const double[3], double)> data_handler);

    ~Accelerometer();

    void getAcceleration(double &x, double &y, double &z,
                         double &timestamp) const;

    void setDataInterval(uint32_t interval_ms) const;

    void dataHandler(const double acceleration[3], double timestamp) const;

  private:
    std::function<void(const double[3], double)> data_handler_;
    PhidgetAccelerometerHandle accel_handle_;

    static void DataHandler(PhidgetAccelerometerHandle input_handle, void *ctx,
                            const double acceleration[3], double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ACCELEROMETER_H
