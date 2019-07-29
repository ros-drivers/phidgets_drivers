#ifndef PHIDGETS_API_SPATIAL_H
#define PHIDGETS_API_SPATIAL_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class Spatial final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Spatial)

    explicit Spatial(int32_t serial_number, int hub_port,
                     bool is_hub_port_device,
                     std::function<void(const double[3], const double[3],
                                        const double[3], double)>
                         data_handler);

    ~Spatial();

    void setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
                                        double cc_offset1, double cc_offset2,
                                        double cc_gain0, double cc_gain1,
                                        double cc_gain2, double cc_T0,
                                        double cc_T1, double cc_T2,
                                        double cc_T3, double cc_T4,
                                        double cc_T5);

    void setDataInterval(uint32_t interval_ms) const;

    void zero() const;

    void dataHandler(const double acceleration[3], const double angular_rate[3],
                     const double magnetic_field[3], double timestamp) const;

  private:
    std::function<void(const double[3], const double[3], const double[3],
                       double)>
        data_handler_;
    PhidgetSpatialHandle spatial_handle_;

    static void DataHandler(PhidgetSpatialHandle input_handle, void *ctx,
                            const double acceleration[3],
                            const double angular_rate[3],
                            const double magnetic_field[3], double timestamp);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_SPATIAL_H
