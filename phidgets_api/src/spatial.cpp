#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"
#include "phidgets_api/spatial.h"

namespace phidgets {

Spatial::Spatial(int32_t serial_number, int hub_port, bool is_hub_port_device,
                 std::function<void(const double[3], const double[3],
                                    const double[3], double)>
                     data_handler)
    : data_handler_(data_handler)
{
    PhidgetReturnCode ret = PhidgetSpatial_create(&spatial_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Spatial handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(spatial_handle_), serial_number,
        hub_port, is_hub_port_device, 0);

    ret = PhidgetSpatial_setOnSpatialDataHandler(spatial_handle_, DataHandler,
                                                 this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for Spatial", ret);
    }
}

Spatial::~Spatial()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(spatial_handle_);
    helpers::closeAndDelete(&handle);
}

void Spatial::zero() const
{
    PhidgetReturnCode ret = PhidgetSpatial_zeroGyro(spatial_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to calibrate Gyroscope", ret);
    }
}

void Spatial::setCompassCorrectionParameters(
    double cc_mag_field, double cc_offset0, double cc_offset1,
    double cc_offset2, double cc_gain0, double cc_gain1, double cc_gain2,
    double cc_T0, double cc_T1, double cc_T2, double cc_T3, double cc_T4,
    double cc_T5)
{
    PhidgetReturnCode ret = PhidgetSpatial_setMagnetometerCorrectionParameters(
        spatial_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2,
        cc_gain0, cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set magnetometer correction parameters",
                             ret);
    }
}

void Spatial::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetSpatial_setDataInterval(spatial_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Spatial::dataHandler(const double acceleration[3],
                          const double angular_rate[3],
                          const double magnetic_field[3],
                          double timestamp) const
{
    data_handler_(acceleration, angular_rate, magnetic_field, timestamp);
}

void Spatial::DataHandler(PhidgetSpatialHandle /* input_handle */, void *ctx,
                          const double acceleration[3],
                          const double angular_rate[3],
                          const double magnetic_field[3], double timestamp)
{
    ((Spatial *)ctx)
        ->dataHandler(acceleration, angular_rate, magnetic_field, timestamp);
}

}  // namespace phidgets
