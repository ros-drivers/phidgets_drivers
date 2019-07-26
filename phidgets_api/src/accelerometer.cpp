#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/accelerometer.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Accelerometer::Accelerometer(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(const double[3], double)> data_handler)
    : data_handler_(data_handler)
{
    PhidgetReturnCode ret = PhidgetAccelerometer_create(&accel_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Accelerometer handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(accel_handle_), serial_number, hub_port,
        is_hub_port_device, 0);

    ret = PhidgetAccelerometer_setOnAccelerationChangeHandler(
        accel_handle_, DataHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for acceleration",
                             ret);
    }
}

Accelerometer::~Accelerometer()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(accel_handle_);
    helpers::closeAndDelete(&handle);
}

void Accelerometer::getAcceleration(double &x, double &y, double &z,
                                    double &timestamp) const
{
    double accel[3];
    PhidgetReturnCode ret =
        PhidgetAccelerometer_getAcceleration(accel_handle_, &accel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration", ret);
    }

    x = accel[0];
    y = accel[1];
    z = accel[2];

    double ts;
    ret = PhidgetAccelerometer_getTimestamp(accel_handle_, &ts);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration timestamp", ret);
    }

    timestamp = ts;
}

void Accelerometer::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetAccelerometer_setDataInterval(accel_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Accelerometer::dataHandler(const double acceleration[3],
                                double timestamp) const
{
    data_handler_(acceleration, timestamp);
}

void Accelerometer::DataHandler(PhidgetAccelerometerHandle /* input_handle */,
                                void *ctx, const double acceleration[3],
                                double timestamp)
{
    ((Accelerometer *)ctx)->dataHandler(acceleration, timestamp);
}

}  // namespace phidgets
