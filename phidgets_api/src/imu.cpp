#include "phidgets_api/imu.h"

namespace phidgets {

Imu::Imu() : Phidget(), imu_handle_(nullptr)
{
    // create the handle
    CPhidgetSpatial_create(&imu_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)imu_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register imu data callback
    CPhidgetSpatial_set_OnSpatialData_Handler(imu_handle_, SpatialDataHandler,
                                              this);
}

Imu::~Imu()
{
}

void Imu::setDataRate(int rate)
{
    CPhidgetSpatial_setDataRate(imu_handle_, rate);
}

void Imu::zero()
{
    // zero (calibrate) gyro
    CPhidgetSpatial_zeroGyro(imu_handle_);
}

int Imu::SpatialDataHandler(CPhidgetSpatialHandle /* handle */, void *userptr,
                            CPhidgetSpatial_SpatialEventDataHandle *data,
                            int count)
{
    for (int i = 0; i < count; ++i)
    {
        double ts = data[i]->timestamp.seconds +
                    (data[i]->timestamp.microseconds / 1000.0 / 1000.0);
        ((Imu *)userptr)
            ->dataHandler(data[i]->acceleration, data[i]->angularRate,
                          data[i]->magneticField, ts);
    }
    return 0;
}

void Imu::dataHandler(const double acceleration[3], const double angularRate[3],
                      const double magneticField[3], double timestamp)
{
    // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
    (void)acceleration;
    (void)angularRate;
    (void)magneticField;
    (void)timestamp;
}

int Imu::setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
                                        double cc_offset1, double cc_offset2,
                                        double cc_gain0, double cc_gain1,
                                        double cc_gain2, double cc_T0,
                                        double cc_T1, double cc_T2,
                                        double cc_T3, double cc_T4,
                                        double cc_T5)
{
    return CPhidgetSpatial_setCompassCorrectionParameters(
        imu_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
        cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
}

}  // namespace phidgets
