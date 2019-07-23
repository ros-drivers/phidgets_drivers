#ifndef PHIDGETS_API_IMU_H
#define PHIDGETS_API_IMU_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class Imu : public Phidget
{
  public:
    Imu();

    virtual ~Imu();

  protected:
    void zero();
    void setDataRate(int rate);

    int setCompassCorrectionParameters(double cc_mag_field, double cc_offset0,
                                       double cc_offset1, double cc_offset2,
                                       double cc_gain0, double cc_gain1,
                                       double cc_gain2, double cc_T0,
                                       double cc_T1, double cc_T2, double cc_T3,
                                       double cc_T4, double cc_T5);

    virtual void dataHandler(const double acceleration[3],
                             const double angularRate[3],
                             const double magneticField[3], double timestamp);

  private:
    CPhidgetSpatialHandle imu_handle_;

    static int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr,
                                  CPhidgetSpatial_SpatialEventDataHandle *data,
                                  int count);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_IMU_H
