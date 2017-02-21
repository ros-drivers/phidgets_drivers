#ifndef PHIDGETS_API_MOTOR_H
#define PHIDGETS_API_MOTOR_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class MotorController: public Phidget
{
  public:
    MotorController();

    // Motor specific
    int    getMotorCount();
    double getVelocity(int index);
    void   setVelocity(int index, double velocity);
    double getAcceleration(int index);
    void   setAcceleration(int index, double acceleration);
    double getAccelerationMax(int index);
    double getAccelerationMin(int index);
    double getCurrent(int index);

    // Digital inputs
    int  getInputCount();
    bool getInputState(int index);

    // Encoder inputs
    int  getEncoderCount();
    int  getEncoderPosition(int index);
    void setEncoderPosition(int index, int position);

    // Back EMF
    int    getBackEMFSensingState(int index);
    void   setBackEMFSensingState(int index, int bEMFState);
    double getBackEMF(int index);

    double getSupplyVoltage();

    double getBraking(int index);
    void   setBraking(int index, double braking);

    // Analog sensors
    int getSensorCount();
    int getSensorValue(int index);
    int getSensorRawValue(int index);

    int  getRatiometric();
    void setRatiometric(int ratiometric);

    /* TODO Add event handlers from phidget api
     * int CPhidgetMotorControl_set_OnVelocityChange_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double velocity), void *userPtr)
     * int CPhidgetMotorControl_set_OnCurrentChange_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double current), void *userPtr)
     * int CPhidgetMotorControl_set_OnInputChange_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int inputState), void *userPtr)
     * int CPhidgetMotorControl_set_OnEncoderPositionChange_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int time, int positionChange), void *userPtr)
     * int CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int positionChange), void *userPtr)
     * int CPhidgetMotorControl_set_OnBackEMFUpdate_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double voltage), void *userPtr)
     * int CPhidgetMotorControl_set_OnSensorUpdate_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int sensorValue), void *userPtr)
     * int CPhidgetMotorControl_set_OnCurrentUpdate_Handler (CPhidgetMotorControlHandle phid, int(*fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double current), void *userPtr)
     */

  protected:
    CPhidgetMotorControlHandle motor_handle_;
};

} //namespace phidgets

#endif // PHIDGETS_API_MOTOR_H
