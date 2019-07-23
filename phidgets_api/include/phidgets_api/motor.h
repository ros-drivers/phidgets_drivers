#ifndef PHIDGETS_API_MOTOR_H
#define PHIDGETS_API_MOTOR_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class MotorController : public Phidget
{
  public:
    MotorController();

    virtual ~MotorController();

    // Motor specific
    int getMotorCount();
    double getVelocity(int index);
    void setVelocity(int index, double velocity);
    double getAcceleration(int index);
    void setAcceleration(int index, double acceleration);
    double getAccelerationMax(int index);
    double getAccelerationMin(int index);
    double getCurrent(int index);

    // Digital inputs
    int getInputCount();
    bool getInputState(int index);

    // Encoder inputs
    int getEncoderCount();
    int getEncoderPosition(int index);
    void setEncoderPosition(int index, int position);

    // Back EMF
    int getBackEMFSensingState(int index);
    void setBackEMFSensingState(int index, int bEMFState);
    double getBackEMF(int index);

    double getSupplyVoltage();

    double getBraking(int index);
    void setBraking(int index, double braking);

    // Analog sensors
    int getSensorCount();
    int getSensorValue(int index);
    int getSensorRawValue(int index);

    int getRatiometric();
    void setRatiometric(int ratiometric);

  protected:
    virtual void velocityChangeHandler(int index, double velocity);
    virtual void currentChangeHandler(int index, double current);
    virtual void inputChangeHandler(int index, int inputState);
    virtual void encoderPositionChangeHandler(int index, int time,
                                              int positionChange);
    virtual void encoderPositionUpdateHandler(int index, int positionChange);
    virtual void backEMFUpdateHandler(int index, double voltage);
    virtual void sensorUpdateHandler(int index, int sensorValue);
    virtual void currentUpdateHandler(int index, double current);

  private:
    CPhidgetMotorControlHandle motor_handle_;

    static int VelocityChangeHandler(CPhidgetMotorControlHandle phid,
                                     void *userPtr, int index, double velocity);
    static int CurrentChangeHandler(CPhidgetMotorControlHandle phid,
                                    void *userPtr, int index, double current);
    static int InputChangeHandler(CPhidgetMotorControlHandle phid,
                                  void *userPtr, int index, int inputState);
    static int EncoderPositionChangeHandler(CPhidgetMotorControlHandle phid,
                                            void *userPtr, int index, int time,
                                            int positionChange);
    static int EncoderPositionUpdateHandler(CPhidgetMotorControlHandle phid,
                                            void *userPtr, int index,
                                            int positionChange);
    static int BackEMFUpdateHandler(CPhidgetMotorControlHandle phid,
                                    void *userPtr, int index, double voltage);
    static int SensorUpdateHandler(CPhidgetMotorControlHandle phid,
                                   void *userPtr, int index, int sensorValue);
    static int CurrentUpdateHandler(CPhidgetMotorControlHandle phid,
                                    void *userPtr, int index, double current);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_MOTOR_H
