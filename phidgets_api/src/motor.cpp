#include "phidgets_api/motor.h"

#include <cassert>

// Get and return the value name of type type (I varient is indexed)
#define GAR(type, name)      type x; int ret = CPhidgetMotorControl_get ## name (motor_handle_, &x); assert(ret == EPHIDGET_OK); return x;
#define GAR_I(type, name, i) type x; int ret = CPhidgetMotorControl_get ## name (motor_handle_, i, &x); assert(ret == EPHIDGET_OK); return x;

// Set the value name to x (I varient is indexed)
#define SET(name, x)      int ret = CPhidgetMotorControl_set ## name (motor_handle_, x); assert(ret == EPHIDGET_OK);
#define SET_I(name, x, i) int ret = CPhidgetMotorControl_set ## name (motor_handle_, i, x); assert(ret == EPHIDGET_OK);

namespace phidgets
{

MotorController::MotorController():
  Phidget(),
  motor_handle_(0)
{
  // create the handle
  CPhidgetMotorControl_create(&motor_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)motor_handle_);

  // register base class callbacks
  Phidget::registerHandlers();

  // register motor data callbacks
  CPhidgetMotorControl_set_OnVelocityChange_Handler(motor_handle_, VelocityChangeHandler, this);
  CPhidgetMotorControl_set_OnCurrentChange_Handler(motor_handle_, CurrentChangeHandler, this);
  CPhidgetMotorControl_set_OnInputChange_Handler(motor_handle_, InputChangeHandler, this);
  CPhidgetMotorControl_set_OnEncoderPositionChange_Handler(motor_handle_, EncoderPositionChangeHandler, this);
  CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler(motor_handle_, EncoderPositionUpdateHandler, this);
  CPhidgetMotorControl_set_OnBackEMFUpdate_Handler(motor_handle_, BackEMFUpdateHandler, this);
  CPhidgetMotorControl_set_OnSensorUpdate_Handler(motor_handle_, SensorUpdateHandler, this);
  CPhidgetMotorControl_set_OnCurrentUpdate_Handler(motor_handle_, CurrentUpdateHandler, this);
}

int MotorController::getMotorCount()
{
  GAR(int, MotorCount);
}

double MotorController::getVelocity(int index)
{
  GAR_I(double, Velocity, index);
}

void MotorController::setVelocity(int index, double velocity)
{
  SET_I(Velocity, velocity, index);
}

double MotorController::getAcceleration(int index)
{
  GAR_I(double, Acceleration, index);
}

void MotorController::setAcceleration(int index, double acceleration)
{
  SET_I(Acceleration, acceleration, index);
}

double MotorController::getAccelerationMax(int index)
{
  GAR_I(double, AccelerationMax, index);
}

double MotorController::getAccelerationMin(int index)
{
  GAR_I(double, AccelerationMin, index);
}

double MotorController::getCurrent(int index)
{
  GAR_I(double, Current, index);
}

int MotorController::getInputCount()
{
  GAR(int, InputCount);
}

bool MotorController::getInputState(int index)
{
  int state;
  int ret = CPhidgetMotorControl_getInputState(motor_handle_, index, &state);

  assert(ret == EPHIDGET_OK);

  return state == PTRUE;
}

int MotorController::getEncoderCount()
{
  GAR(int, EncoderCount);
}

int MotorController::getEncoderPosition(int index)
{
  GAR_I(int, EncoderPosition, index);
}

void MotorController::setEncoderPosition(int index, int position)
{
  SET_I(EncoderPosition, position, index);
}

int MotorController::getBackEMFSensingState(int index)
{
  GAR_I(int, BackEMFSensingState, index);
}

void MotorController::setBackEMFSensingState(int index, int bEMFState)
{
  SET_I(BackEMFSensingState, bEMFState, index);
}

double MotorController::getBackEMF(int index)
{
  GAR_I(double, BackEMF, index);
}

double MotorController::getSupplyVoltage()
{
  GAR(double, SupplyVoltage);
}

double MotorController::getBraking(int index)
{
  GAR_I(double, Braking, index);
}

void MotorController::setBraking(int index, double braking)
{
  SET_I(Braking, braking, index);
}

int MotorController::getSensorCount()
{
  GAR(int, SensorCount);
}

int MotorController::getSensorValue(int index)
{
  GAR_I(int, SensorValue, index);
}

int MotorController::getSensorRawValue(int index)
{
  GAR_I(int, SensorRawValue, index);
}

int MotorController::getRatiometric()
{
  GAR(int, Ratiometric);
}

void MotorController::setRatiometric(int ratiometric)
{
  SET(Ratiometric, ratiometric);
}

int MotorController::VelocityChangeHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, double velocity)
{
  ((MotorController*)userPtr)->velocityChangeHandler(index, velocity);
  return 0;
}

int MotorController::CurrentChangeHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, double current)
{
  ((MotorController*)userPtr)->currentChangeHandler(index, current);
  return 0;
}

int MotorController::InputChangeHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, int inputState)
{
  ((MotorController*)userPtr)->inputChangeHandler(index, inputState);
  return 0;
}

int MotorController::EncoderPositionChangeHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, int time, int positionChange)
{
  ((MotorController*)userPtr)->encoderPositionChangeHandler(index, time, positionChange);
  return 0;
}

int MotorController::EncoderPositionUpdateHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, int positionChange)
{
  ((MotorController*)userPtr)->encoderPositionUpdateHandler(index, positionChange);
  return 0;
}

int MotorController::BackEMFUpdateHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, double voltage)
{
  ((MotorController*)userPtr)->backEMFUpdateHandler(index, voltage);
  return 0;
}

int MotorController::SensorUpdateHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, int sensorValue)
{
  ((MotorController*)userPtr)->sensorUpdateHandler(index, sensorValue);
  return 0;
}

int MotorController::CurrentUpdateHandler(CPhidgetMotorControlHandle /* phid */, void *userPtr, int index, double current)
{
  ((MotorController*)userPtr)->currentUpdateHandler(index, current);
  return 0;
}

void MotorController::velocityChangeHandler(int /* index */, double /* velocity */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::currentChangeHandler(int /* index */, double /* current */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::inputChangeHandler(int /* index */, int /* inputState */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::encoderPositionChangeHandler(int /* index */, int /* time */, int /* positionChange */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::encoderPositionUpdateHandler(int /* index */, int /* positionChange */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::backEMFUpdateHandler(int /* index */, double /* voltage */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::sensorUpdateHandler(int /* index */, int /* sensorValue */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void MotorController::currentUpdateHandler(int /* index */, double /* current */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

} //namespace phidgets
