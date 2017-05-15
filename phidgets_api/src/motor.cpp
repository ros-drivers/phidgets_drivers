#include "phidgets_api/motor.h"

#include <cassert>

// Get and return the value name of type type (I varient is indexed)
#define GAR(type, name)      type x; int ret = CPhidgetMotorControl_get ## name (motor_handle_, &x); assert(ret == EPHIDGET_OK); return x;
#define GAR_I(type, name, i) type x; int ret = CPhidgetMotorControl_get ## name (motor_handle_, i, &x); assert(ret == EPHIDGET_OK); return x;

// Set the value name to x (I varient is indexed)
#define SET(name, x)      int ret = CPhidgetMotorControl_set ## name (motor_handle_, x); assert(ret == EPHIDGET_OK);
#define SET_I(name, x, i) int ret = CPhidgetMotorControl_set ## name (motor_handle_, i, x); assert(ret == EPHIDGET_OK);

namespace phidgets {

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
}

int MotorController::getMotorCount() {
    GAR(int, MotorCount);
}

double MotorController::getVelocity(int index) {
    GAR_I(double, Velocity, index);
}

void MotorController::setVelocity(int index, double velocity) {
    SET_I(Velocity, velocity, index);
}

double MotorController::getAcceleration(int index) {
    GAR_I(double, Acceleration, index);
}

void MotorController::setAcceleration(int index, double acceleration) {
    SET_I(Acceleration, acceleration, index);
}

double MotorController::getAccelerationMax(int index) {
    GAR_I(double, AccelerationMax, index);
}

double MotorController::getAccelerationMin(int index) {
    GAR_I(double, AccelerationMin, index);
}

double MotorController::getCurrent(int index) {
    GAR_I(double, Current, index);
}

int MotorController::getInputCount() {
    GAR(int, InputCount);
}

bool MotorController::getInputState(int index) {
    int state;
    int ret = CPhidgetMotorControl_getInputState(motor_handle_, index, &state);

    assert (ret == EPHIDGET_OK);

    return state == PTRUE;
}

int MotorController::getEncoderCount() {
    GAR(int, EncoderCount);
}

int MotorController::getEncoderPosition(int index) {
    GAR_I(int, EncoderPosition, index);
}

void MotorController::setEncoderPosition(int index, int position) {
    SET_I(EncoderPosition, position, index);
}

int MotorController::getBackEMFSensingState(int index) {
    GAR_I(int, BackEMFSensingState, index);
}

void MotorController::setBackEMFSensingState(int index, int bEMFState) {
    SET_I(BackEMFSensingState, bEMFState, index);
}

double MotorController::getBackEMF(int index) {
    GAR_I(double, BackEMF, index);
}

double MotorController::getSupplyVoltage() {
    GAR(double, SupplyVoltage);
}

double MotorController::getBraking(int index) {
    GAR_I(double, Braking, index);
}

void MotorController::setBraking(int index, double braking) {
    SET_I(Braking, braking, index);
}

int MotorController::getSensorCount() {
    GAR(int, SensorCount);
}

int MotorController::getSensorValue(int index) {
    GAR_I(int, SensorValue, index);
}

int MotorController::getSensorRawValue(int index) {
    GAR_I(int, SensorRawValue, index);
}

int MotorController::getRatiometric() {
    GAR(int, Ratiometric);
}

void MotorController::setRatiometric(int ratiometric) {
    SET(Ratiometric, ratiometric);
}


} //namespace phidgets

