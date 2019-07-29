#include <functional>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/motor.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Motor::Motor(int32_t serial_number, int hub_port, bool is_hub_port_device,
             int channel,
             std::function<void(int, double)> duty_cycle_change_handler,
             std::function<void(int, double)> back_emf_change_handler)
    : channel_(channel),
      duty_cycle_change_handler_(duty_cycle_change_handler),
      back_emf_change_handler_(back_emf_change_handler)
{
    PhidgetReturnCode ret = PhidgetDCMotor_create(&motor_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Motor handle for channel " +
                                 std::to_string(channel),
                             ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(motor_handle_), serial_number, hub_port,
        is_hub_port_device, channel);

    ret = PhidgetDCMotor_setOnVelocityUpdateHandler(
        motor_handle_, DutyCycleChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set duty cycle update handler for Motor channel " +
                std::to_string(channel),
            ret);
    }

    ret = PhidgetDCMotor_setOnBackEMFChangeHandler(motor_handle_,
                                                   BackEMFChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set back EMF update handler for Motor channel " +
                std::to_string(channel),
            ret);
    }
}

Motor::~Motor()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(motor_handle_);
    helpers::closeAndDelete(&handle);
}

double Motor::getDutyCycle() const
{
    double duty_cycle;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getVelocity(motor_handle_, &duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get duty cycle for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return duty_cycle;
}

void Motor::setDutyCycle(double duty_cycle) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setTargetVelocity(motor_handle_, duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set duty cycle for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

double Motor::getAcceleration() const
{
    double accel;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getAcceleration(motor_handle_, &accel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return accel;
}

void Motor::setAcceleration(double acceleration) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setAcceleration(motor_handle_, acceleration);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set acceleration for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

double Motor::getBackEMF() const
{
    double backemf;
    PhidgetReturnCode ret = PhidgetDCMotor_getBackEMF(motor_handle_, &backemf);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get back EMF for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return backemf;
}

void Motor::setDataInterval(uint32_t data_interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setDataInterval(motor_handle_, data_interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval for Motor channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

double Motor::getBraking() const
{
    double braking;
    PhidgetReturnCode ret =
        PhidgetDCMotor_getBrakingStrength(motor_handle_, &braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to get braking strength for Motor channel " +
                std::to_string(channel_),
            ret);
    }
    return braking;
}

void Motor::setBraking(double braking) const
{
    PhidgetReturnCode ret =
        PhidgetDCMotor_setTargetBrakingStrength(motor_handle_, braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set braking strength for Motor channel " +
                std::to_string(channel_),
            ret);
    }
}

void Motor::dutyCycleChangeHandler(double duty_cycle) const
{
    duty_cycle_change_handler_(channel_, duty_cycle);
}

void Motor::backEMFChangeHandler(double back_emf) const
{
    back_emf_change_handler_(channel_, back_emf);
}

void Motor::DutyCycleChangeHandler(PhidgetDCMotorHandle /* motor_handle */,
                                   void *ctx, double duty_cycle)
{
    ((Motor *)ctx)->dutyCycleChangeHandler(duty_cycle);
}

void Motor::BackEMFChangeHandler(PhidgetDCMotorHandle /* motor_handle */,
                                 void *ctx, double back_emf)
{
    ((Motor *)ctx)->backEMFChangeHandler(back_emf);
}

}  // namespace phidgets
