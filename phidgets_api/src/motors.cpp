#include <functional>
#include <memory>

#include <libphidget22/phidget22.h>

#include "phidgets_api/motors.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Motors::Motors(int32_t serial_number, int hub_port, bool is_hub_port_device,
               std::function<void(int, double)> duty_cycle_change_handler,
               std::function<void(int, double)> back_emf_change_handler)
{
    PhidgetReturnCode ret;

    PhidgetDCMotorHandle motor_handle;

    ret = PhidgetDCMotor_create(&motor_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create Motor handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(motor_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_DCMOTOR,
                                        &motor_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get Motor device channel count", ret);
    }

    motors_.resize(motor_count_);
    for (uint32_t i = 0; i < motor_count_; ++i)
    {
        motors_[i] = std::make_unique<Motor>(
            serial_number, hub_port, is_hub_port_device, i,
            duty_cycle_change_handler, back_emf_change_handler);
    }
}

Motors::~Motors()
{
}

uint32_t Motors::getMotorCount() const noexcept
{
    return motor_count_;
}

double Motors::getDutyCycle(int index) const
{
    return motors_.at(index)->getDutyCycle();
}

void Motors::setDutyCycle(int index, double duty_cycle) const
{
    motors_.at(index)->setDutyCycle(duty_cycle);
}

double Motors::getAcceleration(int index) const
{
    return motors_.at(index)->getAcceleration();
}

void Motors::setAcceleration(int index, double acceleration) const
{
    motors_.at(index)->setAcceleration(acceleration);
}

double Motors::getBackEMF(int index) const
{
    return motors_.at(index)->getBackEMF();
}

void Motors::setDataInterval(int index, uint32_t data_interval_ms) const
{
    motors_.at(index)->setDataInterval(data_interval_ms);
}

double Motors::getBraking(int index) const
{
    return motors_.at(index)->getBraking();
}

void Motors::setBraking(int index, double braking) const
{
    motors_.at(index)->setBraking(braking);
}

}  // namespace phidgets
