#include "phidgets_api/ik.h"

namespace phidgets {

IK::IK() : Phidget(), ik_handle_(nullptr)
{
    // create the handle
    CPhidgetInterfaceKit_create(&ik_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)ik_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register ik data callback
    CPhidgetInterfaceKit_set_OnSensorChange_Handler(ik_handle_, SensorHandler,
                                                    this);
    CPhidgetInterfaceKit_set_OnInputChange_Handler(ik_handle_, InputHandler,
                                                   this);
}

IK::~IK()
{
}

int IK::getInputCount() const
{
    int n_in;
    CPhidgetInterfaceKit_getInputCount(ik_handle_, &n_in);
    return n_in;
}

int IK::getOutputCount() const
{
    int n_out;
    CPhidgetInterfaceKit_getOutputCount(ik_handle_, &n_out);
    return n_out;
}

int IK::getSensorCount() const
{
    int n_sensors;
    CPhidgetInterfaceKit_getSensorCount(ik_handle_, &n_sensors);
    return n_sensors;
}

int IK::getSensorRawValue(int index) const
{
    int rawval;
    CPhidgetInterfaceKit_getSensorRawValue(ik_handle_, index, &rawval);
    return rawval;
}

bool IK::setOutputState(int index, bool state) const
{
    return !CPhidgetInterfaceKit_setOutputState(ik_handle_, index, state);
}

int IK::SensorHandler(CPhidgetInterfaceKitHandle /* ik */, void *userptr,
                      int index, int sensorValue)
{
    ((IK *)userptr)->sensorHandler(index, sensorValue);
    return 0;
}

void IK::sensorHandler(int /* index */, int /* sensorValue */)
{
    // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

int IK::InputHandler(CPhidgetInterfaceKitHandle /* ik */, void *userptr,
                     int index, int inputValue)
{
    ((IK *)userptr)->inputHandler(index, inputValue);
    return 0;
}

void IK::inputHandler(int /* index */, int /* inputValue */)
{
    // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

}  // namespace phidgets
