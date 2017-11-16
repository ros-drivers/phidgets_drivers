#include "phidgets_api/ik.h"

namespace phidgets {

IK::IK():
  Phidget(),
  ik_handle_(0)
{
  // create the handle
  CPhidgetInterfaceKit_create(&ik_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)ik_handle_);

  // register base class callbacks
  Phidget::registerHandlers();

  // register ik data callback
  CPhidgetInterfaceKit_set_OnSensorChange_Handler(ik_handle_, SensorHandler, this);
}


int IK::SensorHandler(CPhidgetInterfaceKitHandle ik, void *userptr, int index, int sensorValue)
{
  (void)ik;
  ((IK*)userptr)->sensorHandler(index, sensorValue);
  return 0;
}

void IK::sensorHandler(size_t index, int sensorValue)
{
  printf("index: %ld, value: %d\n", index, sensorValue);
}

int IK::InputHandler(CPhidgetInterfaceKitHandle ik, void *userptr, int index, int inputValue)
{
  (void)ik;
  ((IK*)userptr)->inputHandler(index, inputValue);
  return 0;
}

void IK::inputHandler(size_t index, int inputValue)
{
  printf("index: %ld, value: %d\n", index, inputValue);
}

} // namespace phidgets
