#ifndef PHIDGETS_API_IK_H
#define PHIDGETS_API_IK_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class IK: public Phidget
{
  public:

    IK();

  protected:

    CPhidgetInterfaceKitHandle ik_handle_;
    virtual void sensorHandler(
      size_t index,
      int sensorValue);
    virtual void inputHandler(
      size_t index,
      int inputValue);

  private:

    static int SensorHandler(
      CPhidgetInterfaceKitHandle ik,
      void * userptr,
      int index,
      int sensorValue);
    static int InputHandler(
      CPhidgetInterfaceKitHandle ik,
      void * userptr,
      int index,
      int inputValue);
};

} //namespace phidgets

#endif // PHIDGETS_API_IK_H
