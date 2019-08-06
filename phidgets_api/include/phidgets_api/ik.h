#ifndef PHIDGETS_API_IK_H
#define PHIDGETS_API_IK_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class IK : public Phidget
{
  public:
    IK();

    virtual ~IK();

    int getInputCount() const;
    int getOutputCount() const;
    int getSensorCount() const;
    int getSensorRawValue(int index) const;
    bool setOutputState(int index, bool state) const;

  protected:
    virtual void sensorHandler(int index, int sensorValue);
    virtual void inputHandler(int index, int inputValue);

  private:
    CPhidgetInterfaceKitHandle ik_handle_;
    static int SensorHandler(CPhidgetInterfaceKitHandle ik, void* userptr,
                             int index, int sensorValue);
    static int InputHandler(CPhidgetInterfaceKitHandle ik, void* userptr,
                            int index, int inputValue);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_IK_H
