#include "phidgets_api/ir.h"

#include <cstdio>

namespace phidgets {

IR::IR() : Phidget(), ir_handle_(nullptr)
{
    // create the handle
    CPhidgetIR_create(&ir_handle_);

    // pass handle to base class
    Phidget::init((CPhidgetHandle)ir_handle_);

    // register base class callbacks
    Phidget::registerHandlers();

    // register ir data callback
    CPhidgetIR_set_OnCode_Handler(ir_handle_, CodeHandler, this);
}

IR::~IR()
{
}

int IR::CodeHandler(CPhidgetIRHandle /* ir */, void *userptr,
                    unsigned char *data, int dataLength, int bitCount,
                    int repeat)
{
    ((IR *)userptr)->codeHandler(data, dataLength, bitCount, repeat);
    return 0;
}

void IR::codeHandler(unsigned char * /* data */, int /* dataLength */,
                     int /* bitCount */, int /* repeat */)
{
    // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

}  // namespace phidgets
