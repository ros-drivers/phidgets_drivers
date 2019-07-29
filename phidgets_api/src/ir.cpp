#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/ir.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

IR::IR(int32_t serial_number,
       std::function<void(const char *, uint32_t, int)> code_handler)
    : code_handler_(code_handler)
{
    // create the handle
    PhidgetReturnCode ret = PhidgetIR_create(&ir_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create IR handle", ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(ir_handle_),
                                   serial_number, 0, false, 0);

    // register ir data callback
    PhidgetIR_setOnCodeHandler(ir_handle_, CodeHandler, this);
}

IR::~IR()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(ir_handle_);
    helpers::closeAndDelete(&handle);
}

void IR::codeHandler(const char *code, uint32_t bit_count, int is_repeat) const
{
    code_handler_(code, bit_count, is_repeat);
}

void IR::CodeHandler(PhidgetIRHandle /* ir */, void *ctx, const char *code,
                     uint32_t bit_count, int is_repeat)
{
    ((IR *)ctx)->codeHandler(code, bit_count, is_repeat);
}

}  // namespace phidgets
