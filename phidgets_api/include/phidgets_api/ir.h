#ifndef PHIDGETS_API_IR_H
#define PHIDGETS_API_IR_H

#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"

namespace phidgets {

class IR final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(IR)

    explicit IR(int32_t serial_number,
                std::function<void(const char *, uint32_t, int)> code_handler);

    ~IR();

    void codeHandler(const char *code, uint32_t bit_count, int is_repeat) const;

  private:
    std::function<void(const char *, uint32_t, int)> code_handler_;
    PhidgetIRHandle ir_handle_;

    static void CodeHandler(PhidgetIRHandle ir, void *ctx, const char *code,
                            uint32_t bit_count, int is_repeat);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_IR_H
