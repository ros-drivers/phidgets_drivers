#ifndef PHIDGETS_API_IR_H
#define PHIDGETS_API_IR_H

#include "phidgets_api/phidget21.h"

namespace phidgets {

class IR : public Phidget21
{
  public:
    IR();

    virtual ~IR();

  protected:
    virtual void codeHandler(unsigned char *data, int dataLength, int bitCount,
                             int repeat);

  private:
    CPhidgetIRHandle ir_handle_;

    static int CodeHandler(CPhidgetIRHandle ir, void *userPtr,
                           unsigned char *data, int dataLength, int bitCount,
                           int repeat);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_IR_H
