#include <string>

#include "phidgets_api/phidget22.h"

namespace phidgets {

Phidget22Error::Phidget22Error(const std::string &msg, PhidgetReturnCode code)
    : std::exception()
{
    const char *error_ptr;
    PhidgetReturnCode ret = Phidget_getErrorDescription(code, &error_ptr);
    if (ret == EPHIDGET_OK)
    {
        msg_ = msg + ": " + std::string(error_ptr);
    } else
    {
        msg_ = msg + ": Unknown error";
    }
}

const char *Phidget22Error::what() const noexcept
{
    return msg_.c_str();
}

namespace helpers {

void openWaitForAttachment(PhidgetHandle handle, int32_t serial_number,
                           int hub_port, bool is_hub_port_device, int channel)
{
    PhidgetReturnCode ret;

    ret = Phidget_setDeviceSerialNumber(handle, serial_number);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device serial number", ret);
    }

    ret = Phidget_setHubPort(handle, hub_port);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device hub port", ret);
    }

    ret = Phidget_setIsHubPortDevice(handle, is_hub_port_device);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device is hub port device", ret);
    }

    ret = Phidget_setChannel(handle, channel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set device channel", ret);
    }

    ret = Phidget_openWaitForAttachment(handle, PHIDGET_TIMEOUT_DEFAULT);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to open device", ret);
    }
}

void closeAndDelete(PhidgetHandle *handle) noexcept
{
    Phidget_close(*handle);
    Phidget_delete(handle);
}

}  // namespace helpers
}  // namespace phidgets
