#include "phidgets_api/phidget21.h"

#include <cstdio>

namespace phidgets {

Phidget21::Phidget21()
{
}

Phidget21::~Phidget21()
{
    // close(); // segfaults, why?
    CPhidget_delete(handle_);
}

void Phidget21::registerHandlers()
{
    CPhidget_set_OnAttach_Handler(handle_, &Phidget21::AttachHandler, this);
    CPhidget_set_OnDetach_Handler(handle_, &Phidget21::DetachHandler, this);
    CPhidget_set_OnError_Handler(handle_, &Phidget21::ErrorHandler, this);
}

void Phidget21::init(CPhidgetHandle handle)
{
    handle_ = handle;
}

int Phidget21::openAndWaitForAttachment(int serial_number, int timeout)
{
    int ret = CPhidget_open(handle_, serial_number);
    if (ret != EPHIDGET_OK)
    {
        return ret;
    }

    return CPhidget_waitForAttachment(handle_, timeout);
}

int Phidget21::close()
{
    return CPhidget_close(handle_);
}

std::string Phidget21::getDeviceType()
{
    char a[1000];
    const char *deviceptr = a;
    CPhidget_getDeviceType(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget21::getDeviceName()
{
    char a[1000];
    const char *deviceptr = a;
    CPhidget_getDeviceName(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget21::getDeviceLabel()
{
    char a[1000];
    const char *deviceptr = a;
    CPhidget_getDeviceType(handle_, &deviceptr);
    return std::string(deviceptr);
}

std::string Phidget21::getLibraryVersion()
{
    char a[1000];
    const char *deviceptr = a;
    CPhidget_getLibraryVersion(&deviceptr);
    return std::string(deviceptr);
}

int Phidget21::getDeviceSerialNumber()
{
    int sernum;
    CPhidget_getSerialNumber(handle_, &sernum);
    return sernum;
}

int Phidget21::getDeviceVersion()
{
    int version;
    CPhidget_getDeviceVersion(handle_, &version);
    return version;
}

std::string Phidget21::getErrorDescription(int errorCode)
{
    char a[1000];
    const char *errorPtr = a;
    CPhidget_getErrorDescription(errorCode, &errorPtr);
    return std::string(errorPtr);
}

void Phidget21::attachHandler()
{
    printf("Phidget attached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget21::detachHandler()
{
    printf("Phidget detached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget21::errorHandler(int error)
{
    printf("Phidget error [%d]: %s\n", error,
           getErrorDescription(error).c_str());
}

int Phidget21::AttachHandler(CPhidgetHandle /* handle */, void *userptr)
{
    ((Phidget21 *)userptr)->attachHandler();
    return 0;
}

int Phidget21::DetachHandler(CPhidgetHandle /* handle */, void *userptr)
{
    ((Phidget21 *)userptr)->detachHandler();
    return 0;
}

int Phidget21::ErrorHandler(CPhidgetHandle /* handle */, void *userptr,
                            int ErrorCode, const char * /* unknown */)
{
    ((Phidget21 *)userptr)->errorHandler(ErrorCode);
    return 0;
}

}  // namespace phidgets
