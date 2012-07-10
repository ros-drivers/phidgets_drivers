#include "phidgets_cpp_api/phidgets.h"

namespace phidgets {

Phidget::Phidget(CPhidgetHandle * handle):mHandle(handle)
{

};

~Phidget::Phidget()
{
  CPhidget_delete(*mHandle);
}

int Phidget::open(int serial_number)
{
  return CPhidget_open(*mHandle,serial_number);
}

int Phidget::close(int serial_number)
{
  return CPhidget_close(*mHandle);
}

int Phidget::waitForAttachment(int timeout)
{
  return CPhidget_waitForAttachment(*mHandle, timeout);
}

std::string Phidget::getDeviceType()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType( *mHandle, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getDeviceName()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceName( *mHandle, &deviceptr);
  return std::string(deviceptr);
};

std::string Phidget::getDeviceLabel()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType( *mHandle, &deviceptr);
  return std::string(deviceptr);
};

std::string Phidget::getLibraryVersion(){
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getLibraryVersion(&deviceptr);
  return std::string(deviceptr);
};

int Phidget::getDeviceSerialNumber()
{
  int sernum;
  CPhidget_getSerialNumber(*mHandle, &sernum);
  return sernum;
};

int Phidget::getDeviceVersion()
{
  int version;
  CPhidget_getDeviceVersion(*mHandle, &version);
  return version;
};

std::string Phidget::getErrorDescription(int errorCode)
{
  char a[1000];
  const char * errorPtr = a;
  CPhidget_getErrorDescription(errorCode, &errorPtr);
  return std::string(errorPtr);
};


int Phidget::attachHandler()
{
  printf("Attach handler ran!\n");
  return 0;
};

} //namespace phidgets
