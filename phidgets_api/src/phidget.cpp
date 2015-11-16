#include "phidgets_api/phidget.h"

namespace phidgets {

Phidget::Phidget()
{
  updater.add("IMU Driver Status", this, &phidgets::Phidget::phidgetsDiagnostics);

}

Phidget::~Phidget()
{
  //close(); // segfaults, why?
  CPhidget_delete(handle_);
}

void Phidget::registerHandlers()
{
  CPhidget_set_OnAttach_Handler(handle_, &Phidget::AttachHandler, this); 
  CPhidget_set_OnDetach_Handler(handle_, &Phidget::DetachHandler, this); 
  CPhidget_set_OnError_Handler (handle_, &Phidget::ErrorHandler,  this);
}

void Phidget::init(CPhidgetHandle handle)
{
  handle_ = handle;
}

int Phidget::open(int serial_number)
{
  updater.setHardwareID("none");
  return CPhidget_open(handle_, serial_number);
}

int Phidget::close()
{
  return CPhidget_close(handle_);
}

int Phidget::waitForAttachment(int timeout)
{
  return CPhidget_waitForAttachment(handle_, timeout);
}

std::string Phidget::getDeviceType()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getDeviceName()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceName(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getDeviceLabel()
{
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getDeviceType(handle_, &deviceptr);
  return std::string(deviceptr);
}

std::string Phidget::getLibraryVersion(){
  char a[1000];
  const char * deviceptr = a;
  CPhidget_getLibraryVersion(&deviceptr);
  return std::string(deviceptr);
}

int Phidget::getDeviceSerialNumber()
{
  int sernum;
  CPhidget_getSerialNumber(handle_, &sernum);
  return sernum;
}

int Phidget::getDeviceVersion()
{
  int version;
  CPhidget_getDeviceVersion(handle_, &version);
  return version;
}

std::string Phidget::getErrorDescription(int errorCode)
{
  char a[1000];
  const char * errorPtr = a;
  CPhidget_getErrorDescription(errorCode, &errorPtr);
  return std::string(errorPtr);
}

void Phidget::attachHandler()
{
  	is_connected = true;
  	updater.force_update();
	printf("Phidget attached (serial# %d)\n", getDeviceSerialNumber());
}

void Phidget::detachHandler()
{
	printf("Phidget detached (serial# %d)\n", getDeviceSerialNumber());
        is_connected = false;
        updater.force_update();
}

void Phidget::errorHandler(int error)
{
        is_error = true;
        updater.force_update();
        is_error = false;
	printf("Phidget error [%d]: %s\n", error, getErrorDescription(error).c_str());
}

int Phidget::AttachHandler(CPhidgetHandle handle, void *userptr)
{
  ((Phidget*)userptr)->attachHandler();
  return 0;
}

int Phidget::DetachHandler(CPhidgetHandle handle, void *userptr)
{
  ((Phidget*)userptr)->detachHandler();
  return 0;
}

int Phidget::ErrorHandler(CPhidgetHandle handle, void *userptr, int ErrorCode, const char *unknown)
{
  ((Phidget*)userptr)->errorHandler(ErrorCode);
  return 0;
}

//  Added for diagnostics
void Phidget::phidgetsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (is_connected)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "The Phidget is connected.");
    stat.add("Device Serial Number", getDeviceSerialNumber());
    stat.add("Device Name", getDeviceName());
    stat.add("Device Type", getDeviceType());
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "The Phidget is not connected. Check USB.");
  }

  if (is_error && error_number != 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "The Phidget is in Error.");
    stat.addf("Error Number","%f",error_number);
    stat.add("Error message",getErrorDescription(error_number));
  }
}

} //namespace phidgets
