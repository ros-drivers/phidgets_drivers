#include "phidgets_api/encoder.h"

#include <cassert>

namespace phidgets
{

Encoder::Encoder():
  Phidget(),
  encoder_handle_(0)
{
  // create the handle
  CPhidgetEncoder_create(&encoder_handle_);

  // pass handle to base class
  Phidget::init((CPhidgetHandle)encoder_handle_);

  // register base class callbacks
  Phidget::registerHandlers();

  // register encoder data callbacks
  CPhidgetEncoder_set_OnInputChange_Handler(encoder_handle_, InputChangeHandler, this);
  CPhidgetEncoder_set_OnPositionChange_Handler(encoder_handle_, PositionChangeHandler, this);
  CPhidgetEncoder_set_OnIndex_Handler(encoder_handle_, IndexHandler, this);
}

int Encoder::getInputCount()
{
  int count;
  int ret = CPhidgetEncoder_getInputCount(encoder_handle_, &count);

  assert(ret == EPHIDGET_OK);

  return count;
}

bool Encoder::getInputState(int index)
{
  int state;
  int ret = CPhidgetEncoder_getInputState(encoder_handle_, index, &state);

  assert(ret == EPHIDGET_OK);

  return state == PTRUE;
}

int Encoder::getEncoderCount()
{
  int count;
  int ret = CPhidgetEncoder_getEncoderCount(encoder_handle_, &count);

  assert(ret == EPHIDGET_OK);

  return count;
}

int Encoder::getPosition(int index)
{
  int position;
  int ret = CPhidgetEncoder_getPosition(encoder_handle_, index, &position);

  assert(ret == EPHIDGET_OK);

  return position;
}

void Encoder::setPosition(int index, int position)
{
  int ret = CPhidgetEncoder_setPosition(encoder_handle_, index, position);

  assert(ret == EPHIDGET_OK);
}

int Encoder::getIndexPosition(int index)
{
  int position;
  int ret = CPhidgetEncoder_getIndexPosition(encoder_handle_, index, &position);

  // Encoder does not support indexing or index has not occured
  if (ret == EPHIDGET_UNKNOWNVAL) return 0;

  assert(ret == EPHIDGET_OK);

  return position;
}

bool Encoder::getEnabled(int index)
{
  int enabled;
  int ret = CPhidgetEncoder_getEnabled(encoder_handle_, index, &enabled);

  assert(ret == EPHIDGET_OK);

  return enabled == PTRUE;
}

void Encoder::setEnabled(int index, bool enabled)
{
  int ret = CPhidgetEncoder_setEnabled(encoder_handle_, index, enabled ? PTRUE : PFALSE);

  assert(ret == EPHIDGET_OK);
}

int Encoder::InputChangeHandler(CPhidgetEncoderHandle /* phid */, void *userPtr, int index, int inputState)
{
  ((Encoder*)userPtr)->inputChangeHandler(index, inputState);
  return 0;
}

int Encoder::PositionChangeHandler(CPhidgetEncoderHandle /* phid */, void *userPtr, int index, int time, int positionChange)
{
  ((Encoder*)userPtr)->positionChangeHandler(index, time, positionChange);
  return 0;
}

int Encoder::IndexHandler(CPhidgetEncoderHandle /* phid */, void *userPtr, int index, int indexPosition)
{
  ((Encoder*)userPtr)->indexHandler(index, indexPosition);
  return 0;
}

void Encoder::inputChangeHandler(int /* index */, int /* inputState */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void Encoder::positionChangeHandler(int /* index */, int /* time */, int /* positionChange */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}

void Encoder::indexHandler(int /* index */, int /* indexPosition */)
{
  // This method can be overridden in a concrete subclass (e.g., ROS wrapper)
}


} //namespace phidgets

