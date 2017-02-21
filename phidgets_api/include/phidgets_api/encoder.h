#ifndef PHIDGETS_API_ENCODER_H
#define PHIDGETS_API_ENCODER_H

#include "phidgets_api/phidget.h"

namespace phidgets {

class Encoder: public Phidget
{
  public:
    Encoder();

    /**@brief Gets the number of digital input channels supported by this board */
    int getInputCount();

    /** @brief Reads the current state of a digital input
     * @param index The index of the input to read */
    bool getInputState(int index);

    /**@brief Gets the number of encoder input channels supported by this board */
    int getEncoderCount();

    /** @brief Reads the current position of an encoder
     * @param index The index of the encoder to read */
    int getPosition(int index);

    /** @brief Sets the offset of an encoder such that current position is the specified value
     * @param index The index of the encoder to set
     * @param position The new value that should be returned by 'getPosition(index)' at the current position of the encoder*/
    void setPosition(int index, int position);

    /** @brief Gets the position of an encoder the last time an index pulse occured. An index pulse in this context refers to an input
     * from the encoder the pulses high once every revolution.
     * @param index The index of the encoder to read */
    int getIndexPosition(int index);

    /** @brief Checks if an encoder is powered on and receiving events
     * @param index The index of the encoder to check */
    bool getEnabled(int index);

    /** @brief Set the powered state of an encoder. If an encoder is not enabled, it will not be given
     * power, and events and changes in position will not be captured.
     * @param index The index of the encoder to change
     * @param enabled The new powered state of the encoder*/
    void setEnabled(int index, bool enabled);

    /* TODO Add set event handlers from phidget api
     * int CPhidgetEncoder_set_OnInputChange_Handler (CPhidgetEncoderHandle phid, int(*fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int inputState), void *userPtr)
     * int CPhidgetEncoder_set_OnPositionChange_Handler (CPhidgetEncoderHandle phid, int(*fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int time, int positionChange), void *userPtr)
     * int CPhidgetEncoder_set_OnIndex_Handler (CPhidgetEncoderHandle phid, int(*fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int indexPosition), void *userPtr)
     */

  protected:
    CPhidgetEncoderHandle encoder_handle_;
};

} //namespace phidgets

#endif // PHIDGETS_API_ENCODEr_H
