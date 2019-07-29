#ifndef PHIDGETS_API_ENCODERS_H
#define PHIDGETS_API_ENCODERS_H

#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/encoder.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

class Encoders final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Encoders)

    explicit Encoders(
        int32_t serial_number, int hub_port, bool is_hub_port_device,
        std::function<void(int, int, double, int)> position_change_handler);

    ~Encoders();

    /**@brief Gets the number of encoder input channels supported by this board
     */
    uint32_t getEncoderCount() const;

    /** @brief Reads the current position of an encoder
     * @param index The index of the encoder to read */
    int64_t getPosition(int index) const;

    /** @brief Sets the offset of an encoder such that current position is the
     * specified value
     * @param index The index of the encoder to set
     * @param position The new value that should be returned by
     * 'getPosition(index)' at the current position of the encoder*/
    void setPosition(int index, int64_t position) const;

    /** @brief Gets the position of an encoder the last time an index pulse
     * occured. An index pulse in this context refers to an input from the
     * encoder the pulses high once every revolution.
     * @param index The index of the encoder to read */
    int64_t getIndexPosition(int index) const;

    /** @brief Checks if an encoder is powered on and receiving events
     * @param index The index of the encoder to check */
    bool getEnabled(int index) const;

    /** @brief Set the powered state of an encoder. If an encoder is not
     * enabled, it will not be given power, and events and changes in position
     * will not be captured.
     * @param index The index of the encoder to change
     * @param enabled The new powered state of the encoder*/
    void setEnabled(int index, bool enabled) const;

  private:
    uint32_t encoder_count_;
    std::vector<std::unique_ptr<Encoder>> encs_;
};

}  // namespace phidgets

#endif  // PHIDGETS_API_ENCODERS_H
