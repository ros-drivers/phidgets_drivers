#include <functional>

#include <libphidget22/phidget22.h>

#include "phidgets_api/encoder.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Encoder::Encoder(
    int32_t serial_number, int hub_port, bool is_hub_port_device, int channel,
    std::function<void(int, int, double, int)> position_change_handler)
    : channel_(channel), position_change_handler_(position_change_handler)
{
    // create the handle
    PhidgetReturnCode ret = PhidgetEncoder_create(&encoder_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Encoder handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(encoder_handle_), serial_number,
        hub_port, is_hub_port_device, channel);

    ret = PhidgetEncoder_setOnPositionChangeHandler(
        encoder_handle_, PositionChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set change handler for Encoder channel " +
                std::to_string(channel),
            ret);
    }
}

Encoder::~Encoder()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(encoder_handle_);
    helpers::closeAndDelete(&handle);
}

int64_t Encoder::getPosition() const
{
    int64_t position;
    PhidgetReturnCode ret =
        PhidgetEncoder_getPosition(encoder_handle_, &position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get position for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }

    return position;
}

void Encoder::setPosition(int64_t position) const
{
    PhidgetReturnCode ret =
        PhidgetEncoder_setPosition(encoder_handle_, position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set position for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

int64_t Encoder::getIndexPosition() const
{
    int64_t position;
    PhidgetReturnCode ret =
        PhidgetEncoder_getIndexPosition(encoder_handle_, &position);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to get index position for Encoder channel " +
                std::to_string(channel_),
            ret);
    }

    return position;
}

bool Encoder::getEnabled() const
{
    int enabled;
    PhidgetReturnCode ret =
        PhidgetEncoder_getEnabled(encoder_handle_, &enabled);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get enabled for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }

    return enabled == PTRUE;
}

void Encoder::setEnabled(bool enabled) const
{
    PhidgetReturnCode ret =
        PhidgetEncoder_setEnabled(encoder_handle_, enabled ? PTRUE : PFALSE);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set enabled for Encoder channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

void Encoder::positionChangeHandler(int position_change, double time,
                                    int index_triggered)
{
    position_change_handler_(channel_, position_change, time, index_triggered);
}

void Encoder::PositionChangeHandler(PhidgetEncoderHandle /* phid */, void *ctx,
                                    int position_change, double time,
                                    int index_triggered)
{
    ((Encoder *)ctx)
        ->positionChangeHandler(position_change, time, index_triggered);
}

}  // namespace phidgets
