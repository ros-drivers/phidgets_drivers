#include <functional>
#include <memory>
#include <vector>

#include "phidgets_api/encoder.h"
#include "phidgets_api/encoders.h"
#include "phidgets_api/phidget22.h"

namespace phidgets {

Encoders::Encoders(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(int, int, double, int)> position_change_handler)
{
    PhidgetReturnCode ret;

    PhidgetEncoderHandle enc_handle;

    ret = PhidgetEncoder_create(&enc_handle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to create Encoder handle for determining channel "
            "count",
            ret);
    }

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(enc_handle);

    helpers::openWaitForAttachment(handle, serial_number, hub_port,
                                   is_hub_port_device, 0);

    ret = Phidget_getDeviceChannelCount(handle, PHIDCHCLASS_ENCODER,
                                        &encoder_count_);

    helpers::closeAndDelete(&handle);

    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get Encoder device channel count", ret);
    }

    encs_.resize(encoder_count_);
    for (uint32_t i = 0; i < encoder_count_; ++i)
    {
        encs_[i] = std::make_unique<Encoder>(serial_number, hub_port,
                                             is_hub_port_device, i,
                                             position_change_handler);
    }
}

Encoders::~Encoders()
{
}

uint32_t Encoders::getEncoderCount() const
{
    return encoder_count_;
}

int64_t Encoders::getPosition(int index) const
{
    return encs_.at(index)->getPosition();
}

void Encoders::setPosition(int index, int64_t position) const
{
    return encs_.at(index)->setPosition(position);
}

int64_t Encoders::getIndexPosition(int index) const
{
    return encs_.at(index)->getIndexPosition();
}

bool Encoders::getEnabled(int index) const
{
    return encs_.at(index)->getEnabled();
}

void Encoders::setEnabled(int index, bool enabled) const
{
    return encs_.at(index)->setEnabled(enabled);
}

}  // namespace phidgets
