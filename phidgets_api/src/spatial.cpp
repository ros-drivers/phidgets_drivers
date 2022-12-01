/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <functional>
#include <utility>

#include <libphidget22/phidget22.h>

#include "phidgets_api/phidget22.h"
#include "phidgets_api/spatial.h"

namespace phidgets {

Spatial::Spatial(
    int32_t serial_number, int hub_port, bool is_hub_port_device,
    std::function<void(const double[3], const double[3], const double[3],
                       double)>
        data_handler,
    std::function<void(const double[4], double)> algorithm_data_handler,
    std::function<void()> attach_handler, std::function<void()> detach_handler)
    : data_handler_(std::move(data_handler)),
      algorithm_data_handler_(std::move(algorithm_data_handler)),
      attach_handler_(std::move(attach_handler)),
      detach_handler_(std::move(detach_handler))
{
    PhidgetReturnCode ret = PhidgetSpatial_create(&spatial_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Spatial handle", ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(spatial_handle_), serial_number,
        hub_port, is_hub_port_device, 0);

    ret = PhidgetSpatial_setOnSpatialDataHandler(spatial_handle_, DataHandler,
                                                 this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set change handler for Spatial", ret);
    }

    if (algorithm_data_handler_ != nullptr)
    {
        ret = PhidgetSpatial_setOnAlgorithmDataHandler(
            spatial_handle_, AlgorithmDataHandler, this);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to set algorithm handler for Spatial",
                                 ret);
        }
    }

    if (attach_handler_ != nullptr)
    {
        ret = Phidget_setOnAttachHandler(
            reinterpret_cast<PhidgetHandle>(spatial_handle_), AttachHandler,
            this);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to set attach handler for Spatial",
                                 ret);
        }
    }

    if (detach_handler_ != nullptr)
    {
        ret = Phidget_setOnDetachHandler(
            reinterpret_cast<PhidgetHandle>(spatial_handle_), DetachHandler,
            this);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error("Failed to set detach handler for Spatial",
                                 ret);
        }
    }
}

Spatial::~Spatial()
{
    auto handle = reinterpret_cast<PhidgetHandle>(spatial_handle_);
    helpers::closeAndDelete(&handle);
}

void Spatial::zero() const
{
    PhidgetReturnCode ret = PhidgetSpatial_zeroGyro(spatial_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to calibrate Gyroscope", ret);
    }
}

void Spatial::setCompassCorrectionParameters(
    double cc_mag_field, double cc_offset0, double cc_offset1,
    double cc_offset2, double cc_gain0, double cc_gain1, double cc_gain2,
    double cc_T0, double cc_T1, double cc_T2, double cc_T3, double cc_T4,
    double cc_T5)
{
    PhidgetReturnCode ret = PhidgetSpatial_setMagnetometerCorrectionParameters(
        spatial_handle_, cc_mag_field, cc_offset0, cc_offset1, cc_offset2,
        cc_gain0, cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set magnetometer correction parameters",
                             ret);
    }
}

void Spatial::setSpatialAlgorithm(const std::string algorithm_name)
{
    Phidget_SpatialAlgorithm algorithm;

    if (algorithm_name.compare("none") == 0)
    {
        algorithm = SPATIAL_ALGORITHM_NONE;
    } else if (algorithm_name.compare("ahrs") == 0)
    {
        algorithm = SPATIAL_ALGORITHM_AHRS;
    } else if (algorithm_name.compare("imu") == 0)
    {
        algorithm = SPATIAL_ALGORITHM_IMU;
    } else
    {
        throw std::invalid_argument("Unknown spatial algorithm name");
    }

    PhidgetReturnCode ret =
        PhidgetSpatial_setAlgorithm(spatial_handle_, algorithm);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set spatial algorithm", ret);
    }
}

void Spatial::setAHRSParameters(double angularVelocityThreshold,
                                double angularVelocityDeltaThreshold,
                                double accelerationThreshold, double magTime,
                                double accelTime, double biasTime)
{
    PhidgetReturnCode ret = PhidgetSpatial_setAHRSParameters(
        spatial_handle_, angularVelocityThreshold,
        angularVelocityDeltaThreshold, accelerationThreshold, magTime,
        accelTime, biasTime);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set AHRS parameters", ret);
    }
}

void Spatial::setAlgorithmMagnetometerGain(double magnetometer_gain)
{
    PhidgetReturnCode ret = PhidgetSpatial_setAlgorithmMagnetometerGain(
        spatial_handle_, magnetometer_gain);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set algorithm magnetometer gain", ret);
    }
}

void Spatial::setHeatingEnabled(bool heating_enabled)
{
    PhidgetReturnCode ret =
        PhidgetSpatial_setHeatingEnabled(spatial_handle_, heating_enabled);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set heating flag", ret);
    }
}

void Spatial::setDataInterval(uint32_t interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetSpatial_setDataInterval(spatial_handle_, interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set data interval", ret);
    }
}

void Spatial::dataHandler(const double acceleration[3],
                          const double angular_rate[3],
                          const double magnetic_field[3],
                          double timestamp) const
{
    data_handler_(acceleration, angular_rate, magnetic_field, timestamp);
}

void Spatial::algorithmDataHandler(const double quaternion[4],
                                   double timestamp) const
{
    algorithm_data_handler_(quaternion, timestamp);
}

void Spatial::attachHandler()
{
    attach_handler_();
}

void Spatial::detachHandler()
{
    detach_handler_();
}

void Spatial::DataHandler(PhidgetSpatialHandle /* input_handle */, void *ctx,
                          const double acceleration[3],
                          const double angular_rate[3],
                          const double magnetic_field[3], double timestamp)
{
    ((Spatial *)ctx)
        ->dataHandler(acceleration, angular_rate, magnetic_field, timestamp);
}

void Spatial::AlgorithmDataHandler(PhidgetSpatialHandle /* input_handle */,
                                   void *ctx, const double quaternion[4],
                                   double timestamp)
{
    ((Spatial *)ctx)->algorithmDataHandler(quaternion, timestamp);
}

void Spatial::AttachHandler(PhidgetHandle /* input_handle */, void *ctx)
{
    ((Spatial *)ctx)->attachHandler();
}

void Spatial::DetachHandler(PhidgetHandle /* input_handle */, void *ctx)
{
    ((Spatial *)ctx)->detachHandler();
}

}  // namespace phidgets
