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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "phidgets_api/spatial.hpp"
#include "phidgets_spatial/spatial_ros_i.hpp"

namespace phidgets {

SpatialRosI::SpatialRosI(const rclcpp::NodeOptions &options)
    : rclcpp::Node("phidgets_spatial_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Spatial");

    bool use_orientation = this->declare_parameter(
        "use_orientation",
        false);  // default do not use the onboard orientation
    std::string spatial_algorithm =
        this->declare_parameter("spatial_algorithm", "ahrs");

    double ahrsAngularVelocityThreshold;
    double ahrsAngularVelocityDeltaThreshold;
    double ahrsAccelerationThreshold;
    double ahrsMagTime;
    double ahrsAccelTime;
    double ahrsBiasTime;

    this->declare_parameter("ahrs_angular_velocity_threshold",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("ahrs_angular_velocity_delta_threshold",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("ahrs_acceleration_threshold",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("ahrs_mag_time",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("ahrs_accel_time",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("ahrs_bias_time",
                            rclcpp::ParameterType::PARAMETER_DOUBLE);

    bool has_ahrs_params =
        this->get_parameter("ahrs_angular_velocity_threshold",
                            ahrsAngularVelocityThreshold) &&
        this->get_parameter("ahrs_angular_velocity_delta_threshold",
                            ahrsAngularVelocityDeltaThreshold) &&
        this->get_parameter("ahrs_acceleration_threshold",
                            ahrsAccelerationThreshold) &&
        this->get_parameter("ahrs_mag_time", ahrsMagTime) &&
        this->get_parameter("ahrs_accel_time", ahrsAccelTime) &&
        this->get_parameter("ahrs_bias_time", ahrsBiasTime);

    double algorithm_magnetometer_gain;
    bool set_algorithm_magnetometer_gain = true;
    if (!this->get_parameter("algorithm_magnetometer_gain",
                             algorithm_magnetometer_gain))
    {
        algorithm_magnetometer_gain = 0.0;
        set_algorithm_magnetometer_gain =
            false;  // if parameter not set, do not call api (because this
                    // function is not available from MOT0110 onwards)
    }

    bool heating_enabled;
    bool set_heating_enabled = true;
    if (!this->get_parameter("heating_enabled", heating_enabled))
    {
        set_heating_enabled =
            false;  // if parameter not set, do not call api (because this
                    // function is just available from MOT0109 onwards)
    }

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    // As specified in http://www.ros.org/reps/rep-0145.html
    frame_id_ = this->declare_parameter("frame_id", "imu_link");

    double linear_acceleration_stdev = this->declare_parameter(
        "linear_acceleration_stdev",
        280.0 * 1e-6 *
            G);  // 280 ug accelerometer white noise sigma, as per manual
    linear_acceleration_variance_ =
        linear_acceleration_stdev * linear_acceleration_stdev;

    // 0.095 deg/s gyroscope white noise sigma, as per manual
    double angular_velocity_stdev = this->declare_parameter(
        "angular_velocity_stdev", 0.095 * (M_PI / 180.0));
    angular_velocity_variance_ =
        angular_velocity_stdev * angular_velocity_stdev;

    // 1.1 milligauss magnetometer white noise sigma, as per manual
    double magnetic_field_stdev =
        this->declare_parameter("magnetic_field_stdev", 1.1 * 1e-3 * 1e-4);
    magnetic_field_variance_ = magnetic_field_stdev * magnetic_field_stdev;

    int time_resync_ms =
        this->declare_parameter("time_resynchronization_interval_ms", 5000);
    time_resync_interval_ns_ =
        static_cast<int64_t>(time_resync_ms) * 1000 * 1000;

    int data_interval_ms = this->declare_parameter("data_interval_ms", 8);
    data_interval_ns_ = data_interval_ms * 1000 * 1000;

    int cb_delta_epsilon_ms =
        this->declare_parameter("callback_delta_epsilon_ms", 1);
    cb_delta_epsilon_ns_ = cb_delta_epsilon_ms * 1000 * 1000;
    if (cb_delta_epsilon_ms >= data_interval_ms)
    {
        throw std::runtime_error(
            "Callback epsilon is larger than the data interval; this can never "
            "work");
    }

    publish_rate_ = this->declare_parameter("publish_rate", 0.0);
    if (publish_rate_ > 1000.0)
    {
        throw std::runtime_error("Publish rate must be <= 1000");
    }

    this->declare_parameter("server_name",
                            rclcpp::ParameterType::PARAMETER_STRING);
    this->declare_parameter("server_ip",
                            rclcpp::ParameterType::PARAMETER_STRING);
    if (this->get_parameter("server_name", server_name_) &&
        this->get_parameter("server_ip", server_ip_))
    {
        PhidgetNet_addServer(server_name_.c_str(), server_ip_.c_str(), 5661, "",
                             0);

        RCLCPP_INFO(get_logger(), "Using phidget server %s at IP %s",
                    server_name_.c_str(), server_ip_.c_str());
    }

    // compass correction params (see
    // http://www.phidgets.com/docs/1044_User_Guide)
    this->declare_parameter("cc_mag_field", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_offset0", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_offset1", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_offset2", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_gain0", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_gain1", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_gain2", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t0", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t1", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t2", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t3", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t4", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("cc_t5", rclcpp::PARAMETER_DOUBLE);

    bool has_compass_params = false;
    double cc_mag_field = 0.0;
    double cc_offset0 = 0.0;
    double cc_offset1 = 0.0;
    double cc_offset2 = 0.0;
    double cc_gain0 = 0.0;
    double cc_gain1 = 0.0;
    double cc_gain2 = 0.0;
    double cc_T0 = 0.0;
    double cc_T1 = 0.0;
    double cc_T2 = 0.0;
    double cc_T3 = 0.0;
    double cc_T4 = 0.0;
    double cc_T5 = 0.0;

    try
    {
        cc_mag_field = this->get_parameter("cc_mag_field").get_value<double>();
        cc_offset0 = this->get_parameter("cc_offset0").get_value<double>();
        cc_offset1 = this->get_parameter("cc_offset1").get_value<double>();
        cc_offset2 = this->get_parameter("cc_offset2").get_value<double>();
        cc_gain0 = this->get_parameter("cc_gain0").get_value<double>();
        cc_gain1 = this->get_parameter("cc_gain1").get_value<double>();
        cc_gain2 = this->get_parameter("cc_gain2").get_value<double>();
        cc_T0 = this->get_parameter("cc_t0").get_value<double>();
        cc_T1 = this->get_parameter("cc_t1").get_value<double>();
        cc_T2 = this->get_parameter("cc_t2").get_value<double>();
        cc_T3 = this->get_parameter("cc_t3").get_value<double>();
        cc_T4 = this->get_parameter("cc_t4").get_value<double>();
        cc_T5 = this->get_parameter("cc_t5").get_value<double>();
        has_compass_params = true;
    } catch (const rclcpp::exceptions::ParameterUninitializedException &)
    {
    }

    RCLCPP_INFO(get_logger(),
                "Connecting to Phidgets Spatial serial %d, hub port %d ...",
                serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(spatial_mutex_);

    last_quat_w_ = 0.0;
    last_quat_x_ = 0.0;
    last_quat_y_ = 0.0;
    last_quat_z_ = 0.0;

    try
    {
        std::function<void(const double[4], double)> algorithm_data_handler =
            nullptr;
        if (use_orientation)
        {
            algorithm_data_handler =
                std::bind(&SpatialRosI::spatialAlgorithmDataCallback, this,
                          std::placeholders::_1, std::placeholders::_2);
        }

        spatial_ = std::make_unique<Spatial>(
            serial_num, hub_port, false,
            std::bind(&SpatialRosI::spatialDataCallback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4),
            algorithm_data_handler,
            std::bind(&SpatialRosI::attachCallback, this),
            std::bind(&SpatialRosI::detachCallback, this));

        RCLCPP_INFO(get_logger(), "Connected to serial %d",
                    spatial_->getSerialNumber());

        spatial_->setDataInterval(data_interval_ms);

        cal_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "imu/is_calibrated", rclcpp::SystemDefaultsQoS().transient_local());

        calibrate();

        if (use_orientation)
        {
            spatial_->setSpatialAlgorithm(spatial_algorithm);

            if (has_ahrs_params)
            {
                spatial_->setAHRSParameters(ahrsAngularVelocityThreshold,
                                            ahrsAngularVelocityDeltaThreshold,
                                            ahrsAccelerationThreshold,
                                            ahrsMagTime, ahrsAccelTime,
                                            ahrsBiasTime);
            }

            if (set_algorithm_magnetometer_gain)
                spatial_->setAlgorithmMagnetometerGain(
                    algorithm_magnetometer_gain);
        }

        if (has_compass_params)
        {
            spatial_->setCompassCorrectionParameters(
                cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
                cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
        } else
        {
            RCLCPP_INFO(get_logger(), "No compass correction params found.");
        }

        if (set_heating_enabled)
        {
            spatial_->setHeatingEnabled(heating_enabled);
        }
    } catch (const Phidget22Error &err)
    {
        RCLCPP_ERROR(get_logger(), "Spatial: %s", err.what());
        throw;
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);

    cal_srv_ = this->create_service<std_srvs::srv::Empty>(
        "imu/calibrate",
        std::bind(&SpatialRosI::calibrateService, this, std::placeholders::_1,
                  std::placeholders::_2));

    magnetic_field_pub_ =
        this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&SpatialRosI::timerCallback, this));
    }
}

void SpatialRosI::calibrate()
{
    RCLCPP_INFO(get_logger(),
                "Calibrating IMU, this takes around 2 seconds to finish. "
                "Make sure that the device is not moved during this time.");
    spatial_->zero();
    // The API call returns directly, so we "enforce" the recommended 2 sec
    // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40

    // FIXME: Ideally we'd use an rclcpp method that honors use_sim_time here,
    // but that doesn't actually exist.  Once
    // https://github.com/ros2/rclcpp/issues/465 is solved, we can revisit this.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(get_logger(), "Calibrating IMU done.");

    // publish message
    auto is_calibrated_msg = std::make_unique<std_msgs::msg::Bool>();
    is_calibrated_msg->data = true;
    cal_publisher_->publish(std::move(is_calibrated_msg));
}

void SpatialRosI::calibrateService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    calibrate();
}

void SpatialRosI::publishLatest()
{
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();

    auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

    // build covariance matrices
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (i == j)
            {
                int idx = j * 3 + i;
                msg->linear_acceleration_covariance[idx] =
                    linear_acceleration_variance_;
                msg->angular_velocity_covariance[idx] =
                    angular_velocity_variance_;
                mag_msg->magnetic_field_covariance[idx] =
                    magnetic_field_variance_;
            }
        }
    }

    // Fill out and send IMU message
    msg->header.frame_id = frame_id_;

    uint64_t imu_diff_in_ns = last_data_timestamp_ns_ - data_time_zero_ns_;
    uint64_t time_in_ns = ros_time_zero_.nanoseconds() + imu_diff_in_ns;

    if (time_in_ns < last_ros_stamp_ns_)
    {
        RCLCPP_WARN(get_logger(),
                    "Time went backwards (%lu < %lu)! Not publishing message.",
                    time_in_ns, last_ros_stamp_ns_);
        return;
    }

    last_ros_stamp_ns_ = time_in_ns;

    rclcpp::Time ros_time = rclcpp::Time(time_in_ns);

    msg->header.stamp = ros_time;

    // set linear acceleration
    msg->linear_acceleration.x = last_accel_x_;
    msg->linear_acceleration.y = last_accel_y_;
    msg->linear_acceleration.z = last_accel_z_;

    // set angular velocities
    msg->angular_velocity.x = last_gyro_x_;
    msg->angular_velocity.y = last_gyro_y_;
    msg->angular_velocity.z = last_gyro_z_;

    // set spatial algorithm orientation estimation
    msg->orientation.w = last_quat_w_;
    msg->orientation.x = last_quat_x_;
    msg->orientation.y = last_quat_y_;
    msg->orientation.z = last_quat_z_;

    imu_pub_->publish(std::move(msg));

    // Fill out and publish magnetic message
    mag_msg->header.frame_id = frame_id_;

    mag_msg->header.stamp = ros_time;

    mag_msg->magnetic_field.x = last_mag_x_;
    mag_msg->magnetic_field.y = last_mag_y_;
    mag_msg->magnetic_field.z = last_mag_z_;

    magnetic_field_pub_->publish(std::move(mag_msg));
}

void SpatialRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(spatial_mutex_);
    if (can_publish_)
    {
        publishLatest();
    }
}

void SpatialRosI::spatialDataCallback(const double acceleration[3],
                                      const double angular_rate[3],
                                      const double magnetic_field[3],
                                      double timestamp)
{
    // When publishing the message on the ROS network, we want to publish the
    // time that the data was acquired in seconds since the Unix epoch.  The
    // data we have to work with is the time that the callback happened (on the
    // local processor, in Unix epoch seconds), and the timestamp that the
    // IMU gives us on the callback (from the processor on the IMU, in
    // milliseconds since some arbitrary starting point).
    //
    // At a first approximation, we can apply the timestamp from the device to
    // Unix epoch seconds by taking a common starting point on the IMU and the
    // local processor, then applying the delta between this IMU timestamp and
    // the "zero" IMU timestamp to the local processor starting point.
    //
    // There are several complications with the simple scheme above.  The first
    // is finding a proper "zero" point where the IMU timestamp and the local
    // timestamp line up.  Due to potential delays in servicing this process,
    // along with USB delays, the delta timestamp from the IMU and the time when
    // this callback gets called can be wildly different.  Since we want the
    // initial zero for both the IMU and the local time to be in the same time
    // "window", we throw away data at the beginning until we see that the delta
    // callback and delta timestamp are within reasonable bounds of each other.
    //
    // The second complication is that the time on the IMU drifts away from the
    // time on the local processor.  Taking the "zero" time once at the
    // beginning isn't sufficient, and we have to periodically re-synchronize
    // the times given the constraints above.  Because we still have the
    // arbitrary delays present as described above, it can take us several
    // callbacks to successfully synchronize.  We continue publishing data using
    // the old "zero" time until successfully resynchronize, at which point we
    // switch to the new zero point.

    std::lock_guard<std::mutex> lock(spatial_mutex_);

    rclcpp::Time now = this->now();

    // At the beginning of time, need to initialize last_cb_time for later use;
    // last_cb_time is used to figure out the time between callbacks
    if (last_cb_time_.nanoseconds() == 0)
    {
        last_cb_time_ = now;
        // We need to initialize the ros_time_zero since rclcpp::Duration
        // below won't let us subtract an essentially uninitialized
        // rclcpp::Time from another one.  However, we'll still do an initial
        // synchronization since the default value of synchronize_timestamp
        // is true.
        ros_time_zero_ = now;
        return;
    }

    rclcpp::Duration time_since_last_cb = now - last_cb_time_;
    uint64_t this_ts_ns = static_cast<uint64_t>(timestamp * 1000.0 * 1000.0);

    if (synchronize_timestamps_)
    {
        // The only time it's safe to sync time between IMU and ROS Node is when
        // the data that came in is within the data interval that data is
        // expected. It's possible for data to come late because of USB issues
        // or swapping, etc and we don't want to sync with data that was
        // actually published before this time interval, so we wait until we get
        // data that is within the data interval +/- an epsilon since we will
        // have taken some time to process and/or a short delay (maybe USB
        // comms) may have happened
        if (time_since_last_cb.nanoseconds() >=
                (data_interval_ns_ - cb_delta_epsilon_ns_) &&
            time_since_last_cb.nanoseconds() <=
                (data_interval_ns_ + cb_delta_epsilon_ns_))
        {
            ros_time_zero_ = now;
            data_time_zero_ns_ = this_ts_ns;
            synchronize_timestamps_ = false;
            can_publish_ = true;
        } else
        {
            RCLCPP_DEBUG(
                get_logger(),
                "Data not within acceptable window for synchronization: "
                "expected between %ld and %ld, saw %ld",
                data_interval_ns_ - cb_delta_epsilon_ns_,
                data_interval_ns_ + cb_delta_epsilon_ns_,
                time_since_last_cb.nanoseconds());
        }
    }

    if (can_publish_)  // Cannot publish data until IMU/ROS timestamps have been
                       // synchronized at least once
    {
        // Save off the values
        last_accel_x_ = -acceleration[0] * G;
        last_accel_y_ = -acceleration[1] * G;
        last_accel_z_ = -acceleration[2] * G;

        last_gyro_x_ = angular_rate[0] * (M_PI / 180.0);
        last_gyro_y_ = angular_rate[1] * (M_PI / 180.0);
        last_gyro_z_ = angular_rate[2] * (M_PI / 180.0);

        if (magnetic_field[0] != PUNK_DBL)
        {
            // device reports data in Gauss, multiply by 1e-4 to convert to
            // Tesla
            last_mag_x_ = magnetic_field[0] * 1e-4;
            last_mag_y_ = magnetic_field[1] * 1e-4;
            last_mag_z_ = magnetic_field[2] * 1e-4;
        } else
        {
            // data is PUNK_DBL ("unknown double"), which means the magnetometer
            // did not return valid readings. When publishing at 250 Hz, this
            // will happen in every second message, because the magnetometer can
            // only sample at 125 Hz. It is still important to publish these
            // messages, because a downstream node sometimes uses a
            // TimeSynchronizer to get Imu and Magnetometer nodes.
            double nan = std::numeric_limits<double>::quiet_NaN();

            last_mag_x_ = nan;
            last_mag_y_ = nan;
            last_mag_z_ = nan;
        }
        last_data_timestamp_ns_ = this_ts_ns;

        // Publish if we aren't publishing on a timer
        if (publish_rate_ <= 0.0)
        {
            publishLatest();
        }
    }

    // Determine if we need to resynchronize - time between IMU and ROS Node can
    // drift, periodically resync to deal with this issue
    rclcpp::Duration diff = now - ros_time_zero_;
    if (time_resync_interval_ns_ > 0 &&
        diff.nanoseconds() >= time_resync_interval_ns_)
    {
        synchronize_timestamps_ = true;
    }

    last_cb_time_ = now;
}

void SpatialRosI::spatialAlgorithmDataCallback(const double quaternion[4],
                                               double timestamp)
{
    (void)timestamp;
    last_quat_w_ = quaternion[3];
    last_quat_x_ = quaternion[0];
    last_quat_y_ = quaternion[1];
    last_quat_z_ = quaternion[2];
}

void SpatialRosI::attachCallback()
{
    RCLCPP_INFO(get_logger(), "Phidget Spatial attached.");

    // Set data interval. This is in attachCallback() because it has to be
    // repeated on reattachment.
    spatial_->setDataInterval(data_interval_ns_ / 1000 / 1000);

    // Force resynchronization, because the device time is reset to 0 after
    // reattachment.
    synchronize_timestamps_ = true;
    can_publish_ = false;
    last_cb_time_ = rclcpp::Time(0);
}

void SpatialRosI::detachCallback()
{
    RCLCPP_INFO(get_logger(), "Phidget Spatial detached.");
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::SpatialRosI)
