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
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>

#include "phidgets_gyroscope/gyroscope_ros_i.hpp"

namespace phidgets {

GyroscopeRosI::GyroscopeRosI(const rclcpp::NodeOptions &options)
    : rclcpp::Node("phidgets_gyroscope_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Gyroscope");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int hub_port = this->declare_parameter(
        "hub_port", 0);  // only used if the device is on a VINT hub_port

    // As specified in http://www.ros.org/reps/rep-0145.html
    frame_id_ = this->declare_parameter("frame_id", "imu_link");

    // 0.095 deg/s gyroscope white noise sigma, as per manual
    double angular_velocity_stdev = this->declare_parameter(
        "angular_velocity_stdev", 0.095 * (M_PI / 180.0));
    angular_velocity_variance_ =
        angular_velocity_stdev * angular_velocity_stdev;

    int time_resync_ms =
        this->declare_parameter("time_resynchonization_interval_ms", 5000);
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

    RCLCPP_INFO(get_logger(),
                "Connecting to Phidgets Gyroscope serial %d, hub port %d ...",
                serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(gyro_mutex_);

    try
    {
        gyroscope_ = std::make_unique<Gyroscope>(
            serial_num, hub_port, false,
            std::bind(&GyroscopeRosI::gyroscopeChangeCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Connected to serial %d",
                    gyroscope_->getSerialNumber());

        gyroscope_->setDataInterval(data_interval_ms);

        cal_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "imu/is_calibrated", rclcpp::SystemDefaultsQoS().transient_local());

        calibrate();
    } catch (const Phidget22Error &err)
    {
        RCLCPP_ERROR(get_logger(), "Gyroscope: %s", err.what());
        throw;
    }

    gyroscope_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);

    cal_srv_ = this->create_service<std_srvs::srv::Empty>(
        "imu/calibrate",
        std::bind(&GyroscopeRosI::calibrateService, this, std::placeholders::_1,
                  std::placeholders::_2));

    if (publish_rate_ > 0.0)
    {
        double pub_msec = 1000.0 / publish_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
            std::bind(&GyroscopeRosI::timerCallback, this));
    }
}

void GyroscopeRosI::calibrate()
{
    RCLCPP_INFO(get_logger(),
                "Calibrating Gyro, this takes around 2 seconds to finish. "
                "Make sure that the device is not moved during this time.");
    gyroscope_->zero();
    // The API call returns directly, so we "enforce" the recommended 2 sec
    // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40

    // FIXME: Ideally we'd use an rclcpp method that honors use_sim_time here,
    // but that doesn't actually exist.  Once
    // https://github.com/ros2/rclcpp/issues/465 is solved, we can revisit this.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(get_logger(), "Calibrating Gyro done.");

    // publish message
    auto is_calibrated_msg = std::make_unique<std_msgs::msg::Bool>();
    is_calibrated_msg->data = true;
    cal_publisher_->publish(std::move(is_calibrated_msg));
}

void GyroscopeRosI::calibrateService(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;
    (void)res;
    calibrate();
}

void GyroscopeRosI::publishLatest()
{
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();

    msg->header.frame_id = frame_id_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (i == j)
            {
                int idx = j * 3 + i;
                msg->angular_velocity_covariance[idx] =
                    angular_velocity_variance_;
            }
        }
    }

    uint64_t gyro_diff_in_ns = last_data_timestamp_ns_ - data_time_zero_ns_;
    uint64_t time_in_ns = ros_time_zero_.nanoseconds() + gyro_diff_in_ns;

    if (time_in_ns < last_ros_stamp_ns_)
    {
        RCLCPP_WARN(get_logger(),
                    "Time went backwards (%lu < %lu)! Not publishing message.",
                    time_in_ns, last_ros_stamp_ns_);
        return;
    }

    last_ros_stamp_ns_ = time_in_ns;

    msg->header.stamp = rclcpp::Time(time_in_ns);

    // set angular velocities
    msg->angular_velocity.x = last_gyro_x_;
    msg->angular_velocity.y = last_gyro_y_;
    msg->angular_velocity.z = last_gyro_z_;

    gyroscope_pub_->publish(std::move(msg));
}

void GyroscopeRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(gyro_mutex_);
    if (can_publish_)
    {
        publishLatest();
    }
}

void GyroscopeRosI::gyroscopeChangeCallback(const double angular_rate[3],
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

    std::lock_guard<std::mutex> lock(gyro_mutex_);

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
        last_gyro_x_ = angular_rate[0] * (M_PI / 180.0);
        last_gyro_y_ = angular_rate[1] * (M_PI / 180.0);
        last_gyro_z_ = angular_rate[2] * (M_PI / 180.0);

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

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::GyroscopeRosI)
