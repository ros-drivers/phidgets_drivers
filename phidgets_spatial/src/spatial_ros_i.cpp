#include <functional>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include "phidgets_api/spatial.h"
#include "phidgets_spatial/spatial_ros_i.h"

namespace phidgets {

SpatialRosI::SpatialRosI(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private)
{
    ROS_INFO("Starting Phidgets SPATIAL");

    ROS_INFO("Opening spatial");
    int serial_num;
    if (!nh_private_.getParam("serial", serial_num))
    {
        serial_num = -1;  // default open any device
    }
    int hub_port;
    if (!nh_private.getParam("hub_port", hub_port))
    {
        hub_port = 0;  // only used if the device is on a VINT hub_port
    }
    if (!nh_private_.getParam("frame_id", frame_id_))
    {
        // As specified in http://www.ros.org/reps/rep-0145.html
        frame_id_ = "imu_link";
    }
    if (!nh_private_.getParam("linear_acceleration_stdev",
                              linear_acceleration_stdev_))
    {
        linear_acceleration_stdev_ = 300.0 * 1e-6 * G;  // 300 ug as per manual
    }
    if (!nh_private_.getParam("angular_velocity_stdev",
                              angular_velocity_stdev_))
    {
        angular_velocity_stdev_ =
            0.02 * (M_PI / 180.0);  // 0.02 deg/s resolution, as per manual
    }
    if (!nh_private_.getParam("magnetic_field_stdev", magnetic_field_stdev_))
    {
        magnetic_field_stdev_ =
            0.095 * (M_PI / 180.0);  // 0.095°/s as per manual
    }
    int data_interval_ms;
    if (!nh_private.getParam("data_interval_ms", data_interval_ms))
    {
        data_interval_ms = 250;
    }
    if (!nh_private.getParam("publish_rate", publish_rate_))
    {
        publish_rate_ = 5;
    }

    // compass correction params (see
    // http://www.phidgets.com/docs/1044_User_Guide)
    double cc_mag_field;
    double cc_offset0;
    double cc_offset1;
    double cc_offset2;
    double cc_gain0;
    double cc_gain1;
    double cc_gain2;
    double cc_T0;
    double cc_T1;
    double cc_T2;
    double cc_T3;
    double cc_T4;
    double cc_T5;

    bool has_compass_params =
        nh_private_.getParam("cc_mag_field", cc_mag_field) &&
        nh_private_.getParam("cc_offset0", cc_offset0) &&
        nh_private_.getParam("cc_offset1", cc_offset1) &&
        nh_private_.getParam("cc_offset2", cc_offset2) &&
        nh_private_.getParam("cc_gain0", cc_gain0) &&
        nh_private_.getParam("cc_gain1", cc_gain1) &&
        nh_private_.getParam("cc_gain2", cc_gain2) &&
        nh_private_.getParam("cc_t0", cc_T0) &&
        nh_private_.getParam("cc_t1", cc_T1) &&
        nh_private_.getParam("cc_t2", cc_T2) &&
        nh_private_.getParam("cc_t3", cc_T3) &&
        nh_private_.getParam("cc_t4", cc_T4) &&
        nh_private_.getParam("cc_t5", cc_T5);

    ROS_INFO(
        "Waiting for Phidgets Spatial serial %d, hub port %d to be "
        "attached...",
        serial_num, hub_port);

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(spatial_mutex_);

    spatial_ = std::make_unique<Spatial>(
        serial_num, hub_port, false,
        std::bind(&SpatialRosI::spatialDataCallback, this,
                  std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4));

    ROS_INFO("Connected");

    spatial_->setDataInterval(data_interval_ms);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);

    cal_publisher_ = nh_.advertise<std_msgs::Bool>("spatial/is_calibrated", 5);

    calibrate();

    cal_srv_ = nh_.advertiseService("spatial/calibrate",
                                    &SpatialRosI::calibrateService, this);

    if (has_compass_params)
    {
        spatial_->setCompassCorrectionParameters(
            cc_mag_field, cc_offset0, cc_offset1, cc_offset2, cc_gain0,
            cc_gain1, cc_gain2, cc_T0, cc_T1, cc_T2, cc_T3, cc_T4, cc_T5);
    } else
    {
        ROS_INFO("No compass correction params found.");
    }

    magnetic_field_pub_ =
        nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);

    got_first_data_ = false;

    if (publish_rate_ > 0)
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &SpatialRosI::timerCallback, this);
    } else
    {
        // We'd like to publish the latest data, but the underlying libphidget22
        // class doesn't have the ability to grab the latest data, so we just
        // stay silent until the first event.
    }
}

bool SpatialRosI::calibrateService(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
{
    (void)req;
    (void)res;
    calibrate();
    return true;
}

void SpatialRosI::calibrate()
{
    ROS_INFO("Calibrating Gyro...");
    spatial_->zero();
    ROS_INFO("Calibrating Gyro done.");

    // publish message
    std_msgs::Bool is_calibrated_msg;
    is_calibrated_msg.data = true;
    cal_publisher_.publish(is_calibrated_msg);
}

void SpatialRosI::publishLatest()
{
    std::shared_ptr<sensor_msgs::Imu> msg =
        std::make_shared<sensor_msgs::Imu>();

    msg->header.frame_id = frame_id_;

    // build covariance matrix
    double lin_acc_var =
        linear_acceleration_stdev_ * linear_acceleration_stdev_;
    double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                msg->linear_acceleration_covariance[idx] = lin_acc_var;
                msg->angular_velocity_covariance[idx] = ang_vel_var;
            } else
            {
                msg->linear_acceleration_covariance[idx] = 0.0;
                msg->angular_velocity_covariance[idx] = 0.0;
            }
        }
    }

    double imu_diff_in_secs = (last_data_timestamp_ - data_time_zero_) / 1000.0;
    double time_in_secs = ros_time_zero_.toSec() + imu_diff_in_secs;

    msg->header.stamp = ros::Time(time_in_secs);

    // set linear acceleration
    msg->linear_acceleration.x = -last_accel_x_ * G;
    msg->linear_acceleration.y = -last_accel_y_ * G;
    msg->linear_acceleration.z = -last_accel_z_ * G;

    // set angular velocities
    msg->angular_velocity.x = last_gyro_x_ * (M_PI / 180.0);
    msg->angular_velocity.y = last_gyro_y_ * (M_PI / 180.0);
    msg->angular_velocity.z = last_gyro_z_ * (M_PI / 180.0);

    imu_pub_.publish(*msg);

    std::shared_ptr<sensor_msgs::MagneticField> mag_msg =
        std::make_shared<sensor_msgs::MagneticField>();

    mag_msg->header.frame_id = frame_id_;

    // build covariance matrix
    double mag_field_var = magnetic_field_stdev_ * magnetic_field_stdev_;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            int idx = j * 3 + i;

            if (i == j)
            {
                mag_msg->magnetic_field_covariance[idx] = mag_field_var;
            } else
            {
                mag_msg->magnetic_field_covariance[idx] = 0.0;
            }
        }
    }

    mag_msg->header.stamp = ros::Time(time_in_secs);

    // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
    mag_msg->magnetic_field.x = last_mag_x_ * 1e-4;
    mag_msg->magnetic_field.y = last_mag_y_ * 1e-4;
    mag_msg->magnetic_field.z = last_mag_z_ * 1e-4;

    magnetic_field_pub_.publish(*mag_msg);
}

void SpatialRosI::spatialDataCallback(const double acceleration[3],
                                      const double angular_rate[3],
                                      const double magnetic_field[3],
                                      double timestamp)
{
    std::lock_guard<std::mutex> lock(spatial_mutex_);
    last_accel_x_ = acceleration[0];
    last_accel_y_ = acceleration[1];
    last_accel_z_ = acceleration[2];

    last_gyro_x_ = angular_rate[0];
    last_gyro_y_ = angular_rate[1];
    last_gyro_z_ = angular_rate[2];

    last_mag_x_ = magnetic_field[0];
    last_mag_y_ = magnetic_field[1];
    last_mag_z_ = magnetic_field[2];

    last_data_timestamp_ = timestamp;

    if (!got_first_data_)
    {
        got_first_data_ = true;
        data_time_zero_ = timestamp;
        ros_time_zero_ = ros::Time::now();
    }

    if (publish_rate_ <= 0)
    {
        publishLatest();
    }
}

void SpatialRosI::timerCallback(const ros::TimerEvent & /* event */)
{
    std::lock_guard<std::mutex> lock(spatial_mutex_);
    if (got_first_data_)
    {
        publishLatest();
    }
}

}  // namespace phidgets
