#ifndef PHIDGETS_IMU_IMU_ROS_I_H
#define PHIDGETS_IMU_IMU_ROS_I_H

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <phidgets_api/imu.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <memory>
#include <mutex>

namespace phidgets {

const float G = 9.80665;
const float MAX_TIMEDIFF_SECONDS = 0.1;

class ImuRosI final : public Imu
{
    typedef sensor_msgs::Imu ImuMsg;
    typedef sensor_msgs::MagneticField MagMsg;

  public:
    explicit ImuRosI(ros::NodeHandle nh, ros::NodeHandle nh_private);

    bool calibrateService(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher imu_publisher_;
    ros::Publisher mag_publisher_;
    ros::Publisher cal_publisher_;
    ros::ServiceServer cal_srv_;

    /**@brief updater object of class Update. Used to add diagnostic tasks, set
     * ID etc. refer package API. Added for diagnostics */
    diagnostic_updater::Updater diag_updater_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic>
        imu_publisher_diag_ptr_;

    // diagnostics
    bool is_connected_;
    int error_number_;
    double target_publish_freq_;

    ros::Time last_published_time_;
    int serial_number_;

    ImuMsg imu_msg_;
    MagMsg mag_msg_;

    ros::Time time_zero_;

    // params

    std::string frame_id_;
    int period_;  // rate in ms

    double angular_velocity_stdev_;
    double linear_acceleration_stdev_;
    double magnetic_field_stdev_;
    bool use_imu_time_;

    // compass correction params (see
    // http://www.phidgets.com/docs/1044_User_Guide)
    double cc_mag_field_;
    double cc_offset0_;
    double cc_offset1_;
    double cc_offset2_;
    double cc_gain0_;
    double cc_gain1_;
    double cc_gain2_;
    double cc_T0_;
    double cc_T1_;
    double cc_T2_;
    double cc_T3_;
    double cc_T4_;
    double cc_T5_;

    void calibrate();
    void initDevice();
    void dataHandler(const double acceleration[3], const double angularRate[3],
                     const double magneticField[3], double timestamp) override;
    void attachHandler() override;
    void detachHandler() override;
    void errorHandler(int error) override;

    /**@brief Main diagnostic method that takes care of collecting diagnostic
     * data.
     * @param stat The stat param is what is the diagnostic tasks are added two.
     * Internally published by the diagnostic_updater package. Added for
     * diagnostics */
    void phidgetsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

}  // namespace phidgets

#endif  // PHIDGETS_IMU_IMU_ROS_I_H
