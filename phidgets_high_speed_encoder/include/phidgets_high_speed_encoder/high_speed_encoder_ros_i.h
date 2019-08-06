#ifndef PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
#define PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H

#include <mutex>
#include <vector>

#include <ros/ros.h>

#include <phidgets_api/encoder.h>

namespace phidgets {
class HighSpeedEncoderRosI final : public Encoder
{
  public:
    explicit HighSpeedEncoderRosI(ros::NodeHandle nh,
                                  ros::NodeHandle nh_private);

  private:
    void attachHandler() override;
    void detachHandler() override;
    void errorHandler(int errorCode) override;

    void positionChangeHandler(int index, int time,
                               int positionChange) override;

    void display_properties();

    void timerCallback(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher encoder_pub_;
    ros::Timer timer_;
    struct TStatePerChannel {
        int tickPos = 0;
        double instantaneousSpeed = .0;
        std::vector<double> speeds_buffer;
        bool speed_buffer_updated = false;
        int loops_without_update_speed_buffer = 0;
    };
    std::vector<TStatePerChannel> encoder_states_;
    std::mutex encoder_states_mutex_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_tick2rad_;
    std::vector<ros::Publisher> encoder_decimspeed_pubs_;
    // (Default=10) Number of samples for the sliding window average filter of
    // speeds.
    int speed_filter_samples_len_;
    // (Default=1) Number of "ITERATE" loops without any new encoder tick before
    // resetting the filtered average velocities.
    int speed_filter_idle_iter_loops_before_reset_;
    std::string frame_id_;
};
}  // namespace phidgets

#endif  // PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
