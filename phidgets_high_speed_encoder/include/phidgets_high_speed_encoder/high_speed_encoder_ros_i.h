#ifndef PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
#define PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "phidgets_api/encoders.h"

namespace phidgets {

struct EncoderDataToPub {
    double instantaneous_speed = .0;
    std::vector<double> speeds_buffer;
    bool speed_buffer_updated = false;
    int loops_without_update_speed_buffer = 0;
    std::string joint_name;
    double joint_tick2rad;
    ros::Publisher encoder_decimspeed_pub;
};

class HighSpeedEncoderRosI final
{
  public:
    explicit HighSpeedEncoderRosI(ros::NodeHandle nh,
                                  ros::NodeHandle nh_private);

  private:
    std::unique_ptr<Encoders> encs_;
    std::mutex encoder_mutex_;
    std::vector<EncoderDataToPub> enc_data_to_pub_;
    std::string frame_id_;
    // (Default=10) Number of samples for the sliding window average filter of
    // speeds.
    int speed_filter_samples_len_;
    // (Default=1) Number of "ITERATE" loops without any new encoder tick before
    // resetting the filtered average velocities.
    int speed_filter_idle_iter_loops_before_reset_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher encoder_pub_;
    void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    int publish_rate_;

    void publishLatest(int channel);

    void positionChangeHandler(int channel, int position_change, double time,
                               int index_triggered);
};
}  // namespace phidgets

#endif  // PHIDGETS_HIGH_SPEED_ENCODER_HIGH_SPEED_ENCODER_ROS_I_H
