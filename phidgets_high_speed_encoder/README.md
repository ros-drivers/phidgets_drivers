Phidgets high speed encoder ROS 2 driver
========================================

This is the ROS 2 driver for Phidgets high speed encoder.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_high_speed_encoder high_speed_encoder-launch.py

Published Topics
----------------

* `/joint_states` (`sensor_msgs/JointState`) - A joint state message containing the current state of all encoders.
* `/joint_states_chXX_decim_speed` (`phidgets_msgs/EncoderDecimatedSpeed`) - One topic per encoder of the decimated (average) speed reading from the encoder.

Services
--------

* `~/zero` (`phidgets_msgs/srv::Trigger`) - Service to zero the encoder count for a specific channel. The request takes an integer `channel` and returns a `success` boolean and a `message` string.

Parameters
----------

* `serial` (int) - The serial number of the phidgets high speed encoder to connect to.  If -1 (the default), connects to any high speed encoder phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the high speed encoder phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to `encoder_link`.
* `speed_filter_samples_len` (int) - The number of data points over which to average the speed.  Defaults to 10.
* `speed_filter_idle_iter_loops_before_reset` (int) - The number of idle loops (loops with no data) before the average speed will be reset to 0.  Defaults to 1.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (event-driven on position change).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `server_name` (string) - Optional: The name of the Phidgets network server to connect to.
* `server_ip` (string) - Optional: The IP address of the Phidgets network server to connect to.
* `joint{i}_name` (string) - name of the i-th joint in the JointState message
* `joint{i}_tick2rad` (double) - Count multiplier applied to the i-th joint. For instance, with a 300 CPR encoder and a 14:1 reduction, one can use 2*pi/(14 * 300 * 4) to convert the ticks to radians, accounting for the quadrature cycles.
