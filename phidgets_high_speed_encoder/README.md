Phidgets high speed encoder ROS driver
==================================

This is the ROS driver for Phidgets high speed encoder.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/joint_states` (`sensor_msgs/JointState`) - A joint state message containing the current state of all encoders.
* `/joint_states_chXX_decim_speed` (`phidgets_msgs/EncoderDecimatedSpeed`) - One topic per encoder of the decimated (average) speed reading from the encoder.

Parameters
----------
* `serial` (int) - The serial number of the phidgets high speed encoder to connect to.  If -1 (the default), connects to any high speed encoder phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the high speed encoder phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to `encoder_link`.
* `speed_filter_samples_len` (int) - The number of data points over which to average the speed.  Defaults to 10.
* `speed_filter_idle_iter_loops_before_reset` (int) - The number of idle loops (loops with no data) before the average speed will be reset to 0.  Defaults to 1.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
