Phidgets accelerometer ROS driver
=================================

This is the ROS driver for Phidgets accelerometers.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/imu/data_raw` (`sensor_msgs/Imu`) - The raw accelerometer data.

Parameters
----------
* `serial` (int) - The serial number of the phidgets accelerometer to connect to.  If -1 (the default), connects to any accelerometer phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the accelerometer phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to [REP-0145](http://www.ros.org/reps/rep-0145.html) compliant `imu_link`.
* `linear_acceleration_stdev` (double) - The standard deviation to use for the linear acceleration when publishing the message.  Defaults to 300 ug.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 8 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
