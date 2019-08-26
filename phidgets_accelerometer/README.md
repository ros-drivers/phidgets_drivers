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
* `time_resynchronization_interval_ms` (int) - The number of milliseconds to wait between resynchronizing the time on the Phidgets spatial with the local time.  Larger values have less "jumps", but will have more timestamp drift.  Setting this to 0 disables resynchronization.  Defaults to 5000 ms.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 8 ms.
* `callback_delta_epsilon_ms` (int) - The number of milliseconds epsilon allowed between callbacks when attempting to resynchronize the time.  If this is set to 1, then a difference of `data_interval_ms` plus or minus 1 millisecond will be considered viable for resynchronization.  Higher values give the code more leeway to resynchronize, at the cost of potentially getting bad resynchronizations sometimes.  Lower values can give better results, but can also result in never resynchronizing.  Must be less than `data_interval_ms`.  Defaults to 1 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
