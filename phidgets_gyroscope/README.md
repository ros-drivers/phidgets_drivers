Phidgets gyroscope ROS driver
==================================

This is the ROS driver for Phidgets gyroscope.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `imu/is_calibrated` (`std_msgs/Bool`) - Whether the gyroscope has been calibrated; this will be done automatically at startup time, but can also be re-done at any time by calling the `imu/calibrate` service.
* `imu/data_raw` (`sensor_msgs/Imu`) - The raw data coming out of the gyroscope.

Services
--------
* `imu/calibrate` (`std_srvs/Empty`) - Run calibration on the gyroscope.

Parameters
----------
* `serial` (int) - The serial number of the phidgets gyroscope to connect to.  If -1 (the default), connects to any gyroscope phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the gyroscope phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to [REP-0145](http://www.ros.org/reps/rep-0145.html) compliant `imu_link`.
* `angular_velocity_stdev` (double) - The standard deviation to use for the angular velocity when publishing the message.  Defaults to 0.02 deg/s.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 8 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
