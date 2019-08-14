Phidgets spatial ROS driver
===========================

This is the ROS driver for Phidgets spatial.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/imu/data_raw` (`sensor_msgs/Imu`) - The raw accelerometer and gyroscope data.
* `imu/is_calibrated` (`std_msgs/Bool`) - Whether the gyroscope has been calibrated; this will be done automatically at startup time, but can also be re-done at any time by calling the `imu/calibrate` service.
* `/imu/mag` (`sensor_msgs/MagneticField`) - The raw magnetometer data.

Services
--------
* `imu/calibrate` (`std_srvs/Empty`) - Run calibration on the gyroscope.

Parameters
----------
* `serial` (int) - The serial number of the phidgets spatial to connect to.  If -1 (the default), connects to any spatial phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the spatial phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to [REP-0145](http://www.ros.org/reps/rep-0145.html) compliant `imu_link`.
* `linear_acceleration_stdev` (double) - The standard deviation to use for the linear acceleration when publishing the message.  Defaults to 300 ug.
* `angular_velocity_stdev` (double) - The standard deviation to use for the angular velocity when publishing the message.  Defaults to 0.02 deg/s.
* `magnetic_field_stdev` - The standard deviation to use for the magnetic field when publishing the message.  Defaults to 0.095 deg/s.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 250 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `cc_mag_field` (double) - Ambient magnetic field calibration value; see device's user guide for information on how to calibrate.
* `cc_offset0` (double) - Calibration offset value 0; see device's user guide for information on how to calibrate.
* `cc_offset1` (double) - Calibration offset value 1; see device's user guide for information on how to calibrate.
* `cc_offset2` (double) - Calibration offset value 2; see device's user guide for information on how to calibrate.
* `cc_gain0` (double) - Gain offset value 0; see device's user guide for information on how to calibrate.
* `cc_gain1` (double) - Gain offset value 1; see device's user guide for information on how to calibrate.
* `cc_gain2` (double) - Gain offset value 2; see device's user guide for information on how to calibrate.
* `cc_t0` (double) - T offset value 0; see device's user guide for information on how to calibrate.
* `cc_t1` (double) - T offset value 1; see device's user guide for information on how to calibrate.
* `cc_t2` (double) - T offset value 2; see device's user guide for information on how to calibrate.
* `cc_t3` (double) - T offset value 3; see device's user guide for information on how to calibrate.
* `cc_t4` (double) - T offset value 4; see device's user guide for information on how to calibrate.
* `cc_t5` (double) - T offset value 5; see device's user guide for information on how to calibrate.
