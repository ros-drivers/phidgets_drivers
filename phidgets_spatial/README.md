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
* `use_orientation` (bool) - Use the phidget spatials onboard orientation estimation; Just available on MOT0109 onwards. Set to false for older versions. Defaults to false.
* `spatial_algorithm` (string) - Name of the spatial algorithm used for orientation estimation (one of "none", "ahrs", "imu"); Just used if `use_orientation` is set to true. Defaults to `ahrs`.
* `ahrs_angular_velocity_threshold` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `ahrs_angular_velocity_delta_threshold` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `ahrs_acceleration_threshold` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `ahrs_mag_time` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `ahrs_accel_time` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `ahrs_bias_time` (double) - Parameter for AHRS orientation estimation; Just used if `use_orientation` is set to true.
* `algorithm_magnetometer_gain` (double) - Gain of magnetometer in orientation estimation algorithm; Just used if `use_orientation` is set to true. Defaults to 0.005
* `heating_enabled` (bool) - Use the internal heating element; Just available on MOT0109 onwards. Do not set this parameter for older versions.
* `linear_acceleration_stdev` (double) - The standard deviation to use for the linear acceleration when publishing the message.  Defaults to 280 ug.
* `angular_velocity_stdev` (double) - The standard deviation to use for the angular velocity when publishing the message.  Defaults to 0.095 deg/s.
* `magnetic_field_stdev` (double) - The standard deviation to use for the magnetic field when publishing the message.  Defaults to 1.1 milligauss.
* `time_resynchronization_interval_ms` (int) - The number of milliseconds to wait between resynchronizing the time on the Phidgets spatial with the local time.  Larger values have less "jumps", but will have more timestamp drift.  Setting this to 0 disables resynchronization.  Defaults to 5000 ms.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 8 ms.
* `callback_delta_epsilon_ms` (int) - The number of milliseconds epsilon allowed between callbacks when attempting to resynchronize the time.  If this is set to 1, then a difference of `data_interval_ms` plus or minus 1 millisecond will be considered viable for resynchronization.  Higher values give the code more leeway to resynchronize, at the cost of potentially getting bad resynchronizations sometimes.  Lower values can give better results, but can also result in never resynchronizing.  Must be less than `data_interval_ms`.  Defaults to 1 ms.
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
