Phidgets magnetometer ROS driver
================================

This is the ROS driver for Phidgets magnetometer.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/imu/mag` (`sensor_msgs/MagneticField`) - The raw magnetometer data.

Parameters
----------
* `serial` (int) - The serial number of the phidgets magnetometer to connect to.  If -1 (the default), connects to any magnetometer phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the magnetometer phidget is connected to a VINT hub.  Defaults to 0.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to [REP-0145](http://www.ros.org/reps/rep-0145.html) compliant `imu_link`.
* `magnetic_field_stdev` - The standard deviation to use for the magnetic field when publishing the message.  Defaults to 1.1 milligauss.
* `time_resynchronization_interval_ms` (int) - The number of milliseconds to wait between resynchronizing the time on the Phidgets spatial with the local time.  Larger values have less "jumps", but will have more timestamp drift.  Setting this to 0 disables resynchronization.  Defaults to 5000 ms.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 8 ms.
* `callback_delta_epsilon_ms` (int) - The number of milliseconds epsilon allowed between callbacks when attempting to resynchronize the time.  If this is set to 1, then a difference of `data_interval_ms` plus or minus 1 millisecond will be considered viable for resynchronization.  Higher values give the code more leeway to resynchronize, at the cost of potentially getting bad resynchronizations sometimes.  Lower values can give better results, but can also result in never resynchronizing.  Must be less than `data_interval_ms`.  Defaults to 1 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `cc_mag_field` (double) - Ambient magnetic field calibration value; see device's user guide for information on how to calibrate.
* `cc_offset0` (double) - Calibration offset value 0; see device's user guide for information on how to calibrate.
* `cc_offset1` (double) - Calibration offset value 1; see device's user guide for information on how to calibrate.
* `cc_offset2` (double) - Calibration offset value 2; see device's user guide for information on how to calibrate.
* `cc_gain0` (double) - Gain value 0; see device's user guide for information on how to calibrate.
* `cc_gain1` (double) - Gain value 1; see device's user guide for information on how to calibrate.
* `cc_gain2` (double) - Gain value 2; see device's user guide for information on how to calibrate.
* `cc_t0` (double) - T value 0; see device's user guide for information on how to calibrate.
* `cc_t1` (double) - T value 1; see device's user guide for information on how to calibrate.
* `cc_t2` (double) - T value 2; see device's user guide for information on how to calibrate.
* `cc_t3` (double) - T value 3; see device's user guide for information on how to calibrate.
* `cc_t4` (double) - T value 4; see device's user guide for information on how to calibrate.
* `cc_t5` (double) - T value 5; see device's user guide for information on how to calibrate
