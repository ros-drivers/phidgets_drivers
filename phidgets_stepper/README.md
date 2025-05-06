Phidgets stepper ROS 2 driver
===========================

This is the ROS 2 driver for Phidgets stepper.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_stepper stepper-launch.py

Published Topics
----------------

* `~/config` (`phidgets_msgs/StepperConfig`) - Get the min/max values of the configurable parameters
* `~/state` (`phidgets_msgs/StepperState`) - Get the run time state (engaged, moving, target)
* `~/joint` (`sensor_msgs/JointState`) - Publish the motor state as a standard joint message

Subscribed Topics
-----------------

* `~/command` (`phidgets_msgs/StepperCommand`) - Send a command to the motor, either in position or run mode, can also be used to disengage the motor

Services
-----------------

* `~/zero` (`std_srvs/Trigger`) - Mark the current position to zero

Parameters
----------

* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the motor phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the digital input phidget is connected to a VINT hub.  Defaults to false.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device).  Defaults to 250 ms.
* `data_rate` (double) - The rate of data acquisition. Seems redundant with data_interval_ms
* `failsafe_time_ms` (double) - The failsafe time in ms, default to 1000
* `position_offset` (double) - Position to start at, default to 0.0
* `rescale_factor` (double) - Step multiplier applied to all the measurement.
  For a stepper with 1.8 degree step and a reduction of 14:1, (1./16) * (1.8/14) * (2*pi/360) makes sure all the output are in radians.
* `acceleration` (double) - Authorized acceleration, default to the max value supported by the driver.
* `velocity_limit` (double) - The maximum velocity, will be overloaded by messages published on `~/command`.
* `current_limit` (double) - Authorized current limit, default to the max value supported by the driver.
* `holding_current_limit` (double) - Authorized holding current limit, default to the max value supported by the driver.
