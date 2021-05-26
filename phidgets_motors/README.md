Phidgets motor ROS 2 driver
===========================

This is the ROS 2 driver for Phidgets motor.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_motors motors-launch.py

Published Topics
----------------

* `/motor_duty_cycleXX` (`std_msgs/Float64`) - Get the motor duty cycle.  One topic is created for each motor attached.
* `/motor_back_emfXX` (`std_msgs/Float64`) - Get the motor back EMF value if supported by device.  One topic is created for each motor attached.

Subscribed Topics
-----------------

* `/set_motor_duty_cycleXX` (`std_msgs/Float64`) - Set the motor duty cycle.  One topic is created for each motor attached.

Parameters
----------

* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the motor phidget is connected to a VINT hub.  Defaults to 0.
* `braking_strength` (double) - The braking strength to apply when the duty cycle is 0.  Defaults to 0.0.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device).  Defaults to 250 ms.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
