Phidgets motor ROS driver
=========================

This is the ROS driver for Phidgets motor.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/set_motor_duty_cycleXX` (`std_msgs/Float64`) - Set the motor duty cycle.  One topic is created for each motor attached.
* `/motor_duty_cycleXX` (`std_msgs/Float64`) - Get the motor duty cycle.  One topic is created for each motor attached.
* `/motor_back_emfXX` (`std_msgs/Float64`) - Get the motor back EMF value.  One topic is created for each motor attached.

Parameters
----------
* `serial` (int) - The serial number of the phidgets motor to connect to.  If -1 (the default), connects to any motor phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the motor phidget is connected to a VINT hub.  Defaults to 0.
* `braking_strength` (double) - The braking strength to apply when the duty cycle is 0.  Defaults to 0.0.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 250 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
