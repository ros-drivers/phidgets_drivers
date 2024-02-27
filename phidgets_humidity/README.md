Phidgets temperature ROS driver
===========================

This is the ROS driver for Phidgets temperature.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/temperature` (`std_msgs/Float64`) - The current temperature in degrees Celsius.

Parameters
----------
* `serial` (int) - The serial number of the phidgets gyroscope to connect to.  If -1 (the default), connects to any gyroscope phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the gyroscope phidget is connected to a VINT hub.  Defaults to 0.
* `thermocouple_type` (int) - The type of thermocouple that is connected.  Allowed values are 0 (for not a thermocouple, the default), 1 (for a J-Type), 2 (for a K-Type), 3 (for an E-Type), or 4 (for a T-Type).  See https://www.phidgets.com/docs/Thermocouple_Primer for more information.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 250 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
