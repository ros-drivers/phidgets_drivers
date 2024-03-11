Phidgets humidity ROS driver
============================

This is the ROS driver for Phidgets humidity sensor.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/humidity` (`std_msgs/Float64`) - The current relative humidity in %.

Parameters
----------
* `serial` (int) - The serial number of the phidgets humidity to connect to.  If -1 (the default), connects to any humidity phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the humidity phidget is connected to a VINT hub.  Defaults to 0.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 500 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
