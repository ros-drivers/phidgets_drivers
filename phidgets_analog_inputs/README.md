Phidgets analog inputs ROS driver
=================================

This is the ROS driver for Phidgets analog inputs.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/analog_inputXX` (`std_msgs/Float64`) - The analog input data; one topic will be created for each analog input on the device.

Parameters
---------
* `serial` (int) - The serial number of the phidgets analog input to connect to.  If -1 (the default), connects to any analog input phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the analog input phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the analog input phidget is connected to a VINT hub.  Defaults to false.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device.  Defaults to 250 ms.
* `publish_rate` (int) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
