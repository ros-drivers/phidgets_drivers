Phidgets analog outputs ROS driver
==================================

This is the ROS driver for Phidgets analog outputs.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/analog_outputXX` (`std_msgs/Float64`) - The voltage to set at the analog output; one topic will be created for each analog output on the device.

Services
--------
* `/set_analog_output` (`phidgets_msgs/SetAnalogOutput`) - A service to set the analog output `index` to the specified `voltage`.

Parameters
----------
* `serial` (int) - The serial number of the phidgets analog output to connect to.  If -1 (the default), connects to any analog output phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the analog output phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the analog output phidget is connected to a VINT hub.  Defaults to false.
