Phidgets digital inputs ROS driver
==================================

This is the ROS driver for Phidgets digital inputs.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/digital_outputXX` (`std_msgs/Bool`) - The state to set the digital output to; one topic will be created for each digital output on the device.

Services
--------
* `/set_digital_output` (`phidgets_msgs/SetDigitalOutput`) - A service to set the digital output `index` to the specified `state`.

Parameters
----------
* `serial` (int) - The serial number of the phidgets digital output to connect to.  If -1 (the default), connects to any digital output phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the digital output phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the digital output phidget is connected to a VINT hub.  Defaults to false.
