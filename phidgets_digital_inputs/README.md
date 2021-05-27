Phidgets digital inputs ROS 2 driver
====================================

This is the ROS 2 driver for Phidgets digital inputs.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_digital_inputs digital_inputs-launch.py

Published Topics
----------------

* `/digital_inputXX` (`std_msgs/Bool`) - The digital input state; one topic will be created for each digital input on the device.

Parameters
----------

* `serial` (int) - The serial number of the phidgets digital input to connect to.  If -1 (the default), connects to any digital input phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the digital input phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the digital input phidget is connected to a VINT hub.  Defaults to false.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device.  If positive, it will publish the data at that rate regardless of the acquisition interval.
