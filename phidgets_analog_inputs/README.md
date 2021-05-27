Phidgets analog inputs ROS 2 driver
===================================

This is the ROS 2 driver for Phidgets analog inputs.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_analog_inputs analog_inputs-launch.py

Published Topics
----------------

* `/analog_inputXX` (`std_msgs/Float64`) - The analog input data; one topic will be created for each analog input on the device.

Parameters
---------

* `serial` (int) - The serial number of the phidgets analog input to connect to.  If -1 (the default), connects to any analog input phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the analog input phidget is connected to a VINT hub.  Defaults to 0.
* `is_hub_port_device` (bool) - Whether this device is directly connected to VINT hub port, or whether it is connected via another widget to the hub port.  Only used if the analog input phidget is connected to a VINT hub.  Defaults to false.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device).  Defaults to 250 ms.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `gainXX` (double) - The gain to apply to analog input XX.  The published data is scaled with the formula `pub_value = raw_value * gainXX + offsetXX`.
* `offsetXX` (double) - The offset to apply to analog input XX.  The published data is scaled with the formula `pub_value = raw_value * gainXX + offsetXX`.
