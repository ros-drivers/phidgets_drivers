Phidgets interface kit ROS 2 driver
===================================

This is the meta-ROS 2 driver for Phidgets interface kit.
Since Phidgets interface kits are composed of digital inputs, digital outputs, and analog inputs, we can compose an IK driver by launching the individual drivers together.
This package only contains a launch file that makes it convenient to launch an IK.

Usage
-----

To run this driver standalone, do the following:

    ros2 launch phidgets_ik ik-launch.py
