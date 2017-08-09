Phidgets drivers for ROS
========================

Overview
---------------------------------------------

Drivers for the Phidgets devices. This Catkin metapackage includes:

 * `phidgets_api`: a package which downloads and builds the Phidgets C API from
   phidgets.com (as an external project). It also implements a C++ wrapper
   for the C API, providing a base Phidget class and various inherited classes
   for different phidget devices.

 * ROS nodes exposing the functionality of specific phidgets devices using: 
   * `phidgets_imu` 
   * `phidgets_ir`
   * `phidgets_high_speed_encoder`

Installing
---------------------------------------------

### From source ###

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

#### Install ROS wrapper for Phidgets driver ####

Install dependencies:

    sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev

Execute:

    cd ~/catkin_ws/src
    git clone https://github.com/ipa320/cob_extern.git
    # Only build what we need:
    mv cob_extern/libphidgets . && rm -fr cob_extern

#### Build this package ####

Download the package from GitHub:

    cd ~/catkin_ws/src
    git clone -b $ROS_DISTRO https://github.com/ros-drivers/phidgets_drivers.git

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make

### Udev rules setup: ###

Make sure your catkin workspace has been successfully compiled.
To set up the udev rules for the Phidgets USB devices, run the following commands:

    cd ~/catkin_ws
    sh src/phidgets_drivers/phidgets_api/share/setup-udev.sh

You will be prompted to type in your password.


For documentation regarding nodes, topics, etc:
---------------------------------------------

http://ros.org/wiki/phidgets_drivers
