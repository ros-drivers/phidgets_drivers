Phidgets drivers for ROS Groovy (catkinised)
=============================================

Overview
---------------------------------------------

Drivers for the Phidgets devices. The metapackage includes:

 * `phidgets_c_api`: a package which downloads the Phidgets C API from phidgets.com,
   configures, builds and installs it within the current catkin workspace.

 * `phidgets_api`: a package which implements a C++ wrapper of the C API, providing
   a base Phidget class and various inherited classes for the different phidget devices.

 * Two packages exposing the functionality of specific phidgets using
   the ROS API : `phidgets_imu` and `phidgets_ir`.


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

Download the metapackage from the github repository:

    git clone -b groovy-devel https://github.com/muhrix/phidgets_drivers.git

Install any dependencies using rosdep:

    rosdep install phidgets_drivers

Compile the catkin workspace:

    cd ~/catkin_ws
    catkin_make

### Udev rules setup: ###

To set up the udev rules for the Phidgets USB devices, run the following commands:

    cd ~/catkin_ws
    sh src/phidgets_drivers/phidgets_c_api/setup-udev.sh

You will be prompted to type in your password.


For documentation regarding nodes, topics and services:
---------------------------------------------

http://ros.org/wiki/phidgets_drivers
