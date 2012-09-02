Phidgets drivers for ROS
=============================================

Overview
---------------------------------------------

Drivers for the Phidgets devices. The stack includes:

 * `phidgets_c_api`: a meta-package which downloads the Phidgets C API 
   from phidgets.com and installs it locally within the package's directory.

 * `phidgets_api`: a C++ wrapper of the C API which provides a base Phidget
   class and various inherited classes for the different phidget devices.

 * various packages exposing the functionality of specific phidgets using
   the ROS API.

Installing
---------------------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and add it to `$ROS_PACKAGE_PATH`.

Make sure you have git installed:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/phidgets_drivers.git

Install any dependencies using rosdep.

    rosdep install phidgets_drivers

Compile the stack:

    rosmake phidgets_drivers

### Rules set up: ###

To set up your udev rules for the phidgets USB devices, run this script:

    phidgets_c_api/setup-udev.sh

You will be prompted for a sudo password.

More info
---------------------------------------------

http://ros.org/wiki/phidgets_drivers

