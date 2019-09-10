Phidgets drivers for ROS 2
==========================

Overview
--------

Drivers for various [Phidgets](https://www.phidgets.com) devices. This package includes:

* `phidgets_api`: a package which downloads and builds the Phidgets C API from
   phidgets.com (as an external project). It also implements a C++ wrapper
   for the C API, providing some base Phidget helper functions and various classes
   for different phidget devices.

* ROS 2 nodes exposing the functionality of specific phidgets devices:

  * [`phidgets_accelerometer`](phidgets_accelerometer/README.md)

  * [`phidgets_analog_inputs`](phidgets_analog_inputs/README.md)

  * [`phidgets_digital_inputs`](phidgets_digital_inputs/README.md)

  * [`phidgets_digital_outputs`](phidgets_digital_outputs/README.md)

  * [`phidgets_gyroscope`](phidgets_gyroscope/README.md)

  * [`phidgets_high_speed_encoder`](phidgets_high_speed_encoder/README.md)

  * [`phidgets_ik`](phidgets_ik/README.md)

  * [`phidgets_magnetometer`](phidgets_magnetometer/README.md)

  * [`phidgets_motors`](phidgets_motors/README.md)

  * [`phidgets_spatial`](phidgets_spatial/README.md)

  * [`phidgets_temperature`](phidgets_temperature/README.md)

Installing
----------

### From source ###

Make sure you have ROS 2 Dashing installed: https://index.ros.org/doc/ros2/Installation/Dashing/

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your colcon workspace.
If, for instance, your workspace is `~/colcon_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/colcon_ws/src

Download the metapackage from the github repository (<ros_distro> may be `dashing`, ...)

    git clone -b <ros_distro> https://github.com/ros-drivers/phidgets_drivers.git

Install dependencies using rosdep:

    rosdep install phidgets_drivers

Alternatively, if rosdep does not work, install the following packages:

    sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev

Compile your colcon workspace:

    cd ~/colcon_ws
    colcon build

### Udev rules setup ###

**Note:** The following steps are only required when installing the package
from source. When installing a binary debian package of `phidgets_api` >= 0.7.8,
the udev rules are set up automatically.

Make sure your colcon workspace has been successfully compiled.
To set up the udev rules for the Phidgets USB devices, run the following commands:

    sudo cp ~/colcon_ws/src/phidgets_drivers/phidgets_api/debian/udev /etc/udev/rules.d/99-phidgets.rules
    sudo udevadm control --reload-rules

Afterwards, disconnect the USB cable and plug it in again (or run `sudo udevadm trigger`).


For documentation regarding nodes, topics, etc:
---------------------------------------------

http://ros.org/wiki/phidgets_drivers
