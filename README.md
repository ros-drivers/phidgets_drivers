Phidgets drivers for ROS
========================

Overview
--------

Drivers for various [Phidgets](https://www.phidgets.com) devices. This Catkin metapackage includes:

* `phidgets_api`: a package which downloads and builds the Phidgets C API from
   phidgets.com (as an external project). It also implements a C++ wrapper
   for the C API, providing some base Phidget helper functions and various classes
   for different phidget devices.

* ROS nodelets exposing the functionality of specific phidgets devices:

  * [`phidgets_accelerometer`](phidgets_accelerometer/README.md)

  * [`phidgets_analog_inputs`](phidgets_analog_inputs/README.md)

  * [`phidgets_analog_outputs`](phidgets_analog_outputs/README.md)

  * [`phidgets_current_inputs`](phidgets_current_inputs/README.md)

  * [`phidgets_digital_inputs`](phidgets_digital_inputs/README.md)

  * [`phidgets_digital_outputs`](phidgets_digital_outputs/README.md)

  * [`phidgets_gyroscope`](phidgets_gyroscope/README.md)

  * [`phidgets_high_speed_encoder`](phidgets_high_speed_encoder/README.md)

  * [`phidgets_humidity`](phidgets_humidity/README.md)

  * [`phidgets_ik`](phidgets_ik/README.md)

  * [`phidgets_magnetometer`](phidgets_magnetometer/README.md)

  * [`phidgets_motors`](phidgets_motors/README.md)

  * [`phidgets_spatial`](phidgets_spatial/README.md)

  * [`phidgets_temperature`](phidgets_temperature/README.md)

Migrating from earlier versions
-------------------------------

Prior to ROS Noetic, this library was based on the older libphidget21 library.  While it is still supported by Phidgets, they no longer add new features to it and [VINT hub](https://www.phidgets.com/?tier=3&catid=2&pcid=1&prodid=643) style devices cannot be used with it.  In ROS Noetic, this library was rewritten to use the libphidget22 library, which is the latest supported and contains all of the newest features.  However, the new libphidget22 library is very different from the old libphidget21 library, and some of those differences leak through to the drivers themselves.  The following is a list of things that may help in migrating to the new drivers.

## General ##
* All drivers now have nodelets.
* No drivers have nodes anymore.  While this makes debugging somewhat harder, nodelets are the only way to support devices on a VINT hub.
* The "serial_number" parameter has been renamed to "serial".

## Specific nodes ##
### IMU ###
* The "imu" node was renamed to the "spatial" node.
* Diagnostics have been removed from the spatial driver.  They will be reinstated later.
* The "period" parameter has been renamed to "data_interval_ms".
* The default "frame_id" has been changed from "imu" to "imu_link" to comply with [REP-0145](http://www.ros.org/reps/rep-0145.html).

### IK ###
* The "ik" node is now just a launch file which composes an Analog Input, Digital Input, and Digital Output nodelet together.

Installing
----------

### From source ###

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

Download the metapackage from the github repository (<ros_distro> may be `groovy`, `hydro`, `indigo`...):

    git clone -b <ros_distro> https://github.com/ros-drivers/phidgets_drivers.git

Install dependencies using rosdep:

    rosdep install phidgets_drivers

Alternatively, if rosdep does not work, install the following packages:

    sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make

### Udev rules setup ###

**Note:** The following steps are only required when installing the package
from source. When installing a binary debian package of `phidgets_api` >= 0.7.8,
the udev rules are set up automatically.

Make sure your catkin workspace has been successfully compiled.
To set up the udev rules for the Phidgets USB devices, run the following commands:

    roscd phidgets_api
    sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
    sudo udevadm control --reload-rules

Afterwards, disconnect the USB cable and plug it in again (or run `sudo udevadm trigger`).


For documentation regarding nodes, topics, etc:
---------------------------------------------

http://ros.org/wiki/phidgets_drivers


pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```
