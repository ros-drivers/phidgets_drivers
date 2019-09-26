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

Concerning Phidgets
-------------------

Phidgets are typically plugged into USB on a host computer (though there are
wireless ones, they will be ignored here).  In the "old-style" Phidgets, there
was one USB plug per device.  So if you have a
[temperature Phidget](https://www.phidgets.com/?tier=3&catid=14&pcid=12&prodid=1042),
and an [accelerometer Phidget](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=1026),
they take up 2 USB plugs on the computer.  These "old-style" Phidgets are still
around and still available for sale, but most of the new development and
sensors are in "new-style" Phidgets.  In "new-style" Phidgets, a
[VINT hub](https://www.phidgets.com/?tier=3&catid=2&pcid=1&prodid=643) is
connected to the host computer via USB, and then the other Phidgets connect to
a port on the VINT hub.  Most of the "old-style" Phidget functions (temperature,
acclerometer, etc.) are also available as "new-style" Phidgets, and most new
functionality is only available as VINT devices.

### Identifying Phidgets devices ###

All Phidgets that plug directly into a USB port (including the VINT hub) have a
unique serial number. This serial number is printed on the back of the device,
and is also printed out by the phidgets drivers when they start up.  The serial
number can be specified as a parameter when the driver starts up; otherwise, the
default is to connect to *any* Phidgets that are of the correct type at startup.

Uniquely identifying a "new-style" Phidget also requires one more piece of
information, which is the VINT hub port it is connected to.  This also must be
provided as a parameter when starting up the driver.

Note that there are "smart" and "simple" VINT hub devices.  "Smart" devices have
their own microcontrollers on board and use a protocol to communicate with the
VINT hub.

"Simple" VINT hub devices don't have a microcontroller.  They just provide
or accept a voltage from the VINT hub port (which can act as a digital input,
digital output, or analog inputs).

Whether the Phidget is "smart" or "simple" can be determined by looking at the
"Connection and Compatibility" portion of the webpage for the individual sensor.
If the device is "smart", then "is_hub_port_device" must be set to "false"
when launching a driver; if the device is "simple", then "is_hub_port_device"
must be set to "true".

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
