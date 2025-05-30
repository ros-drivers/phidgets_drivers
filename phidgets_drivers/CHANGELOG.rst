^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-05-30)
------------------

2.3.4 (2025-05-19)
------------------
* Upgrade to CMake 3.8, add file depend (`#189 <https://github.com/ros-drivers/phidgets_drivers/issues/189>`_)
* Contributors: Martin Günther

2.3.3 (2024-03-13)
------------------

2.3.2 (2023-11-27)
------------------

2.3.1 (2023-03-03)
------------------

2.3.0 (2022-04-13)
------------------

2.2.2 (2022-02-17)
------------------

2.2.1 (2021-08-03)
------------------

2.2.0 (2021-05-20)
------------------

2.1.0 (2021-03-29)
------------------

2.0.2 (2020-06-01)
------------------

2.0.1 (2019-12-05)
------------------

2.0.0 (2019-12-05)
------------------
* Port phidgets_drivers to ROS 2.
* Ignore all packages for ROS 2 port.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Completely remove libphidget21.
* Rewrite Motor Phidget to use libphidget22.
* Rewrite IMU using libphidget22.
* Add support for Phidgets Magnetometer sensors.
* Add support for Phidgets Gyroscope sensors.
* Add support for Phidgets Accelerometer sensors.
* Add in support for Phidgets Temperature sensors.
* Add in support for Phidgets Analog inputs.
* Add in support for Phidgets Digital Inputs.
* Add in support for Phidgets Digital Outputs.
* Add in libphidget22 package.
* Merge pull request `#36 <https://github.com/ros-drivers/phidgets_drivers/issues/36>`_ from clalancette/phidget-cleanup2
* Split custom messages into their own package.
* Add in phidgets_ik to the phidgets_drivers metapackage.
* Switch to package format 2.
* Contributors: Chris Lalancette, Martin Günther

0.7.9 (2019-06-28)
------------------

0.7.8 (2019-05-06)
------------------

0.7.7 (2018-09-18)
------------------

0.7.6 (2018-08-09)
------------------

0.7.5 (2018-01-31)
------------------

0.7.4 (2017-10-04)
------------------
* Add phidgets_high_speed_encoder to metapackage
* Contributors: Jose Luis Blanco-Claraco, Martin Günther

0.7.3 (2017-06-30)
------------------

0.7.2 (2017-06-02)
------------------

0.7.1 (2017-05-22)
------------------

0.7.0 (2017-02-17)
------------------
* Remove phidgets_ir package
  It was a stub anyway. If somebody has such a device and cares to expose
  the data via ROS topics, this can be revived.
* Contributors: Martin Günther

0.2.3 (2017-02-17)
------------------
* Update package.xml meta info
* Contributors: Martin Günther

0.2.2 (2015-03-23)
------------------

0.2.1 (2015-01-15)
------------------
* phidgets_drivers: removed phidgets_c_api dependency
* Updated version, maintainer and author information
* Deleted comments within files of all packages
* phidgets_drivers: converted stack into metapackage
* Contributors: Murilo FM
