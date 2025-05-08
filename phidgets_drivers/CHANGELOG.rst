^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2025-05-08)
-------------------

1.0.9 (2024-03-13)
------------------

1.0.8 (2023-11-27)
------------------

1.0.7 (2023-03-02)
------------------

1.0.6 (2022-12-01)
------------------

1.0.5 (2022-02-17)
------------------

1.0.4 (2021-10-22)
------------------

1.0.3 (2021-09-29)
------------------

1.0.2 (2021-03-09)
------------------

1.0.1 (2020-06-04)
------------------
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.0 (2020-06-03)
------------------
* Update maintainers in package.xml
* Switch to libphidget22
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
