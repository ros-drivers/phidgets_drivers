^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2025-05-08)
-------------------
* Contributors: Martin Günther, Ben Schattinger

1.0.9 (2024-03-13)
------------------
* Add support for Phidgets Humidity sensors (`#173 <https://github.com/ros-drivers/phidgets_drivers/issues/173>`_)
* Contributors: Gary Edwards

1.0.8 (2023-11-27)
------------------

1.0.7 (2023-03-02)
------------------

1.0.6 (2022-12-01)
------------------
* Merge pull request `#153 <https://github.com/ros-drivers/phidgets_drivers/issues/153>`_ from naturerobots/noetic
  Add support for onboard orientation estimation and other new PhidgetSpatial features of MOT0109 and onwards
* Support configuring encoder data interval and IO Mode (`#137 <https://github.com/ros-drivers/phidgets_drivers/issues/137>`_)
* Merge pull request `#129 <https://github.com/ros-drivers/phidgets_drivers/issues/129>`_ from mintar/feat-pre-commit
  Add pre-commit, move from travis to GitHub actions, fix style
* Don't modify CMAKE_CXX_FLAGS
* Contributors: Ben Schattinger, Malte kl. Piening, Martin Günther

1.0.5 (2022-02-17)
------------------
* spatial: Add attach + detach handlers
* Fix some clang-tidy warnings
* Contributors: Martin Günther

1.0.4 (2021-10-22)
------------------

1.0.3 (2021-09-29)
------------------
* Fix typo in error message
* Add Analog Outputs (`#103 <https://github.com/ros-drivers/phidgets_drivers/issues/103>`_)
* Contributors: Carsten Plasberg, Martin Günther

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
* Add in try/catch blocks for connecting.
  If an error occurs, we catch it, print it, then re-throw it.
  This allows us to show a better error when using nodelets.
* Fixes from review.
* Implement data interval setting for analog inputs.
* Add in the license files and add to the headers.
* Completely remove libphidget21.
  Nothing else depends on it.
* Rewrite Motor Phidget to use libphidget22.
  Also implement a new motor node.
* Rewrite High Speed Encoder to use libphidget22.
* Rewrite IR to use libphidget22.
* Rewrite IMU using libphidget22.
* Add support for Phidgets Magnetometer sensors.
* Add support for Phidgets Gyroscope sensors.
* Add support for Phidgets Accelerometer sensors.
* Add in support for Phidgets Temperature sensors.
* Rewrite phidgets_ik on top of libphidget22 classes.
  This should be equivalent functionality, but allows the use
  of VINT devices now.
* Add in support for Phidgets Analog inputs.
* Add in support for Phidgets Digital Inputs.
* Add in support for Phidgets Digital Outputs.
* Add in libphidget22 helper functions.
* Rename Phidget class to Phidget21 class.
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Remove unused indexHandler from Encoder class.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Push libphidgets API calls down to phidgets_api.
* Quiet down the to-be-overridden callbacks.
* Consistently use nullptr instead of 0.
* Make the phidget_api destructors virtual.
* Style cleanup.
* Move libusb dependency into the libphidget21 package.xml.
* Switch to package format 2.
* Contributors: Chris Lalancette, Martin Günther

0.7.9 (2019-06-28)
------------------
* Add missing OnInputChange handler (`#33 <https://github.com/ros-drivers/phidgets_drivers/issues/33>`_)
* Contributors: Kai Hermann

0.7.8 (2019-05-06)
------------------
* Install udev rules on binary package installation
* Contributors: Martin Günther

0.7.7 (2018-09-18)
------------------

0.7.6 (2018-08-09)
------------------

0.7.5 (2018-01-31)
------------------
* Add support for the phidgets_ik (Phidgets Interface Kit)
* Contributors: Russel Howe, James Sarrett, Martin Günther

0.7.4 (2017-10-04)
------------------
* Fix typo and doxygen docs
* Contributors: Jose Luis Blanco Claraco, Martin Günther

0.7.3 (2017-06-30)
------------------

0.7.2 (2017-06-02)
------------------

0.7.1 (2017-05-22)
------------------
* Set event handlers for motor + encoder APIs
* Added basic motor api
* Added basic encoder board api
* Contributors: Zach Anderson, Martin Günther

0.7.0 (2017-02-17)
------------------
* Use our own libphidget21 instead of external libphidgets
* Contributors: Martin Günther

0.2.3 (2017-02-17)
------------------
* Add IMU diagnostics (`#24 <https://github.com/ccny-ros-pkg/phidgets_drivers/pull/24>`_)
* Contributors: Mani Monajjemi, Keshav Iyengar, Martin Günther

0.2.2 (2015-03-23)
------------------
* phidgets_api: updated build/installation rules to use 3rd party libphdigets ROS package
* phidgets_api: updated package details
* phidgets_api: added copy of udev rule to package and updated path in script
* phidgets_api: updated path to libphidgets header file
* phidgets_api: removed license and header file of phidgets library
* Contributors: Murilo FM

0.2.1 (2015-01-15)
------------------
* phidgets_api: add libusb dependency
  This caused Jenkins CI tests to fail.
* phidgets_api: fix case in CMakeLists
* phidgets_api: added GNU LGPLv3 copy (phidget21.h)
* phidgets_api: updated license and author information
* phidgets_api: added script to setup udev rules for Phidgets devices
* phidgets_api: added libphidget21 dependency as cmake external project
* phidgets_api: updated path to libphidget header file
* phidgets_api: added libphidget header file to package
* phidgets_api: removed phidgets_c_api dependency
* Deleted comments within files of all packages
* Catkinised packages
* added missing cmakelists
* added api, imu and ir
* removed deps directory
* initial commit
* Contributors: Ivan Dryanovski, Martin Günther, Murilo FM
