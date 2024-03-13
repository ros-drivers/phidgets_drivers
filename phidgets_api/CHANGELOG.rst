^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2024-03-13)
------------------

2.3.2 (2023-11-27)
------------------
* added new parameters for spatial precision MOT0109 onwards
* added support for phidget spatial onboard orientation estimation
* Contributors: Malte kl. Piening, Martin Günther

2.3.1 (2023-03-03)
------------------
* adding support for analog outputs for ROS2 (`#145 <https://github.com/ros-drivers/phidgets_drivers/issues/145>`_)
* Contributors: Alexis Fetet

2.3.0 (2022-04-13)
------------------

2.2.2 (2022-02-17)
------------------
* spatial: Add attach + detach handlers
* Fix some clang-tidy warnings
* Fix typo in error message (`#104 <https://github.com/ros-drivers/phidgets_drivers/issues/104>`_)
* Contributors: Martin Günther

2.2.1 (2021-08-03)
------------------

2.2.0 (2021-05-20)
------------------

2.1.0 (2021-03-29)
------------------
* Switch header guards to _HPP SUFFIX.
* Remove unnecessary cstddef.
* Contributors: Chris Lalancette

2.0.2 (2020-06-01)
------------------
* Use '=default' for default destructors. (`#66 <https://github.com/ros-drivers/phidgets_drivers/issues/66>`_)
* Contributors: Chris Lalancette

2.0.1 (2019-12-05)
------------------
* Switch the buildtoo_depend to ament_cmake_ros. (`#65 <https://github.com/ros-drivers/phidgets_drivers/issues/65>`_)
* Contributors: Chris Lalancette

2.0.0 (2019-12-05)
------------------
* Remove unnecessary base-class initialization.
* Get rid of C-style casts.
* Make sure what() method is marked as override.
* Rename method parameter name to match implementation.
* Make sure to include cstddef for libphidget22.h include.
* Make sure to initialize class member variables.
* Reformat using clang
* Make sure to set the VoltageRange to AUTO at the beginning.
* Only publish motor_back_emf if supported by DC Motor device (`#53 <https://github.com/ros-drivers/phidgets_drivers/issues/53>`_)
* Print out the serial number when connecting.
* Port phidgets_api to ament.
* Ignore all packages for ROS 2 port.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Add in try/catch blocks for connecting.
* Fixes from review.
* Implement data interval setting for analog inputs.
* Add in the license files and add to the headers.
* Completely remove libphidget21.
* Rewrite Motor Phidget to use libphidget22.
* Rewrite High Speed Encoder to use libphidget22.
* Rewrite IR to use libphidget22.
* Rewrite IMU using libphidget22.
* Add support for Phidgets Magnetometer sensors.
* Add support for Phidgets Gyroscope sensors.
* Add support for Phidgets Accelerometer sensors.
* Add in support for Phidgets Temperature sensors.
* Rewrite phidgets_ik on top of libphidget22 classes.
* Add in support for Phidgets Analog inputs.
* Add in support for Phidgets Digital Inputs.
* Add in support for Phidgets Digital Outputs.
* Add in libphidget22 helper functions.
* Rename Phidget class to Phidget21 class.
* Switch to C++14.
* Merge pull request `#36 <https://github.com/ros-drivers/phidgets_drivers/issues/36>`_ from clalancette/phidget-cleanup2
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Remove unused indexHandler from Encoder class.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Small cleanups throughout the code.
* Push libphidgets API calls down to phidgets_api.
* Quiet down the to-be-overridden callbacks.
* Consistently use nullptr instead of 0.
* Make the phidget_api destructors virtual.
* Style cleanup.
* Move libusb dependency into the libphidget21 package.xml.
* Switch to package format 2.
* Cleanup spacing in all of the CMakeLists.txt
* Contributors: Chris Lalancette, Martin Günther, Peter Polidoro

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
