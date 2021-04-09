^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.11 (2021-04-09)
-------------------

0.7.10 (2020-06-03)
-------------------
* Update maintainers in package.xml
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Remove unused indexHandler from Encoder class.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Small cleanups throughout the code.
* Push libphidgets API calls down to phidgets_api.
* Quiet down the to-be-overridden callbacks.
* Consistently use nullptr instead of 0.
* Make the phidget_api destructors virtual.
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
