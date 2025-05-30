^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_magnetometer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-05-30)
------------------
* Merge pull request `#191 <https://github.com/ros-drivers/phidgets_drivers/issues/191>`_ from mintar/remove-ament-target-dependencies
  [kilted] Update deprecated call to ament_target_dependencies
* Update to C++ standard 17
* [kilted] Update deprecated call to ament_target_dependencies
* Contributors: Martin Günther

2.3.4 (2025-05-19)
------------------
* Upgrade to CMake 3.8, add file depend (`#189 <https://github.com/ros-drivers/phidgets_drivers/issues/189>`_)
* Contributors: Martin Günther

2.3.3 (2024-03-13)
------------------
* Add support for VINT networkhub (`#172 <https://github.com/ros-drivers/phidgets_drivers/issues/172>`_)
  This is a port of `#127 <https://github.com/ros-drivers/phidgets_drivers/issues/127>`_ to ROS2.
  Closes `#135 <https://github.com/ros-drivers/phidgets_drivers/issues/135>`_.
* Contributors: Martin Günther

2.3.2 (2023-11-27)
------------------

2.3.1 (2023-03-03)
------------------

2.3.0 (2022-04-13)
------------------
* Merge pull request `#132 <https://github.com/ros-drivers/phidgets_drivers/issues/132>`_ from mintar/feat-pre-commit-ros2
  [galactic] Add pre-commit, move from travis to GitHub actions, fix style
* Fix clang-format
* Contributors: Martin Günther

2.2.2 (2022-02-17)
------------------

2.2.1 (2021-08-03)
------------------
* Make the magnetometer corrections optional again. (`#95 <https://github.com/ros-drivers/phidgets_drivers/issues/95>`_)
* Update the ROS 2 readme files. (`#93 <https://github.com/ros-drivers/phidgets_drivers/issues/93>`_)
* Contributors: Chris Lalancette

2.2.0 (2021-05-20)
------------------
* Make sure to declare the type while declaring the parameter. (`#89 <https://github.com/ros-drivers/phidgets_drivers/issues/89>`_)
* Contributors: Chris Lalancette

2.1.0 (2021-03-29)
------------------
* Don't publish messages that jumped back in time. (`#86 <https://github.com/ros-drivers/phidgets_drivers/issues/86>`_)
* Log synchronization window details at DEBUG level (`#84 <https://github.com/ros-drivers/phidgets_drivers/issues/84>`_)
* Get rid of deprecation warnings in Foxy. (`#75 <https://github.com/ros-drivers/phidgets_drivers/issues/75>`_)
* Switch header guards to _HPP SUFFIX.
* Contributors: Chris Lalancette, Martin Günther, Michael Grupp

2.0.2 (2020-06-01)
------------------
* Release build fixes (`#67 <https://github.com/ros-drivers/phidgets_drivers/issues/67>`_)
* Contributors: Chris Lalancette

2.0.1 (2019-12-05)
------------------
* Switch the buildtoo_depend to ament_cmake_ros. (`#65 <https://github.com/ros-drivers/phidgets_drivers/issues/65>`_)
* Contributors: Chris Lalancette

2.0.0 (2019-12-05)
------------------
* Make sure exceptions get caught by reference.
* Switch from NULL to nullptr.
* Make sure to initialize class member variables.
* Update READMEs to use "Published Topics" and "Subscribed Topics". (`#59 <https://github.com/ros-drivers/phidgets_drivers/issues/59>`_)
* Change launch output to "both" so it logs as well.
* Make publish_rate a double.
* Print out the serial number when connecting.
* Update documentation to mention device dependent fields.
* Port magnetometer to ROS 2.
* Ignore all packages for ROS 2 port.
* Fix wrong defaults for standard deviations (`#48 <https://github.com/ros-drivers/phidgets_drivers/issues/48>`_)
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Resynchronize the times at periodic intervals.
* Add launch files for all drivers.
* Fix small typos in README and LICENSE files.
* Add in try/catch blocks for connecting.
* Fixes from review.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Add support for Phidgets Magnetometer sensors.
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

0.7.3 (2017-06-30)
------------------

0.7.2 (2017-06-02)
------------------

0.7.1 (2017-05-22)
------------------

0.7.0 (2017-02-17 17:40)
------------------------

0.2.3 (2017-02-17 12:11)
------------------------

0.2.2 (2015-03-23)
------------------

0.2.1 (2015-01-15)
------------------
