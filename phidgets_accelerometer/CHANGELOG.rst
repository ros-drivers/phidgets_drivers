^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_accelerometer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

2.2.2 (2022-02-17)
------------------

2.2.1 (2021-08-03)
------------------
* Update the ROS 2 readme files. (`#93 <https://github.com/ros-drivers/phidgets_drivers/issues/93>`_)
* Contributors: Chris Lalancette

2.2.0 (2021-05-20)
------------------

2.1.0 (2021-03-29)
------------------
* Don't publish messages that jumped back in time. (`#86 <https://github.com/ros-drivers/phidgets_drivers/issues/86>`_)
* Log synchronization window details at DEBUG level (`#84 <https://github.com/ros-drivers/phidgets_drivers/issues/84>`_)
* Get rid of deprecation warnings in Foxy. (`#75 <https://github.com/ros-drivers/phidgets_drivers/issues/75>`_)
* Switch header guards to _HPP SUFFIX.
* Contributors: Chris Lalancette, Martin Günther, Michael Grupp

2.0.2 (2020-06-01)
------------------

2.0.1 (2019-12-05)
------------------
* Switch the buildtoo_depend to ament_cmake_ros. (`#65 <https://github.com/ros-drivers/phidgets_drivers/issues/65>`_)
* Contributors: Chris Lalancette

2.0.0 (2019-12-05)
------------------
* Switch from NULL to nullptr.
* Make sure to initialize class member variables.
* Update READMEs to use "Published Topics" and "Subscribed Topics". (`#59 <https://github.com/ros-drivers/phidgets_drivers/issues/59>`_)
* Change launch output to "both" so it logs as well.
* Make publish_rate a double.
* Print out the serial number when connecting.
* Update documentation to mention device dependent fields.
* Port accelerometer to ROS 2.
* Ignore all packages for ROS 2 port.
* Fix wrong defaults for standard deviations (`#48 <https://github.com/ros-drivers/phidgets_drivers/issues/48>`_)
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Resynchronize the times at periodic intervals.
* Add launch files for all drivers.
* Add in try/catch blocks for connecting.
* Fixes from review.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Add support for Phidgets Accelerometer sensors.
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
