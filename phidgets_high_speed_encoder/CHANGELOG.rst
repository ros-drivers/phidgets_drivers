^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_high_speed_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* BUGFIX: Z-channel index was not observed in reported positions (`#158 <https://github.com/ros-drivers/phidgets_drivers/issues/158>`_)
  * fix doxygen format and ensure initial values
  * BUGFIX: Encoder index was not used
* BUGFIX: Wrong speed conversion factor (`#157 <https://github.com/ros-drivers/phidgets_drivers/issues/157>`_)
  The current code assumed time intervals from the Phisgets API was microseconds, but it's actually milliseconds. Reported speeds are all wrong by a factor of 1e3.
* Contributors: Jose Luis Blanco-Claraco, Martin Günther

2.3.0 (2022-04-13)
------------------
* Merge pull request `#132 <https://github.com/ros-drivers/phidgets_drivers/issues/132>`_ from mintar/feat-pre-commit-ros2
  [galactic] Add pre-commit, move from travis to GitHub actions, fix style
* Fix clang-format
* BUGFIX: duplicated values for all encoders (`#126 <https://github.com/ros-drivers/phidgets_drivers/issues/126>`_)
* Contributors: Martin Günther

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
* Get rid of deprecation warnings in Foxy. (`#75 <https://github.com/ros-drivers/phidgets_drivers/issues/75>`_)
* Switch header guards to _HPP SUFFIX.
* Contributors: Chris Lalancette

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
* Switch from NULL to nullptr.
* Update READMEs to use "Published Topics" and "Subscribed Topics". (`#59 <https://github.com/ros-drivers/phidgets_drivers/issues/59>`_)
* Change launch output to "both" so it logs as well.
* Make publish_rate a double.
* Print out the serial number when connecting.
* Port high speed encoder to ROS 2.
* Ignore all packages for ROS 2 port.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Add launch files for all drivers.
* Add in try/catch blocks for connecting.
* Fixes from review.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Fix a small typo.
* Rewrite High Speed Encoder to use libphidget22.
* Rename Phidget class to Phidget21 class.
* Switch to C++14.
* Remove unused std_msgs dependency from Phidgets High Speed Encoder.
* Merge pull request `#36 <https://github.com/ros-drivers/phidgets_drivers/issues/36>`_ from clalancette/phidget-cleanup2
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Split custom messages into their own package.
* Rewrite the high speed encoder node.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Small cleanups throughout the code.
* Style cleanup.
* Switch to package format 2.
* Cleanup spacing in all of the CMakeLists.txt
* Contributors: Chris Lalancette, Martin Günther

0.7.9 (2019-06-28)
------------------

0.7.8 (2019-05-06)
------------------
* phidgets_high_speed_encoder: fix missing tick2rad values (`#30 <https://github.com/ros-drivers/phidgets_drivers/issues/30>`_)
* Contributors: Charles Brian Quinn

0.7.7 (2018-09-18)
------------------

0.7.6 (2018-08-09)
------------------

0.7.5 (2018-01-31)
------------------

0.7.4 (2017-10-04)
------------------
* Merge pull request `#15 <https://github.com/ros-drivers/phidgets_drivers/issues/15>`_ from jlblancoc/kinetic
  Add Phidgets high-speed encoder package
* Contributors: Jose Luis Blanco-Claraco, Geoff Viola, Martin Günther

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
