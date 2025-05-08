^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_high_speed_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2025-05-08)
-------------------

1.0.9 (2024-03-13)
------------------

1.0.8 (2023-11-27)
------------------

1.0.7 (2023-03-02)
------------------
* BUGFIX: Z-channel index was not observed in reported positions (`#158 <https://github.com/ros-drivers/phidgets_drivers/issues/158>`_)
  * fix doxygen format and ensure initial values
  * BUGFIX: Encoder index was not used
* BUGFIX: Wrong speed conversion factor (`#155 <https://github.com/ros-drivers/phidgets_drivers/issues/155>`_)
  The current code assumed time intervals from the Phidgets API was microseconds, but it's actually milliseconds. Reported speeds are all wrong by a factor of 1e3.
* Contributors: Jose Luis Blanco-Claraco

1.0.6 (2022-12-01)
------------------
* Support configuring encoder data interval and IO Mode (`#137 <https://github.com/ros-drivers/phidgets_drivers/issues/137>`_)
* Merge pull request `#129 <https://github.com/ros-drivers/phidgets_drivers/issues/129>`_ from mintar/feat-pre-commit
  Add pre-commit, move from travis to GitHub actions, fix style
* Don't modify CMAKE_CXX_FLAGS
* Fix clang-format
* Add support for VINT networkhub (`#127 <https://github.com/ros-drivers/phidgets_drivers/issues/127>`_)
* BUGFIX: duplicated values for all encoders (`#124 <https://github.com/ros-drivers/phidgets_drivers/issues/124>`_)
* Contributors: Ben Schattinger, James Haley, Jose Luis Blanco-Claraco, Martin Günther

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
* Add launch files for all drivers.
* Add in try/catch blocks for connecting.
  If an error occurs, we catch it, print it, then re-throw it.
  This allows us to show a better error when using nodelets.
* Fixes from review.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
  This means we will only publish on changes.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Fix a small typo.
* Rewrite High Speed Encoder to use libphidget22.
* Rename Phidget class to Phidget21 class.
* Remove unused std_msgs dependency from Phidgets High Speed Encoder.
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Split custom messages into their own package.
* Rewrite the high speed encoder node.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Style cleanup.
* Switch to package format 2.
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
