^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_magnetometer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#129 <https://github.com/ros-drivers/phidgets_drivers/issues/129>`_ from mintar/feat-pre-commit
  Add pre-commit, move from travis to GitHub actions, fix style
* Don't modify CMAKE_CXX_FLAGS
* Fix clang-format
* Add support for VINT networkhub (`#127 <https://github.com/ros-drivers/phidgets_drivers/issues/127>`_)
* Contributors: James Haley, Martin Günther

1.0.5 (2022-02-17)
------------------

1.0.4 (2021-10-22)
------------------

1.0.3 (2021-09-29)
------------------

1.0.2 (2021-03-09)
------------------
* Don't publish messages that jumped back in time. (`#85 <https://github.com/ros-drivers/phidgets_drivers/issues/85>`_)
* Log synchronization window details at DEBUG level. (`#82 <https://github.com/ros-drivers/phidgets_drivers/issues/82>`_)
* Contributors: Michael Grupp

1.0.1 (2020-06-04)
------------------
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.0 (2020-06-03)
------------------
* Fix wrong defaults for standard deviations (`#48 <https://github.com/ros-drivers/phidgets_drivers/issues/48>`_)
  The old parameter defaults were wrong:
  | parameter                 | old default                       | new default                        |
  |                           |                                   |                                    |
  | angular_velocity_stdev    | 0.000349056 rad/s (= 0.02 deg/s)  | 0.001658 rad/s    (= 0.095deg/s)   |
  | linear_acceleration_stdev | 0.002943 m/s^2 (= 0.0003 g)       | 0.002745862 m/s^2 (= 0.00028 g)    |
  | magnetic_field_stdev      | 0.001658 rad/s (= 0.095deg/s)     | 1.1e-7 T          (= 1.1 mG)       |
  Notes: T = Tesla, mG = milligauss
  This is a forward-port of `#46 <https://github.com/ros-drivers/phidgets_drivers/issues/46>`_ to noetic.
  Specifications come from the PhidgetSpatial Precision 3/3/3 1044_0 data sheet: https://www.phidgets.com/?&prodid=32
* Update maintainers in package.xml
* Switch to libphidget22
* Resynchronize the times at periodic intervals.
* Add launch files for all drivers.
* Fix small typos in README and LICENSE files.
* Add in try/catch blocks for connecting.
  If an error occurs, we catch it, print it, then re-throw it.
  This allows us to show a better error when using nodelets.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
  This means we will only publish on changes.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Add support for Phidgets Magnetometer sensors.
* Contributors: Chris Lalancette, Martin Günther
