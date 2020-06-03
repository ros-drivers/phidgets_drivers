^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_gyroscope
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Improve the IMU calibration service (`#47 <https://github.com/ros-drivers/phidgets_drivers/issues/47>`_)
  * Change misleading IMU calibration log message.
  * Block 2 seconds in IMU calibration handler.
  * Make is_calibrated topic latched
  Forward-port of `#41 <https://github.com/ros-drivers/phidgets_drivers/issues/41>`_. Fixes `#42 <https://github.com/ros-drivers/phidgets_drivers/issues/42>`_.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
  Switch to libphidget22
* Resynchronize the times at periodic intervals.
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
* Add support for Phidgets Gyroscope sensors.
* Contributors: Chris Lalancette, Martin Günther
