^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_imu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.10 (2025-05-08)
-------------------

1.0.9 (2024-03-13)
------------------

1.0.8 (2023-11-27)
------------------
* Only set magnetometer gain if param is set (`#170 <https://github.com/ros-drivers/phidgets_drivers/issues/170>`_)
* Contributors: Martin Günther

1.0.7 (2023-03-02)
------------------

1.0.6 (2022-12-01)
------------------
* Merge pull request `#153 <https://github.com/ros-drivers/phidgets_drivers/issues/153>`_ from naturerobots/noetic
  Add support for onboard orientation estimation and other new PhidgetSpatial features of MOT0109 and onwards
* Merge pull request `#129 <https://github.com/ros-drivers/phidgets_drivers/issues/129>`_ from mintar/feat-pre-commit
  Add pre-commit, move from travis to GitHub actions, fix style
* Don't modify CMAKE_CXX_FLAGS
* Fix clang-format
* Add support for VINT networkhub (`#127 <https://github.com/ros-drivers/phidgets_drivers/issues/127>`_)
* Contributors: James Haley, Malte kl. Piening, Martin Günther

1.0.5 (2022-02-17)
------------------
* spatial: Fix behavior after USB reattachment
  The Phidged Spatial never recovered after detaching and reattaching to
  the USB port. This commit fixes that.
  The cause of the failure was the following:
  * After reattachment, the device time stamp restarts from 0. This caused
  our driver to throw the following error messages:
  [ WARN]: Time went backwards [...]! Not publishing message.
  * However, the data interval is also reset to the default of 256 ms. If
  the parameter data_interval_ms is set to something else, this caused the
  arriving data to always be outside the acceptable window for
  synchronization. Therefore, synchronization never happened, and the
  driver never resumed publishing messages.
  This commit fixes the bug by setting the appropriate data interval after
  each reattachment and forcing a resynchronization immediately.
* spatial: Add attach + detach handlers
* spatial: Fix publishing of invalid mag readings
* spatial.launch: Remove use_magnetic_field_msg
* Contributors: Martin Günther

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
* Finish removing launch file from phidgets_spatial.
* Rewrite IMU using libphidget22.
* Contributors: Chris Lalancette, Martin Günther

0.7.9 (2019-06-28)
------------------

0.7.8 (2019-05-06)
------------------

0.7.7 (2018-09-18)
------------------
* Add parameter use_imu_time (default true) (`#27 <https://github.com/ros-drivers/phidgets_drivers/issues/27>`_)
  Setting use_imu_time to false will disable the imu time calibration and
  always use the Host time, i.e. ros::Time::now().
* Contributors: Jochen Sprickerhof

0.7.6 (2018-08-09)
------------------
* phidgets_imu: Ensure strictly ordered timestamps (`#26 <https://github.com/ros-drivers/phidgets_drivers/issues/26>`_)
  Fixes `#17 <https://github.com/ros-drivers/phidgets_drivers/issues/17>`_.
* Contributors: Michael Grupp, Martin Günther

0.7.5 (2018-01-31)
------------------
* phidgets_imu: Add roslaunch_add_file_check
* phidgets_imu: Add diagnostic_aggregator dependency
* phidgets_imu: Add missing install rule for config
* update to use non deprecated pluginlib macro (`#19 <https://github.com/ros-drivers/phidgets_drivers/issues/19>`_)
* Contributors: Martin Günther, Mikael Arguedas

0.7.4 (2017-10-04)
------------------

0.7.3 (2017-06-30)
------------------

0.7.2 (2017-06-02)
------------------
* First release into Lunar
* phidgets_imu: Add use_magnetic_field_msg to launch
  This is required in Jade: Since Jade, phidgets_imu publishes
  MagneticField messages, but imu_filter_madgwick still subscribes by
  default to Vector3Stamped messages. When running as nodelets, this can
  produce a silent error.
  In Kinetic, this is optional: imu_filter_madgwick now defaults to
  MagneticField.
  From Lunar on, it should be removed, because the use_magnetic_field_msg
  param was removed from imu_filter_madgwick.
* Contributors: Martin Günther

0.7.1 (2017-05-22)
------------------
* phidgets_imu: add optional serial number parameter (`#7 <https://github.com/ros-drivers/phidgets_drivers/issues/7>`_)
* phidgets_imu: Add imu_filter_madgwick dependency
  Closes `#9 <https://github.com/ros-drivers/phidgets_drivers/issues/9>`_.
* Contributors: Johan M. von Behren, Martin Günther

0.7.0 (2017-02-17)
------------------
* Publish MagneticField instead of Vector3Stamped
* Report mag data in Tesla, not Gauss
  This is to conform with sensor_msgs/MagneticField, which requires the
  data to be in Tesla.
* Contributors: Martin Günther

0.2.3 (2017-02-17)
------------------
* Add IMU diagnostics (`#24 <https://github.com/ccny-ros-pkg/phidgets_drivers/pull/24>`_)
* Set data rate after reattachment
  This fixes a bug where after disconnecting and reconnecting the USB
  cable, the data rate would be set to the default of 125 Hz (= period of
  8ms). By moving the setDataRate call to the attachHandler, the data rate
  is correctly set after each reattachment.
* Contributors: Mani Monajjemi, Keshav Iyengar, Martin Günther

0.2.2 (2015-03-23)
------------------
* Merge pull request #18 from ccny-ros-pkg/libphidgets
  Merge libphidgets branch into indigo
* set orientation_covariance[0] to -1
  from Imu.msg:
  > If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation
  > estimate), please set element 0 of the associated covariance matrix to -1.
* phidgets_imu: fixed issue #9
* Contributors: Martin Günther, Murilo FM

0.2.1 (2015-01-15)
------------------
* add boost depends to CMakeLists
  All non-catkin things that we expose in our headers should be added to
  the DEPENDS, so that packages which depend on our package will also
  automatically link against it.
  Also see: http://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/\#58593
* improve error output when setting compass corr params
  The previous implementation didn't catch a number of error codes
  (EPHIDGET_INVALIDARG, EPHIDGET_NOTATTACHED, EPHIDGET_UNEXPECTED), and
  the new one is more elegant and consistent with the previous code anyway.
* Set compass correction params on the device
  Tested with a Phidget Spatial 3/3/3 1044.
* phidgets_imu: install phidgets_imu_nodelet.xml
* phidgets_imu: not exporting nodelet as library anymore
* Updated version, maintainer and author information
* phidgets_imu: added install rule to launch files
* phidgets_imu: removed unnecessary dependency
* Deleted comments within files of all packages
* Catkinised packages
* Merge pull request #1 from uos/fix_imu_time_lag
  fix IMU time lag
* add some hints to error message
  I just spent 30 minutes trying to figure out why the IMU works on one
  computer and doesn't on another one. Felt a little foolish when I found
  out that the udev rules weren't installed; maybe providing some more
  info in the error message helps others.
* use ros::Time::now() if time lag exceeds threshold
* added warning if IMU time lags behind ROS time
* renamed rate parameter to period
* added timestamp in imu data
* fixed cmakelists by including lib to compile on electric
* adding missing imu_ros h file
* adding missing imu_ros cpp file
* added api, imu and ir
* initial commit
* Contributors: Ivan Dryanovski, Martin Günther, Murilo FM
