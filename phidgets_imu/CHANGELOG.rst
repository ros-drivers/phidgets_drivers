^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_imu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.11 (2021-04-09)
-------------------

0.7.10 (2020-06-03)
-------------------
* Update maintainers in package.xml
* Fix wrong defaults for standard deviations (`#46 <https://github.com/ros-drivers/phidgets_drivers/issues/46>`_)
  The old parameter defaults were wrong:
  | parameter                 | old default                       | new default                        |
  |                           |                                   |                                    |
  | angular_velocity_stdev    | 0.000349056 rad/s (= 0.02 deg/s)  | 0.001658 rad/s    (= 0.095deg/s)   |
  | linear_acceleration_stdev | 0.002943 m/s^2 (= 0.0003 g)       | 0.002745862 m/s^2 (= 0.00028 g)    |
  | magnetic_field_stdev      | 0.001658 rad/s (= 0.095deg/s)     | 1.1e-7 T         (= 1.1 mG)        |
  Notes: T = Tesla, mG = milligauss
  Specifications come from the PhidgetSpatial Precision 3/3/3 1044_0 data sheet: https://www.phidgets.com/?&prodid=32
* Improve the IMU calibration service (`#41 <https://github.com/ros-drivers/phidgets_drivers/issues/41>`_)
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Small cleanups throughout the code.
* Push libphidgets API calls down to phidgets_api.
* IMU: small fixes found by turning on compiler warnings.
* Completely remove boost from the project.
* Remove unused tf dependency from phidgets_imu.
* Switch to package format 2.
* Cleanup spacing in all of the CMakeLists.txt
* Contributors: Chris Lalancette, Martin Günther, Michael Grupp

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
