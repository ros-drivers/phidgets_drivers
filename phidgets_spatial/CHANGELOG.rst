^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_spatial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2024-03-13)
------------------
* Add support for VINT networkhub (`#172 <https://github.com/ros-drivers/phidgets_drivers/issues/172>`_)
  This is a port of `#127 <https://github.com/ros-drivers/phidgets_drivers/issues/127>`_ to ROS2.
  Closes `#135 <https://github.com/ros-drivers/phidgets_drivers/issues/135>`_.
* Contributors: Martin Günther

2.3.2 (2023-11-27)
------------------
* Only set magnetometer gain if param is set (`#169 <https://github.com/ros-drivers/phidgets_drivers/issues/169>`_)
* added new parameters for spatial precision MOT0109 onwards
* added support for phidget spatial onboard orientation estimation
* Contributors: Malte kl. Piening, Martin Günther

2.3.1 (2023-03-03)
------------------

2.3.0 (2022-04-13)
------------------

2.2.2 (2022-02-17)
------------------
* Fix behavior after USB reattachment (`#119 <https://github.com/ros-drivers/phidgets_drivers/issues/119>`_)
  The Phidged Spatial never recovered after detaching and reattaching to
  the USB port. This commit fixes that.
* Add attach + detach handlers
* Fix publishing of invalid mag readings (`#116 <https://github.com/ros-drivers/phidgets_drivers/issues/116>`_)
* Contributors: Martin Günther

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
* Include file cleanup.
* Make sure exceptions get caught by reference.
* Switch from NULL to nullptr.
* Make sure to initialize class member variables.
* Update READMEs to use "Published Topics" and "Subscribed Topics". (`#59 <https://github.com/ros-drivers/phidgets_drivers/issues/59>`_)
* Change launch output to "both" so it logs as well.
* Make publish_rate a double.
* Print out the serial number when connecting.
* Update documentation to mention device dependent fields.
* Update FIXME comments for sleep.
* Port spatial to ROS 2.
* Ignore all packages for ROS 2 port.
* Fix wrong defaults for standard deviations (`#48 <https://github.com/ros-drivers/phidgets_drivers/issues/48>`_)
* Improve the IMU calibration service (`#47 <https://github.com/ros-drivers/phidgets_drivers/issues/47>`_)
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
