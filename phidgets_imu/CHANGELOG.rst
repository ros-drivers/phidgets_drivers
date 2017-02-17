^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_imu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
