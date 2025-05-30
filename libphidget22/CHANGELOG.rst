^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libphidget22
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-05-30)
------------------
* Merge pull request `#191 <https://github.com/ros-drivers/phidgets_drivers/issues/191>`_ from mintar/remove-ament-target-dependencies
  [kilted] Update deprecated call to ament_target_dependencies
* libphidget22: Fix CMake warning (DOWNLOAD_EXTRACT_TIMESTAMP)
* Contributors: Martin Günther

2.3.4 (2025-05-19)
------------------
* Upgrade to CMake 3.8, add file depend (`#189 <https://github.com/ros-drivers/phidgets_drivers/issues/189>`_)
* Contributors: Martin Günther

2.3.3 (2024-03-13)
------------------
* Update to libphidget22 1.19 (`#176 <https://github.com/ros-drivers/phidgets_drivers/issues/176>`_)
* Contributors: Martin Günther

2.3.2 (2023-11-27)
------------------

2.3.1 (2023-03-03)
------------------
* Update to libphidget22 1.13. (`#160 <https://github.com/ros-drivers/phidgets_drivers/issues/160>`_)
* Contributors: Chris Lalancette

2.3.0 (2022-04-13)
------------------
* Update to libphidgets 20220203 (`#138 <https://github.com/ros-drivers/phidgets_drivers/issues/138>`_)
* Contributors: Chris Lalancette

2.2.2 (2022-02-17)
------------------
* Remove outdated patch file (`#112 <https://github.com/ros-drivers/phidgets_drivers/issues/112>`_)
* Update to libphidget22-1.7.20210816 (`#108 <https://github.com/ros-drivers/phidgets_drivers/issues/108>`_)
  This is required to support new devices such as the MOT0109.
  Fixes `#99 <https://github.com/ros-drivers/phidgets_drivers/issues/99>`_, fixes `#105 <https://github.com/ros-drivers/phidgets_drivers/issues/105>`_.
  This is a forward-port of `#106 <https://github.com/ros-drivers/phidgets_drivers/issues/106>`_ to ROS2.
* Make sure libphidget22 library can be found. (`#97 <https://github.com/ros-drivers/phidgets_drivers/issues/97>`_) (`#100 <https://github.com/ros-drivers/phidgets_drivers/issues/100>`_)
  In Foxy and later, we need to provide the .dsv hook so that
  the library can be found.
* Contributors: Chris Lalancette, Martin Günther

2.2.1 (2021-08-03)
------------------
* Update to the latest libphidgets 1.6 library. (`#91 <https://github.com/ros-drivers/phidgets_drivers/issues/91>`_)
* Contributors: Chris Lalancette

2.2.0 (2021-05-20)
------------------

2.1.0 (2021-03-29)
------------------
* Compile libphidget22 with -fPIC (`#88 <https://github.com/ros-drivers/phidgets_drivers/issues/88>`_)
* Update to libphidget22 from 2020.
* Contributors: Chris Lalancette, Scott K Logan

2.0.2 (2020-06-01)
------------------

2.0.1 (2019-12-05)
------------------

2.0.0 (2019-12-05)
------------------
* Port libphidget22 vendor package to ament.
* Ignore all packages for ROS 2 port.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Patch warnings in libphidget22.
* Fix up libphidget22 package.xml dependencies.
* Add in libphidget22 package.
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
