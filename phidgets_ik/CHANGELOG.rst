^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2025-05-30)
------------------

2.3.4 (2025-05-19)
------------------
* Upgrade to CMake 3.8, add file depend (`#189 <https://github.com/ros-drivers/phidgets_drivers/issues/189>`_)
* Contributors: Martin Günther

2.3.3 (2024-03-13)
------------------

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
* Get rid of deprecation warnings in Foxy. (`#75 <https://github.com/ros-drivers/phidgets_drivers/issues/75>`_)
  Foxy deprecated some of the names we use in the launch files.
  Switch to the new supported names here.
* Contributors: Chris Lalancette

2.0.2 (2020-06-01)
------------------

2.0.1 (2019-12-05)
------------------

2.0.0 (2019-12-05)
------------------
* Update READMEs to use "Published Topics" and "Subscribed Topics". (`#59 <https://github.com/ros-drivers/phidgets_drivers/issues/59>`_)
* Add in parameters to the IK launch file.
* Change launch output to "both" so it logs as well.
* Port IK to ROS 2.
* Ignore all packages for ROS 2 port.
* Update maintainers in package.xml
* Merge pull request `#39 <https://github.com/ros-drivers/phidgets_drivers/issues/39>`_ from clalancette/add-libphidget22
* Fix phidgets_ik dependencies.
* Rewrite IK as just a launch file.
* Add launch files for all drivers.
* Add in try/catch blocks for connecting.
* Fixes from review.
* Documentation updates to README.md
* Set the publish_rate to 0 by default.
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Rewrite phidgets_ik on top of libphidget22 classes.
* Rename Phidget class to Phidget21 class.
* Switch to C++14.
* Merge pull request `#36 <https://github.com/ros-drivers/phidgets_drivers/issues/36>`_ from clalancette/phidget-cleanup2
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Split custom messages into their own package.
* Add in a nodelet version of the interfaceKit.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
* Small cleanups throughout the code.
* Push libphidgets API calls down to phidgets_api.
* Completely remove boost from the project.
* Style cleanup.
* Remove unused dependencies from phidgets_ik.
* Switch to package format 2.
* Cleanup spacing in all of the CMakeLists.txt
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
* Initial release of the phidgets_ik package for the Phidgets Interface Kit
* Contributors: Russel Howe, James Sarrett, Dorian Goepp, Martin Günther

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
