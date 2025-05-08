^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fix phidgets_ik dependencies.
* Rewrite IK as just a launch file.
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
* Rewrite phidgets_ik on top of libphidget22 classes.
* Rename Phidget class to Phidget21 class.
  Also rename the files, etc.
* Run clang-format on the whole codebase.
* Switch to C++14 everywhere.
* Split custom messages into their own package.
* Add in a nodelet version of the interfaceKit.
* Change API from separate open/waitForAttachment to openAndWaitForAttachment.
  None of the current users need it to be split, and this will make
  it easier to port to libphidgets22.
* Small cleanups throughout the code.
* Push libphidgets API calls down to phidgets_api.
* Completely remove boost from the project.
* Style cleanup.
* Remove unused dependencies from phidgets_ik.
* Switch to package format 2.
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
