^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_digital_outputs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Add missing license headers
* Contributors: Martin Günther

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
* Add in the license files and add to the headers.
* Remove nodes in favor of nodelets.
* Add in support for Phidgets Digital Outputs.
* Contributors: Chris Lalancette, Martin Günther
