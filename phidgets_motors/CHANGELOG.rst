^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package phidgets_motors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update maintainers in package.xml
* Switch to libphidget22
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
* Rewrite Motor Phidget to use libphidget22.
  Also implement a new motor node.
* Contributors: Chris Lalancette, Martin Günther
