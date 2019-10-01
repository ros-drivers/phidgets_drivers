Phidgets stepper ROS 2 driver
=============================

This is the ROS 2 driver for Phidgets stepper.  The various topics, services, and parameters that the node operates with are listed below.

Topics
------
* `/joint_states` (`sensor_msgs/JointState`) - A joint state message containing the current state of all steppers.

Services
--------
* `/set_enabledXX` (`std_srvs/SetBool`) - Set true to enable/engage and false to disable/disengage stepper.  Must be set true before use.  One service is created for each stepper attached.
* `/set_target_positionXX` (`phidgets_msgs/SetFloat64`) - Set the stepper target position when in position control mode.  One service is created for each stepper attached.
* `/set_velocity_limitXX` (`phidgets_msgs/SetFloat64`) - Set the stepper velocity limit.  Must be set before use.  One service is created for each stepper attached.
* `/set_accelerationXX` (`phidgets_msgs/SetFloat64`) - Set the stepper acceleration.  One service is created for each stepper attached.
* `/set_current_limitXX` (`phidgets_msgs/SetFloat64`) - Set the stepper current limit.  Must be set before use.  One service is created for each stepper attached.
* `/set_holding_current_limitXX` (`phidgets_msgs/SetFloat64`) - Set the stepper holding current limit.  One service is created for each stepper attached.
* `/set_positionXX` (`phidgets_msgs/SetFloat64`) - Set the current position and target position a new value.  For example, setting the position to zero after homing.  One service is created for each stepper attached.
* `/get_settingsXX` (`phidgets_msgs/GetStepperSettings`) - Get the values of the stepper settings. One service is created for each stepper attached.
* `/get_setting_rangesXX` (`phidgets_msgs/GetStepperSettingRanges`) - Get the values of the stepper setting ranges. One service is created for each stepper attached.

Parameters
----------
* `serial` (int) - The serial number of the phidgets stepper to connect to.  If -1 (the default), connects to any stepper phidget that can be found.
* `hub_port` (int) - The phidgets VINT hub port to connect to.  Only used if the stepper phidget is connected to a VINT hub.  Defaults to 0.
* `data_interval_ms` (int) - The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device).  Defaults to 250 ms.
* `publish_rate` (double) - How often the driver will publish data on the ROS topic.  If 0 (the default), it will publish every time there is an update from the device (so at the `data_interval_ms`).  If positive, it will publish the data at that rate regardless of the acquisition interval.
* `watchdog_interval_ms` (int) - The number of milliseconds allowed to elapse between calls to a watchdog timer.  Used as a failsafe to help prevent hardware damage in case of program crash. Defaults to 1000 ms with a range between 500-10000 ms.
* `frame_id` (string) - The header frame ID to use when publishing the message.  Defaults to `stepper`.
* `joint_nameXX` (string) - Name of each joint published in joint_states. Defaults to `joint_nameXX`.
* `rescale_factorXX` (double) - Applies a factor to the [user units] per step to all movement parameters to make the units in your application more intuitive. Setting a new target position with a rescale factor, the stepper will move ((Target Position - Current Position) / RescaleFactor) steps. Defaults to 1.0.
* `position_control_modeXX` (bool) - Set true for position control mode (step mode) or false for velocity control mode (run mode). Defaults to true.

Command Line Examples
---------------------

```bash
ros2 launch phidgets_steppers steppers-launch.py
ros2 topic echo /joint_states
ros2 service call /get_setting_ranges00 phidgets_msgs/GetStepperSettingRanges
ros2 service call /set_current_limit00 phidgets_msgs/SetFloat64 "data: 1.0"
ros2 service call /set_holding_current_limit00 phidgets_msgs/SetFloat64 "data: 0.5"
ros2 service call /get_settings00 phidgets_msgs/GetStepperSettings
ros2 service call /set_enabled00 std_srvs/SetBool "data: true"
ros2 service call /set_velocity_limit00 phidgets_msgs/SetFloat64 "data: 5000"
ros2 service call /set_target_position00 phidgets_msgs/SetFloat64 "data: 10000"
ros2 service call /set_position00 phidgets_msgs/SetFloat64 "data: 0"
```
