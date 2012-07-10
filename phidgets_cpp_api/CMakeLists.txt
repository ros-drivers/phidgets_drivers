cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(phidgetspp)

rospack_add_executable(rfid_demo src/rfid_demo.cc)
rospack_add_executable(lcd_demo src/lcd_demo.cc)

rospack_add_gtest(test_servo test/servo_unittest.cpp)
