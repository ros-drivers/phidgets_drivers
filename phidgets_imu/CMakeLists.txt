cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# create phidgets_imu library
rosbuild_add_library (phidgets_imu src/imu_ros_i.cpp)
target_link_libraries (phidgets_imu phidgets_api)

# create phidgets_imu_nodelet library
rosbuild_add_library (phidgets_imu_nodelet src/phidgets_imu_nodelet.cpp)
target_link_libraries (phidgets_imu_nodelet phidgets_imu)

# create phidgets_imu_node executable
rosbuild_add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)
target_link_libraries (phidgets_imu_node phidgets_imu)
