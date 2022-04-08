cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(phidgets_accelerometer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS nodelet phidgets_api roscpp roslaunch sensor_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_accelerometer
    CATKIN_DEPENDS nodelet phidgets_api roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_accelerometer src/accelerometer_ros_i.cpp)
add_library(phidgets_accelerometer_nodelet src/phidgets_accelerometer_nodelet.cpp)

add_dependencies(phidgets_accelerometer ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_accelerometer_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_accelerometer ${catkin_LIBRARIES})
target_link_libraries(phidgets_accelerometer_nodelet ${catkin_LIBRARIES} phidgets_accelerometer)

install(TARGETS phidgets_accelerometer phidgets_accelerometer_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES phidgets_accelerometer_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
