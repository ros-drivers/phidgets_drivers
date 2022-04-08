cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(phidgets_magnetometer)

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
    LIBRARIES phidgets_magnetometer
    CATKIN_DEPENDS nodelet phidgets_api roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_magnetometer src/magnetometer_ros_i.cpp)
add_library(phidgets_magnetometer_nodelet src/phidgets_magnetometer_nodelet.cpp)

add_dependencies(phidgets_magnetometer ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_magnetometer_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_magnetometer ${catkin_LIBRARIES})
target_link_libraries(phidgets_magnetometer_nodelet ${catkin_LIBRARIES} phidgets_magnetometer)

install(TARGETS phidgets_magnetometer phidgets_magnetometer_nodelet
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

install(FILES phidgets_magnetometer_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
