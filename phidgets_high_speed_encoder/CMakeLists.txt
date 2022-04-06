cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(phidgets_high_speed_encoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS nodelet phidgets_api phidgets_msgs pluginlib roscpp roslaunch sensor_msgs)

catkin_package(
    CATKIN_DEPENDS nodelet phidgets_api phidgets_msgs pluginlib roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_high_speed_encoder src/high_speed_encoder_ros_i.cpp)
add_library(phidgets_high_speed_encoder_nodelet src/phidgets_high_speed_encoder_nodelet.cpp)

add_dependencies(phidgets_high_speed_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_high_speed_encoder_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_high_speed_encoder ${catkin_LIBRARIES})
target_link_libraries(phidgets_high_speed_encoder_nodelet ${catkin_LIBRARIES} phidgets_high_speed_encoder)

install(TARGETS phidgets_high_speed_encoder phidgets_high_speed_encoder_nodelet
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

install(FILES phidgets_high_speed_encoder_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
