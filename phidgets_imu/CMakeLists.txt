cmake_minimum_required(VERSION 2.8.3)
project(phidgets_imu)

# Use C++11
if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif()
else()
    set (CMAKE_CXX_STANDARD 11)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs diagnostic_updater nodelet pluginlib phidgets_api roscpp roslaunch sensor_msgs std_msgs std_srvs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_imu
    CATKIN_DEPENDS diagnostic_msgs diagnostic_updater nodelet pluginlib phidgets_api roscpp sensor_msgs std_msgs std_srvs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_imu src/imu_ros_i.cpp)
add_library(phidgets_imu_nodelet src/phidgets_imu_nodelet.cpp)

add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)

add_dependencies(phidgets_imu ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_imu_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_imu_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_imu ${catkin_LIBRARIES})
target_link_libraries(phidgets_imu_nodelet ${catkin_LIBRARIES} phidgets_imu)
target_link_libraries(phidgets_imu_node ${catkin_LIBRARIES} phidgets_imu)

install(TARGETS phidgets_imu phidgets_imu_nodelet phidgets_imu_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES phidgets_imu_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch)
