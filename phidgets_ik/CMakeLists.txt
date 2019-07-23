cmake_minimum_required(VERSION 2.8.3)
project(phidgets_ik)

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

find_package(catkin REQUIRED COMPONENTS nodelet phidgets_api phidgets_msgs pluginlib roscpp sensor_msgs std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_ik
    CATKIN_DEPENDS nodelet phidgets_api phidgets_msgs pluginlib roscpp sensor_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_ik src/ik_ros_i.cpp)
add_library(phidgets_ik_nodelet src/phidgets_ik_nodelet.cpp)

add_executable(phidgets_ik_node src/phidgets_ik_node.cpp)

add_dependencies(phidgets_ik ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_ik_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_ik_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_ik ${catkin_LIBRARIES})
target_link_libraries(phidgets_ik_nodelet ${catkin_LIBRARIES} phidgets_ik)
target_link_libraries(phidgets_ik_node ${catkin_LIBRARIES} phidgets_ik)

install(TARGETS phidgets_ik phidgets_ik_nodelet phidgets_ik_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES phidgets_ik_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
