cmake_minimum_required(VERSION 2.8.3)
project(phidgets_high_speed_encoder)

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

find_package(catkin REQUIRED COMPONENTS nodelet phidgets_api phidgets_msgs pluginlib roscpp std_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS nodelet phidgets_api phidgets_msgs pluginlib roscpp std_msgs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_high_speed_encoder src/high_speed_encoder_ros_i.cpp)
add_library(phidgets_high_speed_encoder_nodelet src/phidgets_high_speed_encoder_nodelet.cpp)

add_executable(phidgets_high_speed_encoder_node src/phidgets_high_speed_encoder_node.cpp)

add_dependencies(phidgets_high_speed_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_high_speed_encoder_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_high_speed_encoder_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_high_speed_encoder ${catkin_LIBRARIES})
target_link_libraries(phidgets_high_speed_encoder_nodelet ${catkin_LIBRARIES} phidgets_high_speed_encoder)
target_link_libraries(phidgets_high_speed_encoder_node ${catkin_LIBRARIES} phidgets_high_speed_encoder)

install(TARGETS phidgets_high_speed_encoder phidgets_high_speed_encoder_nodelet phidgets_high_speed_encoder_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES phidgets_high_speed_encoder_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
