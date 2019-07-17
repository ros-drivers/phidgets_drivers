cmake_minimum_required(VERSION 2.8.3)
project(phidgets_digital_outputs)

# Use C++14
if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_CXX_FLAGS "-std=gnu++14 ${CMAKE_CXX_FLAGS}")
    endif()
else()
    set (CMAKE_CXX_STANDARD 14)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS nodelet phidgets_api phidgets_msgs roscpp std_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_digital_outputs
    CATKIN_DEPENDS nodelet phidgets_api phidgets_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_digital_outputs src/digital_outputs_ros_i.cpp)
add_library(phidgets_digital_outputs_nodelet src/phidgets_digital_outputs_nodelet.cpp)

add_executable(phidgets_digital_outputs_node src/phidgets_digital_outputs_node.cpp)

add_dependencies(phidgets_digital_outputs ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_digital_outputs_nodelet ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_digital_outputs_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_digital_outputs ${catkin_LIBRARIES})
target_link_libraries(phidgets_digital_outputs_nodelet ${catkin_LIBRARIES} phidgets_digital_outputs)
target_link_libraries(phidgets_digital_outputs_node ${catkin_LIBRARIES} phidgets_digital_outputs)

install(TARGETS phidgets_digital_outputs phidgets_digital_outputs_nodelet phidgets_digital_outputs_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(FILES phidgets_digital_outputs_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)