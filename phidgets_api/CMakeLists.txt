cmake_minimum_required(VERSION 2.8.3)

project(phidgets_api)

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

find_package(catkin REQUIRED COMPONENTS libphidget21)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_api
    CATKIN_DEPENDS libphidget21
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_api src/encoder.cpp
                         src/imu.cpp
                         src/ik.cpp
                         src/ir.cpp
                         src/motor.cpp
                         src/phidget.cpp)

add_dependencies(phidgets_api ${catkin_EXPORTED_TARGETS})

target_link_libraries(phidgets_api ${catkin_LIBRARIES})

install(TARGETS phidgets_api
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
