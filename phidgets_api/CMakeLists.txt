cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(phidgets_api)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS libphidget22)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES phidgets_api
    CATKIN_DEPENDS libphidget22
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_api src/accelerometer.cpp
                         src/analog_input.cpp
                         src/analog_inputs.cpp
                         src/analog_output.cpp
                         src/analog_outputs.cpp
                         src/current_input.cpp
                         src/current_inputs.cpp
                         src/digital_input.cpp
                         src/digital_inputs.cpp
                         src/digital_output.cpp
                         src/digital_outputs.cpp
                         src/encoder.cpp
                         src/encoders.cpp
                         src/gyroscope.cpp
                         src/humidity.cpp
                         src/ir.cpp
                         src/magnetometer.cpp
                         src/motor.cpp
                         src/motors.cpp
                         src/phidget22.cpp
                         src/spatial.cpp
                         src/temperature.cpp)

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
