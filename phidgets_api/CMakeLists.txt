cmake_minimum_required(VERSION 3.8)

project(phidgets_api)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(libphidget22 REQUIRED)

add_library(phidgets_api SHARED
                         src/accelerometer.cpp
                         src/analog_input.cpp
                         src/analog_inputs.cpp
                         src/analog_output.cpp
                         src/analog_outputs.cpp
                         src/digital_input.cpp
                         src/digital_inputs.cpp
                         src/digital_output.cpp
                         src/digital_outputs.cpp
                         src/encoder.cpp
                         src/encoders.cpp
                         src/gyroscope.cpp
                         src/ir.cpp
                         src/magnetometer.cpp
                         src/motor.cpp
                         src/motors.cpp
                         src/phidget22.cpp
                         src/spatial.cpp
                         src/stepper.cpp
                         src/temperature.cpp)

target_compile_features(phidgets_api PUBLIC cxx_std_17)

target_link_libraries(phidgets_api PUBLIC
    ${libphidget22_LIBRARIES}
)

target_include_directories(phidgets_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${libphidget22_INCLUDE_DIRS}
)

install(
  TARGETS phidgets_api
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  ament_cmake
  libphidget22
)
ament_export_include_directories(include)
ament_export_libraries(phidgets_api)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
