cmake_minimum_required(VERSION 3.5)

project(phidgets_api)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(libphidget22 REQUIRED)

include_directories(include)

add_library(phidgets_api src/motor.cpp
                         src/motors.cpp
                         src/phidget22.cpp)

ament_target_dependencies(phidgets_api
  libphidget22
)

install(TARGETS phidgets_api
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_dependencies(ament_cmake libphidget22)
ament_export_include_directories(include)
ament_export_libraries(phidgets_api)

ament_package()
