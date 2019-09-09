cmake_minimum_required(VERSION 3.5)
project(phidgets_magnetometer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(phidgets_magnetometer SHARED src/magnetometer_ros_i.cpp)
ament_target_dependencies(phidgets_magnetometer
  phidgets_api
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(phidgets_magnetometer
  "phidgets::MagnetometerRosI")

install(TARGETS phidgets_magnetometer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
