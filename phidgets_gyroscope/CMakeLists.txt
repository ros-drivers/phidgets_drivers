cmake_minimum_required(VERSION 3.5)
project(phidgets_gyroscope)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(phidgets_gyroscope SHARED src/gyroscope_ros_i.cpp)
ament_target_dependencies(phidgets_gyroscope
  phidgets_api
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
)

rclcpp_components_register_nodes(phidgets_gyroscope
  "phidgets::GyroscopeRosI")

install(TARGETS phidgets_gyroscope
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
