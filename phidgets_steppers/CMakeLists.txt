cmake_minimum_required(VERSION 3.5)
project(phidgets_steppers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(phidgets_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(phidgets_steppers SHARED src/steppers_ros_i.cpp)
ament_target_dependencies(phidgets_steppers
  phidgets_api
  phidgets_msgs
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
)

rclcpp_components_register_nodes(phidgets_steppers
  "phidgets::SteppersRosI")

install(TARGETS phidgets_steppers
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
