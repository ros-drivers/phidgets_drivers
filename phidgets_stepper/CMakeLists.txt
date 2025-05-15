cmake_minimum_required(VERSION 3.8)
project(phidgets_stepper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -ggdb)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(phidgets_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(phidgets_stepper SHARED src/stepper_ros_i.cpp)
ament_target_dependencies(phidgets_stepper
  phidgets_api
  phidgets_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
)

rclcpp_components_register_nodes(phidgets_stepper
    "phidgets::StepperRosI")

install(TARGETS phidgets_stepper
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
