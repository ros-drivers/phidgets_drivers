cmake_minimum_required(VERSION 3.8)
project(phidgets_temperature)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(phidgets_temperature SHARED src/temperature_ros_i.cpp)
target_compile_features(phidgets_temperature PUBLIC cxx_std_17)
target_link_libraries(phidgets_temperature PUBLIC
  ${std_msgs_TARGETS}
  phidgets_api::phidgets_api
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
)

rclcpp_components_register_nodes(phidgets_temperature
  "phidgets::TemperatureRosI")

install(TARGETS phidgets_temperature
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
