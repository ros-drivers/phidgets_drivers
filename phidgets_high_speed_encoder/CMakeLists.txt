cmake_minimum_required(VERSION 3.8)
project(phidgets_high_speed_encoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(phidgets_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(phidgets_high_speed_encoder SHARED src/high_speed_encoder_ros_i.cpp)
target_compile_features(phidgets_high_speed_encoder PUBLIC cxx_std_17)
target_link_libraries(phidgets_high_speed_encoder PUBLIC
  ${phidgets_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  phidgets_api::phidgets_api
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

rclcpp_components_register_nodes(phidgets_high_speed_encoder
  "phidgets::HighSpeedEncoderRosI")

install(TARGETS phidgets_high_speed_encoder
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
