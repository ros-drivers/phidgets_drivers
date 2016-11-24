cmake_minimum_required(VERSION 2.8.3)
project(phidgets_high_speed_encoder)

find_package(catkin REQUIRED COMPONENTS phidgets_api roscpp std_msgs message_generation)

add_message_files(
  FILES
  EncoderParams.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS phidgets_api roscpp std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(phidgets_high_speed_encoder src/phidgets_high_speed_encoder.cpp)
target_link_libraries(phidgets_high_speed_encoder ${catkin_LIBRARIES})
add_dependencies(phidgets_high_speed_encoder ${catkin_EXPORTED_TARGETS})
add_dependencies(phidgets_high_speed_encoder phidgets_high_speed_encoder_generate_messages)

install(TARGETS phidgets_high_speed_encoder
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

