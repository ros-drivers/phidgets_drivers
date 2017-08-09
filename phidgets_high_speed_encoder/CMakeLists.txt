cmake_minimum_required(VERSION 2.8.3)
project(phidgets_high_speed_encoder)

# Use C++11
if (CMAKE_VERSION VERSION_LESS "3.1")
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
	endif ()
else ()
	set (CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS libphidget21 roscpp std_msgs message_generation)

add_message_files(
  FILES
  EncoderParams.msg
  EncoderDecimatedSpeed.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS libphidget21 roscpp std_msgs message_runtime
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
