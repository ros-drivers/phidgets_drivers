cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(phidgets_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# High level of warnings:
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
    DIRECTORY msg
    FILES
    EncoderDecimatedSpeed.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    SetAnalogOutput.srv
    SetDigitalOutput.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)
