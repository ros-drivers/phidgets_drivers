cmake_minimum_required(VERSION 2.8.3)

project(phidgets_api)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS phidgets_c_api)

#NOTE: Hack to add external project by hand to this project
#set(phidgets_c_api_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so)
#set(phidgets_c_api_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
#set(phidgets_c_api_DEPENDENCIES phidgets_src)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_api
  CATKIN_DEPENDS phidgets_c_api
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#include_directories(include ${catkin_INCLUDE_DIRS} ${phidget_c_api_INCLUDE_DIRS} )
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(phidgets_api src/phidget.cpp
                         src/imu.cpp
                         src/ir.cpp)

#add_library(phidget21 UNKNOWN IMPORTED)
#set_target_properties(phidget21 PROPERTIES IMPORTED_LOCATION "${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so")

## Declare a cpp executable
# add_executable(phidgets_api_node src/phidgets_api_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(phidgets_api_node phidgets_api_generate_messages_cpp)
#add_dependencies(phidgets_api ${catkin_EXPORTED_TARGETS} ${phidgets_c_api_DEPENDENCIES})
#add_dependencies(phidgets_api phidget21)
add_dependencies(phidgets_api phidgets_c_api)

## Specify libraries to link a library or executable target against
#target_link_libraries(phidgets_api ${catkin_LIBRARIES} ${phidgets_c_api_LIBRARIES})
#target_link_libraries(phidgets_api phidget21)
target_link_libraries(phidgets_api ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS phidgets_api
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phidgets_api.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
