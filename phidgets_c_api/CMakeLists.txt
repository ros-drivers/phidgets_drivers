cmake_minimum_required(VERSION 2.8.3)

## From http://www.kitware.com/blog/home/post/63
if(NOT DEFINED PROCESSOR_COUNT)
  # Unknown:
  set(PROCESSOR_COUNT 0)
 
  # Linux:
  set(cpuinfo_file "/proc/cpuinfo")
  if(EXISTS "${cpuinfo_file}")
    file(STRINGS "${cpuinfo_file}" procs REGEX "^processor.: [0-9]+$")
    list(LENGTH procs PROCESSOR_COUNT)
  endif()
 
  # Mac:
  if(APPLE)
    find_program(cmd_sys_pro "system_profiler")
    if(cmd_sys_pro)
      execute_process(COMMAND ${cmd_sys_pro} OUTPUT_VARIABLE info)
      string(REGEX REPLACE "^.*Total Number Of Cores: ([0-9]+).*$" "\\1"
        PROCESSOR_COUNT "${info}")
    endif()
  endif()
endif()

include(ExternalProject)
ExternalProject_Add(libphidget21
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
  URL http://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20130618.tar.gz
  URL_MD5 97c7834e4eb8ae04dd177ab3f4829472
  CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/src/libphidget21/configure CFLAGS=-O3 --disable-jni
--disable-ldconfig 
--prefix=${CMAKE_CURRENT_BINARY_DIR} --exec-prefix=${CMAKE_CURRENT_BINARY_DIR}
#--prefix=${CATKIN_DEVEL_PREFIX} --exec-prefix=${CATKIN_DEVEL_PREFIX}
--libdir=${CATKIN_DEVEL_PREFIX}/lib
#--includedir=${CATKIN_DEVEL_PREFIX}/include/phidgets_c_api
--includedir=${CMAKE_CURRENT_SOURCE_DIR}/include/phidgets_c_api
  BUILD_COMMAND make -j${PROCESSOR_COUNT}
  INSTALL_COMMAND make install
)

ExternalProject_Get_Property(libphidget21 install_dir)

#MESSAGE( STATUS "INSTALL_DIR: " ${install_dir} )

project(phidgets_c_api)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidget21
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${install_dir}/include)

## Declare a cpp library
# add_library(phidgets_c_api
#   src/${PROJECT_NAME}/phidgets_c_api.cpp
# )
add_library(phidget21 UNKNOWN IMPORTED)
set_target_properties(phidget21 PROPERTIES IMPORTED_LOCATION "${install_dir}/lib/libphidget21.so")


## Declare a cpp executable
# add_executable(phidgets_c_api_node src/phidgets_c_api_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(phidgets_c_api_node phidgets_c_api_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(phidgets_c_api_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS phidget21
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark header files for installation
#install(DIRECTORY ${install_dir}/include/
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.a
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so.0
   ${CATKIN_DEVEL_PREFIX}/lib/libphidget21.so.0.0.0
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
   ${install_dir}/src/libphidget21/udev/99-phidgets.rules
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phidgets_c_api.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
