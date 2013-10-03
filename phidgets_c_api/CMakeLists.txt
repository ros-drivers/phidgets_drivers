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
--libdir=${CATKIN_DEVEL_PREFIX}/lib
--includedir=${CMAKE_CURRENT_SOURCE_DIR}/include/phidgets_c_api
  BUILD_COMMAND make -j${PROCESSOR_COUNT}
  INSTALL_COMMAND make install
)

ExternalProject_Get_Property(libphidget21 install_dir)

project(phidgets_c_api)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidget21
)

include_directories(${install_dir}/include)

add_library(phidget21 UNKNOWN IMPORTED)
set_target_properties(phidget21 PROPERTIES IMPORTED_LOCATION "${install_dir}/lib/libphidget21.so")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

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
