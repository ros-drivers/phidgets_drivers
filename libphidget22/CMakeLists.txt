cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(libphidget22)

find_package(catkin REQUIRED)

catkin_destinations() # set-up destination variables
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(extra_c_flags "-g -O2 -Wno-incompatible-pointer-types -Wno-deprecated-declarations -Wno-format-truncation")

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
    URL https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22/libphidget22-1.19.20240304.tar.gz
    URL_MD5 9b059eaef8cb8ce70b8abd7e4d309d1d

    CONFIGURE_COMMAND "./configure"
    SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
    BUILD_IN_SOURCE 1
    BUILD_COMMAND make "CFLAGS=-g -O2 -Wno-incompatible-pointer-types -Wno-deprecated-declarations -Wno-format-truncation"
        # copy headers to devel space (catkin does not like headers in source space)
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/phidget22.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        # copy libs, set-up soname chain
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/.libs/libphidget22.so.0.0.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
        COMMAND ${CMAKE_COMMAND} -E create_symlink libphidget22.so.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libphidget22.so
        COMMAND ${CMAKE_COMMAND} -E create_symlink libphidget22.so.0.0.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libphidget22.so.0
    INSTALL_COMMAND ""
)

catkin_package(
    CFG_EXTRAS libphidget22-extras.cmake # catkin_package does not support artifacts in devel space
    EXPORTED_TARGETS EP_${PROJECT_NAME}
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libphidget22.so
              ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libphidget22.so.0
              ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libphidget22.so.0.0.0
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
