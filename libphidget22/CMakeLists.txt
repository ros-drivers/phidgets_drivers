cmake_minimum_required(VERSION 3.5)
project(libphidget22)

find_package(ament_cmake REQUIRED)

set(extra_c_flags "-g -O2 -Wno-incompatible-pointer-types -Wno-deprecated-declarations -Wno-format-truncation -fPIC")

include(ExternalProject)
ExternalProject_Add(EP_${PROJECT_NAME}
    URL https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22/libphidget22-1.19.20240304.tar.gz
    URL_MD5 9b059eaef8cb8ce70b8abd7e4d309d1d

    SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
    CONFIGURE_COMMAND
      <SOURCE_DIR>/configure
      CFLAGS=${extra_c_flags}
      --prefix=${CMAKE_CURRENT_BINARY_DIR}/libphidget22_install
      --disable-ldconfig
    BUILD_COMMAND $(MAKE)
    INSTALL_COMMAND $(MAKE) install
)

# The external project will install to the build folder, but we'll install that on make install.
# Note that we install lib and include separately so we can add the extra level of indirection
# to the include directory.
install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/libphidget22_install/lib/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/opt/libphidget22/lib/
)

install(
  DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/libphidget22_install/include/
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/opt/libphidget22/include/libphidget22/
)

ament_environment_hooks(env_hook/libphidget22_library_path.sh)
set(ENV_VAR_NAME "LD_LIBRARY_PATH")
set(ENV_VAR_VALUE "opt/libphidget22/lib")
ament_environment_hooks(env_hook/libphidget22_library_path.dsv.in)

ament_package(CONFIG_EXTRAS "cmake/libphidget22-extras.cmake.in")
