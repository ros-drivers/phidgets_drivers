cmake_minimum_required(VERSION 2.8.3)

project(phidgets_api)

find_package(catkin REQUIRED COMPONENTS libphidgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_api
  CATKIN_DEPENDS libphidgets
#  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_api src/phidget.cpp
                         src/imu.cpp
                         src/ir.cpp)

target_link_libraries(phidgets_api ${catkin_LIBRARIES})

install(TARGETS phidgets_api
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY share/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
