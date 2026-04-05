cmake_minimum_required(VERSION 2.8.3)
project(relative_odom_tf)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  nav_msgs
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(relativeOdometry_zyf src/relativeOdometry_zyf.cpp)
add_executable(relativeOdometry_zyf_2 src/relativeOdometry_zyf_2.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(relativeOdometry_zyf ${catkin_LIBRARIES})
target_link_libraries(relativeOdometry_zyf_2 ${catkin_LIBRARIES})

install(TARGETS relativeOdometry_zyf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
