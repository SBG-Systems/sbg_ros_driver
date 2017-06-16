cmake_minimum_required(VERSION 2.8.3)
project(sbg_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/sbgECom/common
  ${PROJECT_SOURCE_DIR}/sbgECom/src
)

## Declare a C++ executable
add_executable(ellipse
        src/ellipse.cpp
        src/SbgDriver.cpp
        ${SRC}
        ${COMMON_SRC}
)
#target_compile_options(ellipse PRIVATE -fpermissive)

## Specify libraries to link a library or executable target against
add_subdirectory(sbgECom)
target_link_libraries(ellipse
   ${catkin_LIBRARIES}
   sbgECom
)

