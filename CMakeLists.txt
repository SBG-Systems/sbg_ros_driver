cmake_minimum_required(VERSION 2.8.3)
project(sbg_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
  cmake_modules
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
  SbgGpsVelStatus.msg
  SbgGpsPosStatus.msg
  SbgMagStatus.msg
  SbgShipMotionStatus.msg
  SbgUtcTimeStatus.msg

  SbgAirData.msg
  SbgAirDataStatus.msg
  SbgEkfEuler.msg
  SbgGpsPos.msg
  SbgMag.msg
  SbgStatusCom.msg
  SbgEkfNav.msg
  SbgGpsRaw.msg
  SbgOdoVel.msg
  SbgStatusGeneral.msg
  SbgEkfQuat.msg
  SbgGpsVel.msg
  SbgStatus.msg
  SbgEvent.msg
  SbgImuStatus.msg
  SbgImuData.msg
  SbgImuShort.msg
  SbgShipMotion.msg
  SbgUtcTime.msg
  SbgGpsHdt.msg
  SbgMagCalib.msg
  SbgStatusAiding.msg
  SbgEkfStatus.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS src
  # LIBRARIES ${PROJECT_NAME} 
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  # DEPENDS sbgECom
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_subdirectory(external/sbgECom)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/sbg_driver/
  ${PROJECT_SOURCE_DIR}/external/sbgECom/common
  ${PROJECT_SOURCE_DIR}/external/sbgECom/src
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(sbg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Define common resources.
set (SBG_COMMON_RESOURCES
  src/config_applier.cpp
  src/message_publisher.cpp
  src/message_wrapper.cpp
  src/config_store.cpp
  src/sbg_device.cpp
)

## Declare a C++ executable
add_executable(sbg_device ${SBG_COMMON_RESOURCES} src/main.cpp)
add_dependencies(sbg_device ${sbg_driver_EXPORTED_TARGETS})
target_compile_options(sbg_device PRIVATE -fpermissive)

add_executable(sbg_device_mag ${SBG_COMMON_RESOURCES} src/main_mag.cpp)
add_dependencies(sbg_device_mag ${sbg_driver_EXPORTED_TARGETS})
target_compile_options(sbg_device_mag PRIVATE -fpermissive)

## Specify libraries to link a library or executable target against
target_link_libraries(sbg_device ${catkin_LIBRARIES} sbgECom)
target_link_libraries(sbg_device_mag ${catkin_LIBRARIES} sbgECom)

set_property(TARGET sbg_device PROPERTY CXX_STANDARD 11)
set_property(TARGET sbg_device_mag PROPERTY CXX_STANDARD 11)

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
install(TARGETS sbg_device sbg_device_mag
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".git" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".git" EXCLUDE)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sbg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)