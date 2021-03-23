cmake_minimum_required(VERSION 2.8.3)
project(tunecode)

## Set the package dependencies: Find catkin macros/libraries and
## other required packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation)

#find_package(Eigen3 REQUIRED)
#find_package(OpenCV)


########################################
## ROS messages, services and actions ##
########################################

## Generate messages in the 'msg' folder
# add_message_files(FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
add_service_files(FILES
  MoveTo.srv
  TuneCfg.srv
)

## Generate actions in the 'action' folder
# add_action_files(FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
#  sensor_msgs
#  std_msgs
)


###################################
## catkin specific configuration ##
###################################

## Catkin configuration.  Allow finding of message/etc.
catkin_package(CATKIN_DEPENDS message_runtime)


###########
## Build ##
###########

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

## Declare the C++ executables
#add_executable(fkinnode  src/fkinnode.cpp)

## Add cmake target dependencies of the executables
#add_dependencies(fkinnode  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(fkinnode  ${catkin_LIBRARIES})
#target_link_libraries(fkinnode  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
