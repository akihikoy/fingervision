cmake_minimum_required(VERSION 3.0.2)
project(fingervision_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BlobMove.msg
  BlobMoves.msg
  Filter1ObjInfo.msg
  Filter1Wrench.msg
  Int32Array.msg
  NamedVariableList.msg
  NamedVariableListStamped.msg
  NamedVariable.msg
  ProxVision.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetInt32.srv
  SetStringInt32Array.srv
  SetStringInt32.srv
  SetString.srv
  TakeSnapshot.srv
)

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
)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Install environment is not configured.

## Testing method is not configured.

