cmake_minimum_required(VERSION 3.0.2)
project(fv_gripper_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  tf
  ay_py
  ay_util
  fingervision_msgs
)


## Uncomment this if the package has a setup.py.
catkin_python_setup()


## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fv_gripper_ctrl
  CATKIN_DEPENDS ay_py ay_util
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Install environment is not configured.

## Testing method is not configured.

