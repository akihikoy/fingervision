cmake_minimum_required(VERSION 3.0.2)
project(fingervision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  fingervision_msgs
  ay_vision
  ay_cpp
  roscpp
  std_msgs
  std_srvs
  image_transport
  camera_info_manager
  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_package(Eigen3 REQUIRED)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS ${FV_CORE_DIR}
  LIBRARIES fv_blob_tracker2 fv_prox_vision
  CATKIN_DEPENDS fingervision_msgs ay_vision ay_cpp
#  DEPENDS system_lib
)


set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(FV_CORE_DIR ${PROJECT_BASE_DIR}/fv_core)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${FV_CORE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(fv_blob_tracker2
  ${FV_CORE_DIR}/blob_tracker2.cpp
)
target_link_libraries(fv_blob_tracker2
  ${OpenCV_LIBRARIES}
)

add_library(fv_prox_vision
  ${FV_CORE_DIR}/prox_vision.cpp
)
target_link_libraries(fv_prox_vision
  ${OpenCV_LIBRARIES}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_fv_core_node
  src/fv_core_node.cpp
)
add_dependencies(${PROJECT_NAME}_fv_core_node
  ${fingervision_msgs_EXPORTED_TARGETS}
)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_fv_core_node PROPERTIES OUTPUT_NAME fv_core_node PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_fv_core_node
  fv_blob_tracker2
  fv_prox_vision
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
#   libcv_bridge.so
  boost_thread
  boost_system
)


## Install environment is not configured.

## Testing method is not configured.

