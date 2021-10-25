cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(PROJECT_BASE_DIR ${PROJECT_SOURCE_DIR}/..)
set(FV_CORE_DIR ${PROJECT_BASE_DIR}/fv_core)
set(THIRDPARTY_DIR ${PROJECT_BASE_DIR}/3rdparty)
include_directories(${FV_CORE_DIR})
include_directories(${THIRDPARTY_DIR})

# ++++OPENCV-3.x++++
find_package(OpenCV 3 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.x++++
# find_package(OpenCV 2 REQUIRED)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.4.13++++
# # NOTE: Install OpenCV 2.4.13 on ~/.local from source which includes cv::fisheye
# # We don't use opencv found by any prior running of find_package
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
# set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
# find_package(OpenCV 2.4.13 REQUIRED)
# # restore CMAKE_PREFIX_PATH and other cached variables
# # in order to avoid other packages find this opencv.
# set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ----OPENCV----

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Building libraries:

rosbuild_add_library(fv_blob_tracker2
      ${FV_CORE_DIR}/blob_tracker2.cpp
      )
target_link_libraries(fv_blob_tracker2
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(fv_prox_vision
      ${FV_CORE_DIR}/prox_vision.cpp
      )
target_link_libraries(fv_prox_vision
      ${OpenCV_LIBRARIES}
      )
# WITH_STEREO:
# rosbuild_add_library(ayv_usb_stereo
#       ${THIRDPARTY_DIR}/ay_vision/usb_stereo.cpp
#       )
# target_link_libraries(ayv_usb_stereo
#       ${OpenCV_LIBRARIES}
#       )
rosbuild_add_library(ay_vision
      ${THIRDPARTY_DIR}/ay_vision/vision_util.cpp
      )
target_link_libraries(ay_vision
      ${OpenCV_LIBRARIES}
      )


#Building executables:

# NOTE: Adding libcv_bridge.so is a quick hack.
# In manifest.xml cv_bridge is omitted.
# If we put cv_bridge as a dependency in manifest.xml,
# both opencv 2.4.8 and 2.4.13 will be linked simultaneously,
# which causes a compile issue.

rosbuild_add_executable(fv_core_node
      src/fv_core_node.cpp
      )
target_link_libraries(fv_core_node
      fv_blob_tracker2
      fv_prox_vision
      ay_vision
      ${OpenCV_LIBRARIES}
      boost_thread
      boost_system
      )
