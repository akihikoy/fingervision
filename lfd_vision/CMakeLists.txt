cmake_minimum_required(VERSION 2.4.6)
# # set(ENV{PKG_CONFIG_PATH} $ENV{HOME}/.local/lib/pkgconfig/:$ENV{PKG_CONFIG_PATH} )
# # find_package(PkgConfig)
# # pkg_check_modules(OPENCV opencv)
# include($ENV{HOME}/.local/share/OpenCV/OpenCVConfig.cmake)
# link_directories($ENV{HOME}/.local/lib)
# include_directories(${OpenCV_INCLUDE_DIRS})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# ++++OPENCV TRICK++++
# NOTE: Install latest OpenCV 2.4.13 on ~/.local from source which includes cv::fisheye
# don't use opencv as found by any prior running of find_package
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)
set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
find_package(OpenCV 2.4.13 REQUIRED)
# restore CMAKE_PREFIX_PATH and other cached variables
# so nothing other package finds this opencv
set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)
# ----OPENCV TRICK----

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# # set(CMAKE_MODULE_PATH $ENV{HOME}/.local/share/OpenCV/ ${CMAKE_MODULE_PATH})
# # include(OpenCVConfig)
# # include(OpenCVModules)
# # find_package(OpenCV 2.4.13 EXACT REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(disp_rostime
      src/disp_rostime.cpp )
target_link_libraries(disp_rostime ${OpenCV_LIBRARIES})

rosbuild_add_executable(cv_usb_node
      src/cv_usb_node.cpp
      src/vision_util.cpp)
target_link_libraries(cv_usb_node ${OpenCV_LIBRARIES} libcv_bridge.so)
# NOTE: libcv_bridge.so is a quick hack. In manifest.xml cv_bridge is omitted.
# Otherwise it causes linking opencv 2.4.8 and 2.4.13 simultaneously
# which caused other compile issue.

rosbuild_add_executable(visual_skin_node2
      src/visual_skin_node2.cpp
      src/blob_tracker2.cpp
      src/prox_vision.cpp
      src/usb_stereo.cpp
      src/vision_util.cpp
      src/pcl_util.cpp)
target_link_libraries(visual_skin_node2 ${OpenCV_LIBRARIES})
