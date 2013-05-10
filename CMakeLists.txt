cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(CMAKE_CXX_FLAGS "-std=c++0x")

# libvelodyne
find_path(LIBVELODYNE_INCLUDE_DIR sensor/AcquisitionThread.h
  /usr/include
  PATH_SUFFIXES
  libvelodyne
)
message(Found ${LIBVELODYNE_INCLUDE_DIR})
find_library(LIBVELODYNE_SENSOR_LIBRARY velodyne-sensor
  /usr/lib
)
include_directories(
  ${LIBVELODYNE_INCLUDE_DIR}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/VelodyneNode.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBVELODYNE_SENSOR_LIBRARY})
rosbuild_add_executable(velodyne_node src/velodyne_node.cpp)
target_link_libraries(velodyne_node ${PROJECT_NAME})
