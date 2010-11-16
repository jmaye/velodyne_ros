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

# libvelodyne
find_path(LIBVELODYNE_INCLUDE_DIR sensor/VelodynePointCloud.h
	/usr/local/include
	/usr/include
)
find_library(LIBVELODYNE_ACQUISITION_LIBRARY velodyne-acquisition
	/usr/local/lib
	/usr/lib
)
find_library(LIBVELODYNE_ACQUISITION_SENSOR velodyne-sensor
	/usr/local/lib
	/usr/lib
)

include_directories(${LIBVELODYNE_INCLUDE_DIR}/com ${LIBVELODYNE_INCLUDE_DIR}/sensor ${LIBVELODYNE_INCLUDE_DIR}/exceptions ${LIBVELODYNE_INCLUDE_DIR}/acquisition ${LIBVELODYNE_INCLUDE_DIR}/buffer)

rosbuild_add_executable(velodyne_hdl_64ES2 src/velodyne_hdl_64ES2.cpp)
target_link_libraries(velodyne_hdl_64ES2
	${LIBVELODYNE_ACQUISITION_LIBRARY}
	${LIBVELODYNE_ACQUISITION_SENSOR}
)
