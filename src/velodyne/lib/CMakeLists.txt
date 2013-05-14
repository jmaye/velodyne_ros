remake_find_package(libvelodyne CONFIG)
remake_include(${LIBVELODYNE_INCLUDE_DIRS})

remake_ros_package_add_library(vdyne LINK ${LIBVELODYNE_LIBRARIES})
