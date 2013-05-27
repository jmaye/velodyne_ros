remake_find_package(libvelodyne CONFIG)
remake_find_package(libsnappy CONFIG)

remake_include(${LIBVELODYNE_INCLUDE_DIRS})
remake_include(${LIBSNAPPY_INCLUDE_DIRS})

remake_ros_package_add_library(vdyne LINK ${LIBVELODYNE_LIBRARIES}
  ${LIBSNAPPY_LIBRARIES})
