remake_ros_package(
  velodyne
  DEPENDS roscpp diagnostic_updater
  EXTRA_BUILD_DEPENDS libvelodyne-dev
  EXTRA_RUN_DEPENDS libvelodyne
  DESCRIPTION "Driver for Velodyne HDL devices."
)
