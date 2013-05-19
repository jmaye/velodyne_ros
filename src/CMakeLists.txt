remake_ros_package(
  velodyne
  DEPENDS roscpp sensor_msgs diagnostic_updater
  EXTRA_BUILD_DEPENDS libvelodyne-dev
  EXTRA_RUN_DEPENDS libvelodyne
  DESCRIPTION "Driver for Velodyne HDL devices."
)
