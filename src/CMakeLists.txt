remake_ros_package(
  velodyne
  DEPENDS roscpp rospy rosbash sensor_msgs diagnostic_updater std_msgs
  EXTRA_BUILD_DEPENDS libvelodyne-dev libsnappy-dev
  EXTRA_RUN_DEPENDS libvelodyne libsnappy
  DESCRIPTION "Driver for Velodyne HDL devices."
)
