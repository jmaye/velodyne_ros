cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  velodyne-ros
  VERSION 1:0.1
  RELEASE alpha
  SUMMARY "ROS driver for Velodyne HDL devices."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "http://github.com/jmaye/velodyne_ros"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX velodyne-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/velodyne
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_ros_pack_deb()

remake_ros_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE VELODYNE_ROS_GIT_REVISION
)

remake_ros_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:jmaye/ros
  PASS CMAKE_BUILD_TYPE VELODYNE_ROS_GIT_REVISION
)
