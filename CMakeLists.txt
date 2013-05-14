cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  velodyne-ros
  VERSION 0.1
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
