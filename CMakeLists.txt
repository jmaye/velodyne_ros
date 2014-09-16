cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  velodyne-ros-${ROS_DISTRIBUTION}
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

remake_var_name(GIT_REVISION_VAR ${REMAKE_PROJECT_NAME} GIT_REVISION)

remake_ros_pack_deb()

if(${ROS_DISTRIBUTION} STREQUAL fuerte)
  remake_ros_distribute_deb(
    DISTRIBUTION lucid
    SECTION libs
    DEPENDS remake pkg-config
    UPLOAD ppa:jmaye/ros
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif()

if(${ROS_DISTRIBUTION} STREQUAL fuerte OR ${ROS_DISTRIBUTION} STREQUAL groovy)
  remake_ros_distribute_deb(
    DISTRIBUTION precise
    SECTION libs
    DEPENDS remake pkg-config
    UPLOAD ppa:jmaye/ros
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif()

if(${ROS_DISTRIBUTION} STREQUAL indigo)
  remake_ros_distribute_deb(
    DISTRIBUTION trusty
    SECTION libs
    DEPENDS remake pkg-config
    UPLOAD ppa:jmaye/ros
    PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
    FORCE_CONSISTENCY
  )
endif()
