cmake_minimum_required(VERSION 3.5.1)
project(rr_awsdeepracer)

list(APPEND CMAKE_PREFIX_PATH "/opt/aws/deepracer/share/ctrl_pkg")
find_package(catkin
  REQUIRED
  rospy
  actionlib
  sensor_msgs
  ctrl_pkg
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  ctrl_pkg
  sensor_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)