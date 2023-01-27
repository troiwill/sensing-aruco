cmake_minimum_required(VERSION 3.0.0)
project(sense_aruco_simulations)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  sense_aruco
  sense_aruco_ros
  aruco_marker_description
)

catkin_package()

install(DIRECTORY launch world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
