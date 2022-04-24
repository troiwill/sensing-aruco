cmake_minimum_required(VERSION 2.8.3)
project(sense_aruco)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
  cv_bridge)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/aruco_pose_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
