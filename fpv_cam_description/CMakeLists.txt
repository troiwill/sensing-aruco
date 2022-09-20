cmake_minimum_required(VERSION 2.8.3)
project(fpv_cam_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
